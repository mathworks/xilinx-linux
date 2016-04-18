/*
 * Copyright 2013-2016 The MathWorks, Inc.
 *
 */


#define COHERENT_MMAPPING
#include "mwadma.h"  /* IOCTL */
#include <linux/amba/xilinx_dma.h>
#include <linux/version.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>

dev_t mwadma_dev_id = 0;

static unsigned int device_num = 0;
static unsigned int cur_minor = 0;
static size_t channel_offset;
static struct class *mwadma_class = NULL;
static struct mwadma_tx_watermarks tx_wm;

char * mwdma_buf;
dma_addr_t mwdma_addr;
/*
 * Forward declaration of functions
 */
/*************************************************************************/
#ifndef ADI_XILINX_DMA
void xilinx_dma_reset(struct dma_chan *chan);
#endif
static int mw_axidma_setupchannel(struct mwadma_dev *mwdev,
        struct mwadma_chan *mwchan,
        struct mw_axidma_params *usrbuf);

static int mw_axidma_setupdesc(struct mwadma_dev *mwdev,
        struct mwadma_chan *mwchan,
        struct mw_axidma_params *usrbuf);

static int mw_axidma_alloc(struct mwadma_dev *mwdev, size_t bufferSize);

static int mwadma_mmap(struct file *fp, struct vm_area_struct *vma);

static void mwadma_free_channel(struct mwadma_dev *mwdev,
        struct mwadma_chan *mwchan);

static int mwadma_channel_probe(struct mwadma_dev *mwdev);

static void mwdma_test_loopback(struct mwadma_dev * mwdev,
        struct mw_axidma_params chn_prm);

int mwadma_start(struct mwadma_dev *mwdev,struct mwadma_chan *mwchan);
static irqreturn_t fifoirq_handler(int irq, void *data);
static int mwadma_fifo_probe(struct mwadma_dev *mwdev,struct mwadma_chan *mwchan);
static int mwadma_fifo_reset_configure(struct mwadma_chan *mwchan, int overflow_en, int underflow_en);
static int mwadma_fifo_poll_count(struct mwadma_chan *mwchan);

/*************************************************************************/
/*
 * @brief mwadma_reg_write
 */
void mwadma_reg_write(struct mwadma_dev *mwdev, unsigned reg, unsigned val)
{
    iowrite32(val, mwdev->regs + reg);
}

/*
 * @brief mwadma_reg_read
 */
unsigned int mwadma_reg_read(struct mwadma_dev *mwdev, unsigned reg)
{
    return (ioread32(mwdev->regs + reg));
}

/*
 * @brief mwadma_fasync_impl
 */
static int mwadma_fasync_impl(int fd, struct file* fp, int mode)
{
    struct mwadma_dev *mwdev = fp->private_data;
    return fasync_helper(fd, fp, mode, &mwdev->asyncq);
}

/*
 * @brief mwadma_open
 */
static int mwadma_open(struct inode *inode, struct file *fp)
{
    struct mwadma_dev *mwdev = fp->private_data;
    if (inode == NULL)
    {
        dev_dbg(&IP2DEV(mwdev), "INODE is NULL\n");
    }
    mwdev = container_of(inode->i_cdev, struct mwadma_dev, cdev);
    fp->private_data = mwdev;

    return 0;
}
/*
 * @brief mwadma_set_counter
 */
static int mwadma_set_counter(struct mwadma_dev *mwdev, unsigned val)
{
#ifndef LEGACY_COUNTER
    unsigned loop = 100;
    mwadma_reg_write(mwdev,SAMPLE_COUNTER_VAL_WR_REG, val);
    // Resets sample count, and AXI DMA Engine
    mwadma_reg_write(mwdev,SAMPLE_COUNTER_RESET_WR_REG, 0x1);

    while(loop--)
    {
        /*pr_info("RST VAL = %x",mwadma_reg_read(mwdev, SAMPLE_COUNTER_RESET_RD_REG));*/
        if(!mwadma_reg_read(mwdev, SAMPLE_COUNTER_RESET_RD_REG))
            break;

    }
    
    if(!loop)
    {
        dev_err(&IP2DEV(mwdev), "Counter reset took too long \n");
        return -EBUSY;
    }
#endif
    return 0;

}
/*
 * @brief mwadma_filterfn
 */
static bool mwadma_filterfn(struct dma_chan *chan, void *param)
{
#ifdef ADI_XILINX_DMA
    pr_debug(":chan->private=0x%p, chan_match=%d <--> param=0x%p, param_match=%d\n", chan->private,(int)chan->private, param, *(int*)param);
    if ((int)chan->private == *(int*)param) {
        pr_debug(":chan->private=0x%p, chan_match=%d <--> param=0x%p, param_match=%d\n", chan->private,(int)chan->private, param, *(int*)param);
        return true;
    }
#else
    pr_debug(":chan->private=0x%p, chan_match=%d <--> param=0x%p, param_match=%d\n", chan->private,*(int*)chan->private, param, *(int*)param);
    if (*(int*)chan->private == *(int*)param) {
        pr_debug(":chan->private=0x%p, chan_match=%d <--> param=0x%p, param_match=%d\n", chan->private,*(int*)chan->private, param, *(int*)param);
        return true;
    }
#endif
    return false;
}


/*
 * @brief mwadma_allocate_desc
 */
static int mwadma_allocate_desc(struct mwadma_slist **new, struct mwadma_chan *mwchan, unsigned int idx)
{
    struct scatterlist *this_sg;
    struct mwadma_slist * tmp;
    void *sg_buff;
    int ret, i = 0;
 
    tmp = (struct mwadma_slist *)kmalloc(sizeof(struct mwadma_slist),GFP_KERNEL);
    tmp->status = BD_UNALLOC;
    tmp->sg_t = (struct sg_table *)kmalloc(sizeof(struct sg_table),GFP_KERNEL);
    if (tmp->sg_t == NULL) {
        pr_err(DRIVER_NAME ": Error in sgtable KMALLOC\n");
        return -ENOMEM;
    }
    ret = sg_alloc_table(tmp->sg_t, mwchan->sg_entries, GFP_ATOMIC);
    if (ret) {
        pr_err(DRIVER_NAME ": Error in sg_alloc_table\n");
        sg_free_table(tmp->sg_t);
        return -ENOMEM;
    }
    if (mwchan->buf == NULL) {
        tmp->buf = (char*)__get_free_pages(GFP_KERNEL|__GFP_ZERO, get_order(mwchan->ring_bytes));
        pr_err(DRIVER_NAME ":Channel buffer was null. This should never happen.\n");
    }
    else {
        /* set buffer at offset from larger buffer */
        tmp->buf = &mwchan->buf[idx * mwchan->ring_bytes];
        // pr_dbg(DRIVER_NAME ":tmp->buf:0x%p\n",tmp->buf);
        tmp->buffer_index = idx;
    }
    for_each_sg(tmp->sg_t->sgl, this_sg, mwchan->sg_entries, i)
    {
        sg_buff = &(tmp->buf[(mwchan->bd_bytes)*i]);
        /* Is our requested size not a multiple of the buffer descriptor?
         * If so then let's make the last descriptor allocated exactly match
         * what we requested
         * */
        if(mwchan->requested_length < mwchan->bd_bytes)
        {
            sg_set_buf(this_sg,sg_buff,mwchan->requested_length);
            pr_info(DRIVER_NAME ":length = %d, sg-buff:0x%p\n",mwchan->requested_length, sg_buff);
            mwchan->requested_length = 0;
        }
        else if(mwchan->requested_length > 0)
        {
            sg_set_buf(this_sg,sg_buff,mwchan->bd_bytes);
            mwchan->requested_length -= mwchan->bd_bytes;
        }
        else
        {
            pr_err(DRIVER_NAME ": Invalid size for allocation\n");
        }
        // pr_dbg(DRIVER_NAME ":sg-buff:0x%p\n",sg_buff);
    }
    tmp->status = BD_ALLOC;
    *new = tmp;
    return 0;
}

/*
 * @brief mwadma_unmap_desc
 */
static void mwadma_unmap_desc(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan, struct mwadma_slist *input_slist)
{
    dma_unmap_sg(&IP2DEV(mwdev), input_slist->sg_t->sgl, mwchan->sg_entries, mwchan->direction);
    input_slist->status = BD_UNALLOC;
}

/*
 * @brief mwadma_map_desc
 */
static int mwadma_map_desc(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan, struct mwadma_slist *ipsl)
{
    int retVal;
    retVal = dma_map_sg(&IP2DEV(mwdev), ipsl->sg_t->sgl, mwchan->sg_entries, mwchan->direction);
    if (retVal == 0)  {
        dev_err(&IP2DEV(mwdev),"no buffers available\n");
        ipsl->status = BD_UNALLOC;
        return -ENOMEM;
    }
    ipsl->status = BD_MAPPED;
    return 0;
}

/*
 * @brief mwadma_prep_desc
 */
static int mwadma_prep_desc(struct mwadma_dev *mwdev, struct mwadma_chan * mwchan)
{
    unsigned int i = 0;
    int ret;
    struct mwadma_slist *new;

    /* First BD RING */
    ret = mwadma_allocate_desc(&(mwchan->scatter), mwchan, 0);
    if (ret < 0){
        dev_err(&IP2DEV(mwdev), "Failed in mwadma_allocate_desc");
        return -ENOMEM;
    }
    mwadma_map_desc(mwdev, mwchan, mwchan->scatter);
    /* New List of BD RING */
    INIT_LIST_HEAD(&(mwchan->scatter->list));
    for(i = 1; i < mwchan->ring_total; i++) /* POOL_SIZE - 1 */
    {
        ret = mwadma_allocate_desc(&(new), mwchan, i);
        if ((ret < 0) || (new == NULL)) {
            dev_err(&IP2DEV(mwdev), "Failed in mwadma_allocate_desc");
            return -ENOMEM;
        }
        list_add_tail(&(new->list),&(mwchan->scatter->list));
        mwadma_map_desc(mwdev, mwchan, new);
    }
    mwchan->curr = mwchan->scatter; /*Head of the list*/
    mwchan->prev = list_entry(mwchan->curr->list.prev, struct mwadma_slist, list);
    return 0;
}

void mwadma_tx_cb_single_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    struct device *dev = &IP2DEV(mwdev);
    static int ct = 1;
    spin_lock_bh(&mwchan->slock);
    mwchan->transfer_count++;
    mwchan->status = ready;
    spin_unlock_bh(&mwchan->slock);
    
    /* Signal userspace */
    if (likely(mwdev->asyncq))
    {
         kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT); 
    }
    dev_dbg(dev, "Notify from %s : count:%d\n",__func__,ct++);
    sysfs_notify(&dev->kobj, NULL, "dma_ch2");
}

void mwadma_tx_cb_continuous_signal_dataflow(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    long waiting_transfers;
    struct device *dev = &IP2DEV(mwdev);

    mwchan->transfer_count++;
    mwchan->completed = mwchan->prev;
    waiting_transfers = mwchan->transfer_queued;

    /*dev_dbg(dev, "Completed buffer %d to DMA, direction = %d\n",mwchan->completed->buffer_index,mwchan->direction);*/

    /* Make sure that the DMA engine is running when trying to deal with
     * interrupts. 
     * We must be careful when thinking about the number of queued transfers we
     * are reading since there is likely to be a DMA transfer in progress while
     * processing the interrupt from the last transfer */


    /*dev_info(dev,"Waiting transfers = %ld\n",waiting_transfers);*/
    if((mwchan->status == running) && (waiting_transfers > 0))
    {
        /* prepare to start waiting transfers */
        mwadma_start(mwdev,mwchan);
        /* Prepare next ring */
        if(waiting_transfers == tx_wm.full)
        {
            mwchan->error = TX_ERROR_QFULL;
            /*dev_info(dev, "Tx full\n");*/
        }
        else if(waiting_transfers >= tx_wm.prime)
        {
            mwchan->error = TX_ERROR_QPRIME;
            /*dev_info(dev, "Tx prime\n");*/
        }
        else
        {
            mwchan->error = TX_ERROR_QLOW;
            /*dev_info(dev, "Low tx condition\n");*/
        }
    }
    /* Even if we are only low here, we should not pre-emptively start a new transfer, this can be done by the enqueue function */
    else if((mwchan->status != finishing) && (waiting_transfers == 0))
    {
            mwchan->error = TX_ERROR_QLOW;
            mwchan->status = finishing; /* There will be at least one transfer completion still */
    }
    /* We've transmitted ALL pending transfers. Priming must start from 0 level */
    else 
    {
        mwchan->error = TX_ERROR_QUNDERFLOW;
        mwchan->status = waiting;
        dev_info(dev, "Tx software underflow: requires re-priming\n");

    }
    /* For I/O data flow no need to signal normal state */
    if(mwchan->error == TX_ERROR_QPRIME)
    {
        return;
    }
    /* Signal userspace */
    if (likely(mwdev->asyncq))
    {
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT); 
    }
    sysfs_notify(&dev->kobj, NULL, "dma_ch2");
}


void mwadma_tx_cb_continuous_free(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    struct device *dev = &IP2DEV(mwdev);
    static int ct = 1;
    spin_lock_bh(&mwchan->slock);
    mwchan->transfer_count++;
    mwchan->transfer_queued++; /* Always assume transfers queued */
    spin_unlock_bh(&mwchan->slock);
    
    mwadma_start(mwdev,mwchan);
    dev_dbg(dev, "Notify from %s : count:%d\n",__func__,ct++);
}
void mwadma_tx_cb_single_signal_burst(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    struct device *dev = &IP2DEV(mwdev);
    static int ct = 1;
    spin_lock_bh(&mwchan->slock);
    mwchan->transfer_count++;
    spin_unlock_bh(&mwchan->slock);
    
    if(mwchan->transfer_count < (mwchan->ring_total - 2))
    {
  
        mwadma_start(mwdev,mwchan);

    }
    if(mwchan->transfer_count == mwchan->ring_total)
    {
        mwchan->error = TX_ERROR_QUNDERFLOW;
        if (likely(mwdev->asyncq))
        {
            kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT); 
        }
        sysfs_notify(&dev->kobj, NULL, "dma_ch2");
    }
    dev_dbg(dev, "Notify from %s : count:%d\n",__func__,ct++);
}
void mwadma_tx_cb_continuous_signal_first(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    struct device *dev = &IP2DEV(mwdev);
    static int ct = 1;
    spin_lock_bh(&mwchan->slock);
    mwchan->transfer_count++;
    mwchan->transfer_queued++; /* Used in repeating transmit mode */
    spin_unlock_bh(&mwchan->slock);
    
    mwadma_start(mwdev,mwchan);
    if(mwchan->transfer_count == mwchan->ring_total)
    {
        mwchan->error = TX_ERROR_QUNDERFLOW;
        if (likely(mwdev->asyncq))
        {
            kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT); 
        }
        sysfs_notify(&dev->kobj, NULL, "dma_ch2");
    }
    dev_dbg(dev, "Notify from %s : count:%d\n",__func__,ct++);
}
void mwadma_tx_cb_continuous_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    long waiting_transfers;
    struct device *dev = &IP2DEV(mwdev);

    mwchan->transfer_count++;
    mwchan->completed = mwchan->prev;
    waiting_transfers = mwchan->transfer_queued;

    /*dev_dbg(dev, "Completed buffer %d to DMA, direction = %d\n",mwchan->completed->buffer_index,mwchan->direction);*/

    /* Make sure that the DMA engine is running when trying to deal with
     * interrupts. 
     * We must be careful when thinking about the number of queued transfers we
     * are reading since there is likely to be a DMA transfer in progress while
     * processing the interrupt from the last transfer */


    if((mwchan->status == running) && (waiting_transfers > 0))
    {
        /* prepare to start waiting transfers */
        mwadma_start(mwdev,mwchan);
        /* Prepare next ring */
        if(waiting_transfers == tx_wm.full)
        {
            mwchan->error = TX_ERROR_QFULL;
            /*dev_info(dev, "Tx full\n");*/
        }
        else if(waiting_transfers >= tx_wm.prime)
        {
            mwchan->error = TX_ERROR_QPRIME;
            /*dev_info(dev, "Tx prime\n");*/
        }
        else
        {
            mwchan->error = TX_ERROR_QLOW;
            /*dev_info(dev, "Low tx condition\n");*/
        }
    }
    /* If no new transfers are queued  in time to have continuous transmit */
    else if((mwchan->status != finishing) && (waiting_transfers == 0))
    {
            mwchan->error = TX_ERROR_QLOW;
            mwchan->status = finishing; /* Wait for pending transfer */
    }
    /* We've transmitted ALL pending transfers, now we can actually re-prime transfers */
    else 
    {
        mwchan->error = TX_ERROR_QUNDERFLOW;
        mwchan->status = waiting;
        dev_info(dev, "Tx software underflow: requires re-priming\n");

    }
    /* Signal userspace */
    if (likely(mwdev->asyncq))
    {
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT); 
    }
    sysfs_notify(&dev->kobj, NULL, "dma_ch2");
}


void mwadma_rx_cb_single_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->rx;
    struct device *dev = &IP2DEV(mwdev);
    static int ct = 1;

    spin_lock_bh(&mwchan->slock);
    mwchan->transfer_count++;
    mwchan->completed = mwchan->prev;
    mwchan->next_index = mwchan->completed->buffer_index;
    mwchan->status = ready;
    spin_unlock_bh(&mwchan->slock);
    
    mwchan->completed = mwchan->prev;    
    /* Signal userspace */    
    if (likely(mwdev->asyncq))
    {
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_IN); 
    }
    dev_dbg(dev, "Notify from %s : count:%d\n",__func__,ct++);
    sysfs_notify(&dev->kobj, NULL, "dma_ch1");

}

void mwadma_rx_cb_burst(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->rx;
    struct device *dev = &IP2DEV(mwdev);
    long current_transfers_queued, current_transfers_completed;

    spin_lock_bh(&mwchan->slock);
    mwchan->completed = list_entry(mwchan->prev->list.prev,struct mwadma_slist,list);
    mwchan->transfer_count++;
    current_transfers_queued = mwchan->transfer_queued;
    current_transfers_completed = mwchan->transfer_count;
    spin_unlock_bh(&mwchan->slock);

    if(current_transfers_completed != current_transfers_queued)
    {
        mwadma_start(mwdev,mwchan);
    }
    else
    {
        /* Done burst */
        if(likely(mwdev->asyncq))
        {
            /* Signal userspace */
            kill_fasync(&mwdev->asyncq, SIGIO, POLL_IN);
        }

        dev_dbg(dev, "Notify from %s : count:%ld\n",__func__,current_transfers_completed);
        sysfs_notify(&dev->kobj, NULL, "dma_ch1");
    }
    /* Start at the correct ring, user code is in charge of reading the correct number out*/
    if(current_transfers_completed == 1)
    {

        spin_lock_bh(&mwchan->slock);
        mwchan->next_index = mwchan->completed->buffer_index;
        spin_unlock_bh(&mwchan->slock);

    }
    mwadma_fifo_poll_count(mwchan);
}

void mwadma_rx_cb_pseudoburst_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->rx;
    struct device *dev = &IP2DEV(mwdev);
    long current_transfers_completed, current_transfers_queued;

    spin_lock_bh(&mwchan->slock);
    mwchan->completed = list_entry(mwchan->prev->list.prev,struct mwadma_slist,list);
    mwchan->transfer_count++;
    mwchan->transfer_queued++;
    current_transfers_completed = mwchan->transfer_count;
    current_transfers_queued = mwchan->transfer_queued;
    spin_unlock_bh(&mwchan->slock);

    if(unlikely(mwchan->status == ready))
    {
        return;
    }

    if(current_transfers_completed == 1) /* First transfer */
    {
        mwadma_start(mwdev,mwchan);

        spin_lock_bh(&mwchan->slock);
        mwchan->next_index = mwchan->completed->buffer_index;
        spin_unlock_bh(&mwchan->slock);

    }
    else if(current_transfers_queued >= (mwchan->ring_total - 2)) //Filled buffer
    {
        dev_dbg(dev, "Filled receive dma buffer:%s at %d\n", __func__, __LINE__);

        /* Stop until we start a new burst */
    }
    else
    {
        mwadma_start(mwdev,mwchan);
    }

    if(likely(mwdev->asyncq))
    {
        /* Signal userspace */
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_IN);
    }

    dev_dbg(dev, "Notify from %s : count:%ld\n",__func__,current_transfers_completed);
    sysfs_notify(&dev->kobj, NULL, "dma_ch1");
    mwadma_fifo_poll_count(mwchan);

}

void mwadma_rx_cb_continuous_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->rx;
    struct device *dev = &IP2DEV(mwdev);
    long current_transfers_completed, current_transfers_queued;
    if(unlikely(mwchan->status == ready))
    {
        return;
    }

    spin_lock_bh(&mwchan->slock);
    mwchan->completed = list_entry(mwchan->prev->list.prev,struct mwadma_slist,list);
    mwchan->transfer_count++;
    mwchan->transfer_queued++;
    current_transfers_completed = mwchan->transfer_count;
    current_transfers_queued = mwchan->transfer_queued;
    spin_unlock_bh(&mwchan->slock);

    mwadma_start(mwdev,mwchan);



    if(current_transfers_completed == 1) /* First time transfer */
    {

        spin_lock_bh(&mwchan->slock);
        mwchan->next_index = mwchan->completed->buffer_index;
        spin_unlock_bh(&mwchan->slock);

    }
    else if(current_transfers_queued > mwchan->ring_total) /* The completed transfer overflowed the buffer */
    {
        mwchan->error = ERR_RING_OVERFLOW;
        dev_dbg(dev, "Overflow condition:%s at %d\n", __func__, __LINE__);

        spin_lock_bh(&mwchan->slock);
        mwchan->transfer_queued = 0;
        /* We should start processing data from the completed transfer index
         * Might not be required, since application is going to process this
         * ring anyway. Just making it explicit
         */
        mwchan->next_index = mwchan->completed->buffer_index;
        spin_unlock_bh(&mwchan->slock);

    }

    if(likely(mwdev->asyncq))
    {
        /* Signal userspace */
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_IN);
    }
    dev_dbg(dev, "Notify from %s : count:%ld\n",__func__,current_transfers_completed);
    sysfs_notify(&dev->kobj, NULL, "dma_ch1");
    mwadma_fifo_poll_count(mwchan);
}

/*
 * @brief mwadma_start
 */
int mwadma_start(struct mwadma_dev *mwdev,struct mwadma_chan *mwchan)
{
    int ret = 0;
    struct mwadma_slist *new;
    if(mwdev == NULL) {
        ret = - ENODEV;
        goto start_failed;
    }
    spin_lock_bh(&mwchan->slock);
    /* Don't prepare a frame for DMA transmit if there are none queued */
    if(mwchan->direction == DMA_MEM_TO_DEV)
    {
        if(mwchan->transfer_queued > 0)
        {
            mwchan->transfer_queued--;
        }
        else
        {
            dev_err(&IP2DEV(mwdev), "Attempted to prepare transmit with no data available\n");
            ret = -EINVAL;
            spin_unlock_bh(&mwchan->slock);
            goto start_failed;
        }
    }
    /* Fresh buffer or has been used previously */
    /*dev_dbg(&IP2DEV(mwdev),"mwchan:0x%p, mwchan->chan:0x%p, DMA_CHAN:%s\n", \
            mwchan, mwchan->chan, dma_chan_name(mwchan->chan));        
        */
    mwchan->curr->desc = dmaengine_prep_slave_sg(mwchan->chan, mwchan->curr->sg_t->sgl, mwchan->sg_entries, mwchan->direction, mwchan->flags);
    if (mwchan->curr->desc == NULL)
    {
        mwadma_unmap_desc(mwdev, mwchan, mwchan->curr);
        dev_err(&IP2DEV(mwdev), "Failed to prep slave\n");
        ret = -ENOMEM;
        spin_unlock_bh(&mwchan->slock);
        goto start_failed;
    }
    mwchan->curr->desc->callback = mwchan->callback;
    mwchan->curr->desc->callback_param = mwdev;
    dmaengine_submit(mwchan->curr->desc);
    if (dma_submit_error(mwchan->curr->desc->cookie)) {
        dev_err(&IP2DEV(mwdev), "Failure to submit cookie\n");
        ret = -EINVAL;
        spin_unlock_bh(&mwchan->slock);
        goto start_failed;
    }
    /*dev_info(&IP2DEV(mwdev), "prep+submitted index = %d\n", mwchan->curr->buffer_index);*/
    new = list_entry(mwchan->curr->list.next,struct mwadma_slist,list);
    mwchan->prev = mwchan->curr;
    mwchan->curr = new;
    spin_unlock_bh(&mwchan->slock);
    return ret;

start_failed:
    return ret;
}

/*
 * @brief mwadma_stop
 */
static int mwadma_stop(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan)
{
    unsigned long loop = 10000;
    struct xilinx_dma_config config;
    config.coalesc = 0;
    config.delay = 0;
    dmaengine_device_control(mwchan->chan, DMA_TERMINATE_ALL, (unsigned long)&config);
#ifndef ADI_XILINX_DMA
    /* Make DMA engine aware that last transfer is interrupted */
    xilinx_dma_reset(mwchan->chan);
#endif
    while(loop--);
    dev_dbg(&IP2DEV(mwdev),"DMA STOP\nDirection = %d\nTransfers complete = %lu\n",mwchan->direction, mwchan->transfer_count);
    return 0;
}

/*
 * @brief mwadma_rx_ctl
 */
static long mwadma_rx_ctl(struct mwadma_dev *mwdev, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned long userval;
    struct mw_axidma_params usrbuf;
    struct mwadma_chan *mwchan = mwdev->rx;


#ifndef COHERENT_MMAPPING
    phys_addr_t startaddr = 0;
    phys_addr_t endaddr = 0;
#endif
    enum mwadma_chan_status     status;
    unsigned int                next_index;
    unsigned int                error;
    switch(cmd)
    {
        case MWADMA_SETUP_RX_CHANNEL:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                dev_err(&IP2DEV(mwdev),"Invalid Access\n");
                return -EACCES;
            }
            if (mwchan == NULL)
            {
                dev_err(&IP2DEV(mwdev),"Invalid Memory\n");
                return -ENOMEM;
            }
            ret = mw_axidma_setupchannel(mwdev, mwchan, &usrbuf);
            break;
        case MWADMA_UPDATE_RX_CHANNEL:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                dev_err(&IP2DEV(mwdev),"Invalid Access\n");
                return -EACCES;
            }
            if (mwchan == NULL)
            {
                dev_err(&IP2DEV(mwdev),"Invalid Memory\n");
                return -ENOMEM;
            }
            ret = mw_axidma_setupdesc(mwdev, mwchan, &usrbuf);
            break;
        case MWADMA_RX_SINGLE:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval)))
            {
                return -EACCES;
            }
            
            switch(userval)
            {
                case SIGNAL_TRANSFER_COMPLETE: 
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_single_signal;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_continuous_signal;
            }
            
            spin_lock(&mwchan->slock);
            mwchan->error = 0;
            mwchan->transfer_count = 0;
            spin_unlock(&mwchan->slock);
            mwadma_stop(mwdev, mwchan);
            /*xilinx_dma_reset(mwchan->chan);*/
            mwadma_set_counter(mwdev,mwchan->samples_per_interrupt);
            mwadma_start(mwdev,mwchan);
            dma_async_issue_pending(mwchan->chan);
            // check_completion(mwdev,mwchan);
            break;
        case MWADMA_RX_BURST:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof userval))
            {
                return -EACCES;
            }
            mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_burst;
            /* Start from the first */
            if(userval > mwchan->ring_total)
            {
                return -EINVAL;
            }

            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued = userval;
            mwchan->transfer_count = 0;
            mwchan->error = 0;
            spin_unlock_bh(&mwchan->slock);

            dev_dbg(&IP2DEV(mwdev), "Start DMA Burst of size %lu\n", userval);

            mwadma_stop(mwdev, mwchan);
            /*xilinx_dma_reset(mwchan->chan);*/
            mwadma_set_counter(mwdev, mwchan->samples_per_interrupt);
            mwadma_start(mwdev,mwchan);
            mwchan->status = running;
            dma_async_issue_pending(mwchan->chan);
            mwadma_start(mwdev,mwchan);
            break;
        case MWADMA_RX_PBURST:
            mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_pseudoburst_signal;
            /* Start from the first */

            spin_lock_bh(&mwchan->slock);
            mwchan->error = 0;
            mwchan->transfer_queued = 0;
            mwchan->transfer_count = 0;
            spin_unlock_bh(&mwchan->slock);

            dev_dbg(&IP2DEV(mwdev),"Start Pseudo Burst Rx DMA\n");
            /* At this point we require to reset the sample counter */
            mwadma_stop(mwdev, mwchan);
            /*xilinx_dma_reset(mwchan->chan);*/
            mwadma_set_counter(mwdev, mwchan->samples_per_interrupt);
            mwadma_start(mwdev,mwchan);
            dma_async_issue_pending(mwchan->chan);
            mwadma_start(mwdev,mwchan);

            spin_lock_bh(&mwchan->slock);
            mwchan->status = running;
            spin_unlock_bh(&mwchan->slock);

            break;
        case MWADMA_RX_CONTINUOUS:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof userval))
            {
                return -EACCES;
            }
            switch(userval)
            {
                case SIGNAL_TRANSFER_COMPLETE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_continuous_signal;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_continuous_signal;
                    //dev_dbg(&IP2DEV(mwdev), "No mode specified for Rx continuous\n");
            }

            /* Start from the first */
            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued = 0;
            mwchan->transfer_count = 0;
            mwchan->error = 0;
            spin_unlock_bh(&mwchan->slock);

            dev_dbg(&IP2DEV(mwdev),"Start Continuous Rx DMA\n");

            mwadma_stop(mwdev, mwchan);
            /*xilinx_dma_reset(mwchan->chan);*/
            mwadma_set_counter(mwdev,mwchan->samples_per_interrupt);
            mwadma_fifo_reset_configure(mwchan, 0, 0);
            mwadma_start(mwdev,mwchan);
            spin_lock_bh(&mwchan->slock);
            mwchan->status = running;
            spin_unlock_bh(&mwchan->slock);
            dma_async_issue_pending(mwchan->chan);
            mwadma_start(mwdev,mwchan);
            break;
        case MWADMA_RX_STOP:
            spin_lock_bh(&mwchan->slock);
            status = (unsigned long) mwchan->status;
            spin_unlock_bh(&mwchan->slock);

            if(status != ready)
            {
                ret = mwadma_stop(mwdev,mwchan);
                if (ret) {
                    dev_err(&IP2DEV(mwdev),"Error while stopping DMA\n");
                    return ret;
                }
            }

            mwadma_fifo_reset_configure(mwchan, 0, 0);
            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_count = 0;
            mwchan->status = ready;
            spin_unlock_bh(&mwchan->slock);
            break;
        case MWADMA_RX_GET_NEXT_INDEX:
            spin_lock_bh(&mwchan->slock);
            next_index = (unsigned long) mwchan->next_index;
            mwchan->transfer_queued--; /* user space has consumed a signal */
            mwchan->next_index = (mwchan->next_index + 1) % mwchan->buffer_interrupts;
            spin_unlock_bh(&mwchan->slock);
#ifndef COHERENT_MMAPPING
            startaddr = virt_to_phys(&mwchan->buf[next_index*mwchan->ring_bytes]);
            endaddr = virt_to_phys(&mwchan->buf[next_index*mwchan->ring_bytes + mwchan->ring_bytes]);
            outer_inv_range(startaddr, endaddr);
#endif
            /*dev_info(&IP2DEV(mwdev), "Next index = %d\n", next_index);*/
            if(copy_to_user((unsigned long *) arg, &next_index, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;
        case MWADMA_RX_GET_ERROR:

            spin_lock_bh(&mwchan->slock);
            error = mwchan->error;
            mwchan->error = 0;
            spin_unlock_bh(&mwchan->slock);
            if(mwchan->of_cnt)
            {
                dev_info(&IP2DEV(mwdev), "Rx hardware overflow, lost samples = %d\n", mwchan->of_cnt);
                error = ERR_RING_OVERFLOW;
                mwadma_fifo_reset_configure(mwchan, 0, 0);
            }
            if(copy_to_user((unsigned long *) arg, &error, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;
        case MWADMA_FREE_RX_CHANNEL:
            mwadma_free_channel(mwdev, mwchan);
            break;
        default:
            return 1;
    }
    return 0;
}


/*
 * @brief mwadma_tx_ctl
 */
static long mwadma_tx_ctl(struct mwadma_dev *mwdev, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct mwadma_chan *mwchan = mwdev->tx;
    unsigned long userval;
#ifndef COHERENT_MMAPPING
    phys_addr_t startaddr = 0;
    phys_addr_t endaddr = 0;
#endif

    enum mwadma_chan_status     status;
    long                        transfer_queued;
    struct mw_axidma_params usrbuf;
    struct mwadma_slist *new;
    switch(cmd)
    {
        case MWADMA_SETUP_TX_CHANNEL:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                return -EACCES;
            }
            if (mwchan  == NULL)
            {
                return -ENOMEM;
            }
            ret = mw_axidma_setupchannel(mwdev, mwchan, &usrbuf);
            /* set up watermarks for SDR */
            tx_wm.underflow = 0;
            tx_wm.low = 1;
            tx_wm.prime = 2;
            tx_wm.full = usrbuf.total_rings;
            tx_wm.overflow = tx_wm.full + 1;
            tx_wm.max = 0; /* Not using this variable */
            break;
        case MWADMA_UPDATE_TX_CHANNEL:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                return -EACCES;
            }
            if (mwchan  == NULL)
            {
                return -ENOMEM;
            }
            ret = mw_axidma_setupdesc(mwdev, mwchan, &usrbuf);
            /* set up watermarks for SDR */
            tx_wm.underflow = 0;
            tx_wm.low = 1;
            tx_wm.prime = 2;
            tx_wm.full = usrbuf.total_rings;
            tx_wm.overflow = tx_wm.full + 1;
            tx_wm.max = 0; /* Not using this variable */
            break;
        case MWADMA_TX_ENQUEUE:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval)))
            {
                return -EACCES;
            }
            if(userval == 0)
            {
                return -EINVAL;
            }

            spin_lock_bh(&mwchan->slock);
            status =  mwchan->status;
            transfer_queued = mwchan->transfer_queued;
            spin_unlock_bh(&mwchan->slock);
            /* If we can't queue the requested buffers then we indicate a full error and return */
            if((transfer_queued + (long)userval) > mwchan->ring_total)
            {
                /*dev_err(&IP2DEV(mwdev), \*/
                        /*":queue:%ld, user-queue:%lu, ring:%d\n", \*/
                        /*mwchan->transfer_queued, \*/
                        /*userval, \*/
                        /*mwchan->ring_total);*/
                spin_lock_bh(&mwchan->slock);
                mwchan->error = TX_ERROR_QFULL;
                spin_unlock_bh(&mwchan->slock);
                return -EBUSY;
            }

            // if dma is stopped and queued = 0
            // get next index from list
            // else
            // modulo increment
            if( (status == ready) && (transfer_queued == 0))
            {
                spin_lock_bh(&mwchan->slock);
                mwchan->next_index =  mwchan->curr->buffer_index;
                spin_unlock_bh(&mwchan->slock);
            }
            else
            {
                spin_lock_bh(&mwchan->slock);
                mwchan->next_index = (mwchan->next_index + 1) % mwchan->ring_total;
                spin_unlock_bh(&mwchan->slock);

            }
#ifndef COHERENT_MMAPPING
            startaddr = virt_to_phys(&mwchan->buf[mwchan->next_index*mwchan->ring_bytes]);
            endaddr = virt_to_phys(&mwchan->buf[mwchan->next_index*mwchan->ring_bytes + mwchan->ring_bytes]);
            __cpuc_flush_dcache_area(&mwchan->buf[mwchan->next_index*mwchan->ring_bytes], mwchan->ring_bytes);
            outer_flush_range(startaddr,endaddr);
            /*dma_sync_single_range_for_device(&IP2DEV(mwdev),mwdev->phys,startaddr - mwdev->phys, mwchan->ring_bytes, mwchan->direction);*/

            /*dev_info(&IP2DEV(mwdev), "Synced ring index = %d\n", mwchan->next_index);*/
#endif

            /*dev_info(&IP2DEV(mwdev),"Transfers queued = %ld\n", transfer_queued + (long)userval);*/

            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued += (long)userval;
            transfer_queued = mwchan->transfer_queued;
            status =  mwchan->status;
            spin_unlock_bh(&mwchan->slock);

            /* restart if required (only appropriate for continuous mode) */
            /* A final transfer is in progress and we have a chance to prepare a new one */
            if(status == finishing && transfer_queued > 0)
            {
                mwadma_start(mwdev,mwchan);
                mwchan->status = running;
                /*dev_info(&IP2DEV(mwdev),"Restart transmit\n");*/
            }
            /* There are no transfers in progress, and we need to re-prime thwe buffers completely */
            else if(unlikely((status == waiting) && (transfer_queued >= tx_wm.prime))) /* restart/reprime */
            {
                mwchan->status = priming; /* Fill first two buffers */
                mwadma_start(mwdev,mwchan);
                dma_async_issue_pending(mwchan->chan);
                mwadma_start(mwdev,mwchan);
                if (mwchan->status == priming)
                {
                    mwchan->status = running; /*Data ready */
                }
                else
                {
                    mwchan->status = waiting;
                }
                dev_dbg(&IP2DEV(mwdev),"Tx re-primed\n");
            }
            break;
        case MWADMA_TX_SINGLE:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval)))
            {
                return -EACCES;
            }
            switch(userval)
            {
                case SIGNAL_TRANSFER_COMPLETE: 
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal;
                    break;
                case SIGNAL_BURST_COMPLETE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal_burst;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal;
            }
            if((userval == SIGNAL_BURST_COMPLETE))
            {
#ifndef COHERENT_MMAPPING
                    startaddr = virt_to_phys(&mwchan->buf[0]);
                    endaddr = virt_to_phys(&mwchan->buf[mwchan->length]);
                    __cpuc_flush_dcache_area(&mwchan->buf[0], mwchan->length);
                    outer_flush_range(startaddr,endaddr);
#endif
                    mwchan->status = priming; /* Fill first two buffers */
                    mwadma_start(mwdev,mwchan);
                    dma_async_issue_pending(mwchan->chan);
                    mwadma_start(mwdev,mwchan);
                    if (mwchan->status == priming)
                    {
                        mwchan->status = running; /*Data ready */
                    }
                    else
                    {
                        mwchan->status = waiting;
                    }
                    return 0;
            }
            spin_lock_bh(&mwchan->slock);
            transfer_queued = mwchan->transfer_queued;
            spin_unlock_bh(&mwchan->slock);
            if (!transfer_queued)
            {
                dev_err(&IP2DEV(mwdev),"Queue is empty\n");
                return -EINVAL;
            }
            spin_lock_bh(&mwchan->slock);
            mwchan->next_index = (mwchan->next_index + 1) % mwchan->ring_total;            
            spin_unlock_bh(&mwchan->slock);
            mwadma_start(mwdev,mwchan);
            dma_async_issue_pending(mwchan->chan);
            break;
        case MWADMA_TX_CONTINUOUS:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof userval))
            {
                return -EACCES;
            }
            switch(userval)
            {
                case SIGNAL_TRANSFER_COMPLETE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_signal;
                    break;
                case SIGNAL_DATAFLOW:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_signal_dataflow;
                    break;
                case SIGNAL_ONCE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_signal_first;
                    break;
                case SIGNAL_OFF:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_free;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_signal;
                    dev_dbg(&IP2DEV(mwdev),"No mode specified for Tx continuous\n");
            }
            if((userval == SIGNAL_ONCE) || (userval == SIGNAL_OFF))
            {
#ifndef COHERENT_MMAPPING
                    startaddr = virt_to_phys(&mwchan->buf[0]);
                    endaddr = virt_to_phys(&mwchan->buf[mwchan->length]);
                    __cpuc_flush_dcache_area(&mwchan->buf[0], mwchan->length);
                    outer_flush_range(startaddr,endaddr);
#endif
                    mwadma_fifo_reset_configure(mwchan, 0, 0);
                    mwchan->status = priming; /* Fill first two buffers */
                    mwadma_start(mwdev,mwchan);
                    dma_async_issue_pending(mwchan->chan);
                    mwadma_start(mwdev,mwchan);
                    if (mwchan->status == priming)
                    {
                        mwchan->status = running; /*Data ready */
                    }
                    else
                    {
                        mwchan->status = waiting;
                    }
                    return 0;
            }

            spin_lock_bh(&mwchan->slock);
            transfer_queued = mwchan->transfer_queued;
            spin_unlock_bh(&mwchan->slock);

            mwadma_fifo_reset_configure(mwchan, 0, 0);
            if(transfer_queued >= tx_wm.prime) /* restart if required */
            {
                dev_dbg(&IP2DEV(mwdev),"Starting from fill level = %ld\n",transfer_queued);

                mwchan->status = priming; /* Fill first two buffers */
                mwadma_start(mwdev,mwchan);
                dma_async_issue_pending(mwchan->chan);
                mwadma_start(mwdev,mwchan);
                if (mwchan->status == priming)
                {
                    mwchan->status = running; /*Data ready */
                }
                else
                {
                    mwchan->status = waiting;
                }
            }
            else
            {
                spin_lock_bh(&mwchan->slock);
                mwchan->status = waiting; /* Still not enough data in the buffers */
                spin_unlock_bh(&mwchan->slock);
            }
            break;
        case MWADMA_TX_STOP:
            spin_lock_bh(&mwchan->slock);
            status = mwchan->status;
            spin_unlock_bh(&mwchan->slock);
#ifndef COHERENT_MMAPPING
            startaddr = virt_to_phys(&mwchan->buf[0]);
            endaddr = virt_to_phys(&mwchan->buf[mwchan->length]);
            __cpuc_flush_dcache_area(&mwchan->buf[0], mwchan->length);
            outer_flush_range(startaddr,endaddr);
#endif

            if(status != ready)
            {

                ret = mwadma_stop(mwdev,mwchan);
                if(ret)
                {
                    dev_err(&IP2DEV(mwdev),"Error while stopping DMA\n");
                    return ret;
                }

                spin_lock_bh(&mwchan->slock);
                mwchan->status = ready;
                spin_unlock_bh(&mwchan->slock);

            }

            mwadma_fifo_reset_configure(mwchan, 0, 0);
            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued = 0; /* Reset pending transfers */
            mwchan->transfer_count  = 0;
            spin_unlock_bh(&mwchan->slock);
            break;
        case MWADMA_TX_GET_ERROR:
            /*dev_info(&IP2DEV(mwdev), "Requested Tx error status = %d\n",mwchan->error);*/

            userval = mwchan->error; /* error code */
            mwchan->error = TX_ERROR_QPRIME; /* error code */
            mwadma_fifo_poll_count(mwchan); /* read register from FIFO */
            if(mwchan->uf_cnt)
            {
                dev_info(&IP2DEV(mwdev), "Tx hardware underflow, skipped samples = %d\n", mwchan->uf_cnt);
                /* Reset the fifo core */
                userval = TX_ERROR_QUNDERFLOW;
                mwchan->uf_cnt = 0;
                mwadma_fifo_reset_configure(mwchan, 0, 0);
            }

            if(copy_to_user((unsigned long *) arg, &userval, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;
        case MWADMA_TX_GET_NEXT_INDEX:
            spin_lock_bh(&mwchan->slock);
            new = list_entry(mwchan->curr->list.next,struct mwadma_slist,list);
            userval = (unsigned long) new->buffer_index;
            /*dev_info(&IP2DEV(mwdev), "Next index = %d\n", userval);*/
            spin_unlock_bh(&mwchan->slock);
            if(copy_to_user((unsigned long *) arg, &userval, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;            
        case MWADMA_FREE_TX_CHANNEL:
             mwadma_free_channel(mwdev, mwchan);
            break;
        default:
            return 1;
    }
    return 0;
}

/*
 * @brief mwadma_generic_ctl
 */
static long mwadma_generic_ctl(struct mwadma_dev *mwdev, unsigned int cmd, unsigned long arg)
{
    struct mw_axidma_params usrbuf;
    switch(cmd)
    {
        case MWADMA_GET_PROPERTIES:
            usrbuf.size = mwdev->size;
            usrbuf.phys = (dma_addr_t)mwdev->phys;
            if(copy_to_user((struct mw_axidma_params *)arg, &usrbuf, sizeof(struct mw_axidma_params))) {
                return -EACCES;
            }
            break;
        case MWADMA_TEST_LOOPBACK:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                return -EACCES;
            }
            mwdma_test_loopback(mwdev, usrbuf);
            break;
        default:
            return 1;
    }
    return 0;
}

/*
 * @brief mwadma_ioctl
 */
static long mwadma_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret_rx = 0;
    int ret_tx = 0;
    int ret_generic = 0;
    struct mwadma_dev *mwdev = fp->private_data;

    if (NULL == mwdev)
    {
        return -ENODEV;
    }

    ret_rx = mwadma_rx_ctl(mwdev,cmd,arg);
    ret_tx = mwadma_tx_ctl(mwdev,cmd,arg);
    ret_generic = mwadma_generic_ctl(mwdev,cmd,arg);
    /* Errors */
    if(ret_rx < 0)
    {
        return ret_rx;
    }
    if(ret_tx < 0) 
    {
        return ret_tx;
    }
    if(ret_generic < 0)
    {
        return ret_generic;
    }

    /* No valid case found */
    if(3 == (ret_rx + ret_tx + ret_generic))
    {
        dev_dbg(&IP2DEV(mwdev), "Invalid ioctl: command: %u\n", cmd);
        return -EINVAL;
    }
    return 0;
}


/*
 * @brief mwadma_close
 */
static int mwadma_close(struct inode *inode, struct file *fp)
{
    struct mwadma_dev *mwdev = fp->private_data;
    int ret = 0;

        dev_dbg(&IP2DEV(mwdev), "Closing the file-descriptor for %s\n", DRIVER_NAME);

    if (NULL == mwdev)
    {
        return -ENODEV;
    }
    mwadma_fasync_impl(-1, fp, 0);
    return ret;
}

/*
 * @brief mwadma_mmap_dma_open
 */
static void mwadma_mmap_dma_open(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(&IP2DEV(mwdev), "DMA VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

/*
 * @brief mwadma_free_channel
 */
static void mwadma_free_channel(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan)
{
    struct mwadma_slist *slist, *_slist;
    struct xilinx_dma_config config;

    unsigned long flags;
    mwadma_stop(mwdev, mwchan);
    spin_lock_irqsave(&mwchan->slock, flags);
    /*dma_release_channel(mwchan->chan);*/
    list_for_each_entry_safe(slist, _slist, &mwchan->scatter->list, list) {
        mwadma_unmap_desc(mwdev, mwchan, slist);
        sg_free_table(slist->sg_t);
        kfree(slist->sg_t);
        list_del(&slist->list);
        kfree(&slist->list);
    }
    spin_unlock_irqrestore(&mwchan->slock, flags);
    dev_dbg(&IP2DEV(mwdev), "MWADMA Free channel done.");
}

/*
 * @brief mwadma_mmap_dma_close
 */
static void mwadma_mmap_dma_close(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(&IP2DEV(mwdev), "DMA VMA close.\n");
	/* Free the memory DMA */
    if (mwdev->size) {
        dev_info(&IP2DEV(mwdev), "free dma memory.\n");
        dma_free_coherent(&IP2DEV(mwdev), mwdev->size, mwdev->virt, mwdev->phys);
        mwdev->size = 0;
        channel_offset = 0;
        mwdev->virt = NULL;
        mwdev->phys = 0;
    }
}

/*
 * @brief mwadma_mmap_open
 */
static void mwadma_mmap_open(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(&IP2DEV(mwdev), "Simple VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

/*
 * @brief mwadma_mmap_close
 */
static void mwadma_mmap_close(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(&IP2DEV(mwdev ), "Simple VMA close.\n");
}

/*
 * @brief mwadma_mmap_fault
 */
static int mwadma_mmap_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
    struct page *thisPage;
    unsigned long offset;
    offset = (vmf->pgoff - vma->vm_pgoff) << PAGE_SHIFT;
    thisPage = virt_to_page(mwdev->mem->start + offset);
    get_page(thisPage);
    vmf->page = thisPage;
    return 0;
}

struct vm_operations_struct  mwadma_mmap_ops = {
    .open           = mwadma_mmap_open,
    .close          = mwadma_mmap_close,
    .fault          = mwadma_mmap_fault,
}; 

struct vm_operations_struct mwadma_mmap_dma_ops = {
    .open           = mwadma_mmap_dma_open,
    .close          = mwadma_mmap_dma_close,
};


struct file_operations mwadma_cdev_fops = {
    .owner = THIS_MODULE,
    .open = mwadma_open,
    .fasync = mwadma_fasync_impl,
    .release = mwadma_close,
    .mmap		    = mwadma_mmap,
    .unlocked_ioctl = mwadma_ioctl,
};

/*
 * @brief mwadma_mmap
 */
static int mwadma_mmap(struct file *fp, struct vm_area_struct *vma)
{
    struct mwadma_dev *mwdev = fp->private_data;
    size_t size = vma->vm_end - vma->vm_start;
    int status = 0;
    vma->vm_private_data = mwdev;
    dev_info(&IP2DEV(mwdev), "[MMAP] size:%X pgoff: %lx\n", size, vma->vm_pgoff);
 
    switch(vma->vm_pgoff) {
		case 0: 
			/* mmap the Memory Mapped I/O's base address */
                        #if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
                            vma->vm_flags |= VM_IO | VM_RESERVED;
                        #else
                            vma->vm_flags |= VM_IO | VM_DONTDUMP;
			#endif
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			if (remap_pfn_range(vma, vma->vm_start,
					mwdev->mem->start >> PAGE_SHIFT,
					size,
					vma->vm_page_prot))
			{
				return -EAGAIN;
			}
			vma->vm_ops = &mwadma_mmap_ops;
			break;
		default:
                      /* mmap the DMA region */
                      status = mw_axidma_alloc(mwdev, size);
                     if ((status) && (status != -EEXIST))  {
                         return -ENOMEM;
                     }
                    dev_dbg(&IP2DEV(mwdev), "dma setup_cdev successful\n");

                    status = 0;
			if (mwdev->virt == NULL){
                            return -EINVAL;
                        }
                    vma->vm_pgoff = 0;
#ifndef COHERENT_MMAPPING
                    status = remap_pfn_range(vma, vma->vm_start,
                            __pa(mwdev->virt) >> PAGE_SHIFT, size,
                            vma->vm_page_prot);
#else
                    status = dma_mmap_coherent(&IP2DEV(mwdev), vma, mwdev->virt,
                        mwdev->phys, mwdev->size);
#endif
                    if (status) {
                        dev_dbg(&IP2DEV(mwdev),"Remapping memory failed, error: %d\n", status);
                        return status;
                    }
                    vma->vm_ops = &mwadma_mmap_dma_ops;
                    dev_dbg(&IP2DEV(mwdev),"%s: mapped dma addr 0x%08lx at 0x%08lx, size %d\n",
                          __func__, (unsigned long)mwdev->phys, vma->vm_start,
                          mwdev->size);
                     break;
    }
	return status;
}


/*
 * @brief mw_axidma_alloc
 */
static int mw_axidma_alloc(struct mwadma_dev *mwdev, size_t bufferSize)
{
    if (mwdev == NULL)
    {
        return -ENOMEM;
    }
    if (mwdev->virt != NULL) 
    {
		dev_err(&IP2DEV(mwdev), "DMA memory already allocated\n");		
		return -EEXIST;
	}
    mwdev->virt = dma_zalloc_coherent(&IP2DEV(mwdev), bufferSize, \
            &mwdev->phys, \
            GFP_KERNEL);
    if (mwdev->virt == NULL)
    {
        dev_err(&IP2DEV(mwdev), "Failed to allocate continguous memory\nUsing multiple buffers\n");
    }
    
    else {
        dev_info(&IP2DEV(mwdev), "Address of buffer = 0x%p, Length = %u Bytes\n",\
                (void *)virt_to_phys(mwdev->virt),bufferSize);
        mwdev->size = bufferSize;
    }
    return 0;
}

/*
 * @brief mw_axidma_setupdesc
 */
static int mw_axidma_setupdesc(struct mwadma_dev *mwdev, 
        struct mwadma_chan *mwchan,
        struct mw_axidma_params *usrbuf)
{
    int status = 0;
   
 
    if ( (mwdev == NULL) || (mwchan == NULL) ) {
        return -EINVAL;
    }
    mwchan->ring_total            = usrbuf->total_rings;
    mwchan->length                = usrbuf->bytes_per_ring * usrbuf->total_rings;
    mwchan->requested_length      = usrbuf->size;

    mwchan->bd_bytes              = usrbuf->desc_length;
    mwchan->ring_bytes            = usrbuf->bytes_per_ring;
    mwchan->sg_entries            = usrbuf->bytes_per_ring/usrbuf->desc_length;
    mwchan->buffer_interrupts     = mwchan->ring_total;
    mwchan->samples_per_interrupt = usrbuf->counter;
    
    if(mwchan->requested_length < mwchan->length)
    {
        /* Canary */
        mwchan->buf[mwchan->requested_length + 1] = 0xAA;
        dev_dbg(&IP2DEV(mwdev), "Canary at init = %x\n",mwchan->buf[mwchan->requested_length + 1]);
    }
    /* Write to the IPCore_PacketSize_AXI4_Stream_Master 0x8 to specify the length for recieve channels*/
    if(mwchan->direction == DMA_DEV_TO_MEM){
        mwadma_set_counter(mwdev, mwchan->samples_per_interrupt);
    }
    dev_info(&IP2DEV(mwdev), "### Printing Channel info...\n");
    dev_info(&IP2DEV(mwdev), "Virtual Address        :0x%p\n", mwchan->buf);
    dev_info(&IP2DEV(mwdev), "Channel Length/Size    :%lu\n", mwchan->length);
    dev_info(&IP2DEV(mwdev), "Requested Length/Size  :%d\n", mwchan->requested_length);
    dev_info(&IP2DEV(mwdev), "Channel direction      :%d\n", mwchan->direction);
    dev_info(&IP2DEV(mwdev), "Total number of rings  :%d\n", mwchan->ring_total);
    dev_info(&IP2DEV(mwdev), "Bytes per ring         :%d\n", mwchan->ring_bytes);
    dev_info(&IP2DEV(mwdev), "Buffer Descriptor size :%d\n", mwchan->bd_bytes);
    dev_info(&IP2DEV(mwdev), "Channel SG Entries     :%d\n", mwchan->sg_entries);
    dev_info(&IP2DEV(mwdev), "Buffer Interrupts      :%d\n", mwchan->buffer_interrupts);  
    /* Get channel for DMA */
    dev_dbg(&IP2DEV(mwdev),"Name:%s, mwchan:0x%p, mwchan->chan:0x%p\n", 
            dma_chan_name(mwchan->chan), mwchan, mwchan->chan);
    
    if (mwchan->ring_total >= 2) {
        status = mwadma_prep_desc(mwdev, mwchan);
    } else
    {
        /*
         * Instantiate scatter structure, it contains cookie, desc, callback
         */
        mwchan->scatter = devm_kzalloc(&IP2DEV(mwdev), 
                sizeof(struct mwadma_slist), GFP_KERNEL);
    }
    return status;
}
/*
 * @brief mw_axidma_setupchannel
 */
static int mw_axidma_setupchannel(struct mwadma_dev *mwdev, 
        struct mwadma_chan *mwchan,
        struct mw_axidma_params *usrbuf)
{
    int status = 0;
    static int idx = 0;
    struct xilinx_dma_config    config;
    if ( (mwdev == NULL) || (mwchan == NULL) ) {
        return -EINVAL;
    }
    mwchan->flags                 = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
    mwchan->ring_total            = usrbuf->total_rings;
    mwchan->length                = usrbuf->bytes_per_ring * usrbuf->total_rings;

    mwchan->buf                 = &(mwdev->virt[channel_offset]);
    if (mwchan->buf == NULL) {
        dev_err(&IP2DEV(mwdev), "Buffer is NULL. Failed to allocate memory\n");
        return -ENOMEM;
	}
    mwchan->offset              =  channel_offset;
    channel_offset              =  channel_offset + mwchan->length;
    /*
     * Set channel-index : used to notify appropriate DMA_CHX SYFS node
     */
    mwchan->chan_id             =  idx;
    idx++;
    /* Get channel for DMA */
    mutex_init(&mwchan->lock);
    config.coalesc = 0;
    config.delay = 0;
    dmaengine_device_control(mwchan->chan, DMA_SLAVE_CONFIG, (unsigned long)&config);
    init_completion(&mwchan->dma_complete);
    spin_lock_init(&mwchan->slock);

    return mw_axidma_setupdesc(mwdev, mwchan, usrbuf);
}

/*
 * @brief mwadma_setup_cdev
 */
static int mwadma_setup_cdev(struct mwadma_dev *mwdev, dev_t *dev_id)
{
    int status = 0;
    struct device * thisDevice;
    cdev_init(&mwdev->cdev, &mwadma_cdev_fops);
    mwdev->cdev.owner = THIS_MODULE;
    mwdev->cdev.ops = &mwadma_cdev_fops;
    *dev_id = MKDEV(MAJOR(mwadma_dev_id), cur_minor++);
    status = cdev_add(&mwdev->cdev, *dev_id, 1);

    if (status < 0) {
        return status;
    }
    if (device_num == 0) {
        thisDevice = device_create(mwadma_class, NULL, *dev_id, NULL, "%s", DRIVER_NAME);
    } else {
        thisDevice = device_create(mwadma_class, NULL, *dev_id, NULL, "%s%d", DRIVER_NAME, device_num);
    }
    if(IS_ERR(thisDevice))
    {
        status = PTR_ERR(thisDevice);
        dev_err(&IP2DEV(mwdev), "Error: failed to create device node %s, err %d\n", mwdev->name, status);
        cdev_del(&mwdev->cdev);
    }
    return status;
}

static void mwdma_test_loopback(struct mwadma_dev * mwdev, 
        struct mw_axidma_params chan_prm)
{
    int i = 0;
    size_t len;
    char *dma_addr = mwdev->virt;  
    unsigned int *tmp;
    /* rx = &dma_addr[0];
     * tx = &dma_addr[chan_prm.size];
     */
    dev_dbg(&IP2DEV(mwdev),"### test loopback\n");

    len = chan_prm.size;
    /* prime the rx & tx buffers */
    tmp = (unsigned int *) dma_addr;
    for (i=0;i<(len/sizeof(unsigned int));i++)
    {
        tmp[i] = 0xDEADC0DE;
    }
    tmp = (unsigned int *) (dma_addr + len);
    for (i=0;i<(len/sizeof(unsigned int));i++)
    {
        tmp[i] = (i+1) % (chan_prm.bytes_per_ring/sizeof(unsigned int));
    }
    /* Receive single ring */
    mwdev->rx->callback = (dma_async_tx_callback)mwadma_rx_cb_single_signal;
    mwdev->rx->error = 0;
    mwdev->rx->transfer_count = 0;
    mwadma_start(mwdev, mwdev->rx);
    dma_async_issue_pending(mwdev->rx->chan);
    /* Transmit single ring */
    mwdev->tx->transfer_queued += 1;
    mwdev->tx->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal;
    mwdev->tx->next_index = (mwdev->tx->next_index + 1) % mwdev->tx->ring_total;
    mwadma_start(mwdev,mwdev->tx);
    dma_async_issue_pending(mwdev->tx->chan);
}


static ssize_t mwdma_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    dev_dbg(dev,"sysfs_notify :%s\n", attr->attr.name);
    // sysfs_notify(&dev->kobj, NULL, attr->attr.name);
    return (sizeof(int));
}

static ssize_t mwdma_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    dev_dbg(dev, "Showing channel %s\n",attr->attr.name);
    return sprintf(buf, "%s\n", attr->attr.name);
}

static DEVICE_ATTR(dma_ch1, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch2, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch3, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch4, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch5, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch6, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch7, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch8, S_IRUGO, mwdma_show, mwdma_store);

static struct attribute *mwdma_attributes[] = {
    &dev_attr_dma_ch1.attr,
    &dev_attr_dma_ch2.attr,
    &dev_attr_dma_ch3.attr,
    &dev_attr_dma_ch4.attr,
    &dev_attr_dma_ch5.attr,
    &dev_attr_dma_ch6.attr,
    &dev_attr_dma_ch7.attr,
    &dev_attr_dma_ch8.attr,
    NULL,
};

static const struct attribute_group mwdma_attr_group = {
    .attrs = mwdma_attributes,
};

static int mwadma_channel_probe(struct mwadma_dev *mwdev)
{
    struct dma_chan *rxchan,*txchan;
    static char id = 0;
    dma_cap_mask_t mask;

    u32 match_rx, match_tx;
    
    match_tx  = (DMA_MEM_TO_DEV & 0xFF) | XILINX_DMA_IP_DMA;
    match_rx  = (DMA_DEV_TO_MEM & 0xFF) | XILINX_DMA_IP_DMA;
    
    dma_cap_zero(mask);
    dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);
#ifdef ADI_XILINX_DMA
    dma_cap_set(DMA_CYCLIC, mask);
#endif

    dev_info(&IP2DEV(mwdev), "probing channel\n");
    
    if (id > 1) {
        dev_dbg(&IP2DEV(mwdev), "Exceeded maximum allowable RX-TX DMA channel pairs\n");
        return -ENOMEM;
    }
    
    while (1) {
       dev_dbg(&IP2DEV(mwdev), "requesting tx-channel\n");
       txchan = dma_request_channel(mask, mwadma_filterfn,
                (void *)&match_tx);
       dev_dbg(&IP2DEV(mwdev), "requesting rx-channel\n");
       rxchan = dma_request_channel(mask, mwadma_filterfn,
                (void *)&match_rx);
          if (!txchan && !rxchan) {
            dev_info(&IP2DEV(mwdev), "no more channels found!\n");
            break;
        } else {
            dev_info(&IP2DEV(mwdev), "some channels found!\n");
            if (txchan) {
                mwdev->tx = (struct mwadma_chan*)devm_kzalloc(&IP2DEV(mwdev), 
                        sizeof(struct mwadma_chan),GFP_KERNEL);
                if (mwdev->tx == NULL) {
                    dev_err(&IP2DEV(mwdev), "failed to allocate tx channel\n");
                    return -ENODEV;
                }
                mwdev->tx->chan = txchan;
                mwdev->tx->direction = DMA_MEM_TO_DEV; 
            }
            if (rxchan) {
                mwdev->rx = (struct mwadma_chan*)devm_kzalloc(&IP2DEV(mwdev), 
                        sizeof(struct mwadma_chan),GFP_KERNEL);
                if (mwdev->rx == NULL) {
                    dev_err(&IP2DEV(mwdev), "failed to allocate rx channel\n");
                    return -ENODEV;
                }
                mwdev->rx->chan = rxchan;
                mwdev->rx->direction = DMA_DEV_TO_MEM;
            }
            id++;
        }
    }   
    return 0;
}

static irqreturn_t fifoirq_handler(int irq, void *data)
{
    struct mwadma_chan *mwchan = data;
    iowrite32(0x0, mwchan->registers + wr_ISRE_Data_fifo_irq_ctrl_pcore);
    iowrite32(0x3, mwchan->registers + wr_ISR_Data_fifo_irq_ctrl_pcore); /* disable pending interrupts */
    return IRQ_HANDLED;
}

static int mwadma_fifo_reset_configure(struct mwadma_chan *mwchan, int overflow_en, int underflow_en)
{
    int isre = ((overflow_en << 1) | underflow_en) & 0x3; /* only two bits required here */
    if(!mwchan->registers)
    {
        return -EINVAL;
    }
    pr_info("Resetting fifo core\n");
    /* Reset core */
    iowrite32(0x3, mwchan->registers + wr_CTRL_Data_fifo_irq_ctrl_pcore);
    /* control set up */
    iowrite32(isre, mwchan->registers + wr_ISRE_Data_fifo_irq_ctrl_pcore);

    return 0;
}
static int mwadma_fifo_poll_count(struct mwadma_chan *mwchan)
{
    if(!mwchan->registers)
    {
        return -EINVAL;
    }
    mwchan->of_cnt = ioread32(mwchan->registers + rd_OFCNT_Data_fifo_irq_ctrl_pcore);
    mwchan->uf_cnt = ioread32(mwchan->registers + rd_UFCNT_Data_fifo_irq_ctrl_pcore);
    return 0;
}
static int mwadma_fifo_probe(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan)
{
    int err = 0;
    switch(mwchan->direction)
    {
        case DMA_DEV_TO_MEM: /* Rx */
            mwchan->fifodev = of_find_compatible_node(NULL, NULL, "mathworks,axi-fifoirqctrl-rx");
            dev_info(&IP2DEV(mwdev), "Probing Rx HW FIFO\n");
            break;
        case DMA_MEM_TO_DEV: /* Tx */
            mwchan->fifodev = of_find_compatible_node(NULL, NULL, "mathworks,axi-fifoirqctrl-tx");
            dev_info(&IP2DEV(mwdev), "Probing Tx HW FIFO\n");
            break;
        default:
            return -EINVAL;
            /* Error */
    }

    /* Probe for hw fifo */
    if(mwchan->fifodev) /* Exists */
    {
        /* Set up core */

        if(of_address_to_resource(mwchan->fifodev, 0, &mwchan->res))
        {
            return -EINVAL;
        }

        if(!request_mem_region(mwchan->res.start, resource_size(&mwchan->res),"mathworks-fifo"))
        {
            return -EBUSY;
        }
        mwchan->registers = ioremap(mwchan->res.start, resource_size(&mwchan->res));
        if(!mwchan->registers)
        {
            release_mem_region(mwchan->res.start, resource_size(&mwchan->res));
            return -ENOMEM;
        }
        /* Set core into default state with no interrupts enabled*/
        mwadma_fifo_reset_configure(mwchan, 0, 0);
        /* Probe the interrupt number and register */
        mwchan->fifoirq = irq_of_parse_and_map(mwchan->fifodev, 0);
        err = request_irq(mwchan->fifoirq, fifoirq_handler, IRQF_SHARED,
                    "mathworks-fifo", mwchan);
        if (err) {
            dev_err(&IP2DEV(mwdev), "Unable to request FIFO IRQ\n");
            irq_dispose_mapping(mwchan->fifoirq);
        }
    }
    dev_info(&IP2DEV(mwdev), "HW FIFO Registered\n");
    return 0;
}
/*
 * @brief mwadma_of_probe
 */
static int mwadma_of_probe(struct platform_device *op)
{
    int                         status = 0;
    struct mwadma_dev           *mwdev;
    struct device_node          *np = op->dev.of_node;
    struct of_phandle_args      dma_spec;
    struct device *dev  = &op->dev;

    mwdev = (struct mwadma_dev*)devm_kzalloc(&op->dev, sizeof(struct mwadma_dev),GFP_KERNEL);
    if (!mwdev) {
        status = -ENOMEM;
        goto allocation_error;
    }
    mwdev->mem = platform_get_resource(op, IORESOURCE_MEM,0);
    if(!mwdev->mem) {
        status = -ENOENT;
        dev_err(&op->dev, "Failed to get resource for platform device\n");
        goto invalid_platform_res;
    }
    dev_dbg(&op->dev, "Dev memory resource found at 0x%08X 0x%08X.\n", mwdev->mem->start, resource_size(mwdev->mem));
    mwdev->mem = request_mem_region(mwdev->mem->start, resource_size(mwdev->mem), op->name);
    if(!mwdev->mem) {
        status = -EBUSY;
        dev_err(&op->dev, "Failed to get mem region for our device\n");
        goto invalid_platform_res;
    }
    mwdev->regs = ioremap(mwdev->mem->start, resource_size(mwdev->mem));
    if(!mwdev->regs) {
        status = -ENODEV;
        dev_err(&op->dev, "Failed to do ioremap\n");
        goto ioremap_failed;
    }
    mwdev->pdev = op;
    mwdev->name = np->name;
    if(np->data == NULL) {
        np->data = mwdev;
    }
    if(mwadma_dev_id == 0) {
        status = alloc_chrdev_region(&mwadma_dev_id, 0, 16, DRIVER_NAME);
        if (status)
        {
            dev_err(&op->dev, "Character dev. region not allocated: %d\n", status);
            goto chrdev_alloc_err;
        }
        dev_dbg(&op->dev, "Char dev region registered: major num:%d\n", MAJOR(mwadma_dev_id));
    }
    if(mwadma_class == NULL) {
        mwadma_class = class_create(THIS_MODULE, DRIVER_NAME);
        if(IS_ERR(mwadma_class))
        {
            status = PTR_ERR(mwadma_class);
            goto class_create_err;
        }
        dev_dbg(&op->dev, "mwadma class registration success\n");
    }
    status = mwadma_setup_cdev(mwdev, &(mwdev->dev_id));
    if(status)
    {
        dev_err(&op->dev, "mwadma device addition failed: %d\n", status);
        goto dev_add_err;
    }
    
    dev_info(&op->dev, "pcore phys_addr:0x%08llX mapped to 0x%p\n", (unsigned long long)mwdev->mem->start, mwdev->regs);
    device_num++;
   
    /*
     * ####################################################################
     * Following section does probing & init of DMA channels
     * ####################################################################
     */
    
    /*  find associated AXI DMA  */
    status = of_parse_phandle_with_args(np, "dma-request", "#dma-cells",0, &dma_spec);
    if(status) {
        dev_info(&op->dev, "Device tree binding did not have dma-request. Continue as a MMIO device mapping\n");
	return 0;
    }
    dev_dbg(&IP2DEV(mwdev), "of_parse_handle_with_args successful\n");
    status = mwadma_channel_probe(mwdev);
    if (status){
        dev_err(&IP2DEV(mwdev),"Channel probe failed. Verify device tree and FPGA IP core addresses.\n");
        goto dev_add_err;        
    }
   
    /*
     * ####################################################################
     * Following section creates the sysfs entires for DMA
     * ####################################################################
     */
    status = sysfs_create_group(&dev->kobj, &mwdma_attr_group);
    if (status) {
        dev_err(&op->dev, "Error creating the sysfs devices\n");
        goto dev_add_err;
    }

    mwadma_fifo_probe(mwdev, mwdev->rx);
    mwadma_fifo_probe(mwdev, mwdev->tx);
    return status;
    
dev_add_err:
    if(mwadma_class){
        class_destroy(mwadma_class);
    }
class_create_err:
    unregister_chrdev_region(mwadma_dev_id, 16);
    mwadma_dev_id = 0;
chrdev_alloc_err:
    iounmap(mwdev->regs);
ioremap_failed:
    release_mem_region(mwdev->mem->start, resource_size(mwdev->mem));
invalid_platform_res:

allocation_error:
    return status;
}

/*
 * @brief mwadma_of_remove
 */
static int mwadma_of_remove(struct platform_device *pdev)
{
    struct mwadma_dev *mwdev;
    struct device *dev = &pdev->dev;
    struct device_node *np =  pdev->dev.of_node;
    if(np->data == NULL)
    {
        dev_err(&pdev->dev, "MWADMA device not found.\n");
        return -ENOSYS;
    }
    mwdev = (struct mwadma_dev *) (np->data);
    /* If user did not free up the channel and DMA memory, do it while we
     * unload the driver
     */
    dev_dbg(&IP2DEV(mwdev), "Freeing coherent dma memory\n");
    if(mwdev->virt != NULL)  {
        dma_free_coherent(&IP2DEV(mwdev),mwdev->size, mwdev->virt ,mwdev->phys);
    }
    dev_dbg(&IP2DEV(mwdev), "Freeing rx-scatter channel dma memory\n");
    if ((mwdev->rx !=NULL) && (mwdev->rx->scatter !=NULL) && (&mwdev->rx->scatter->list != NULL)) 
    {
        mwadma_free_channel(mwdev, mwdev->rx);
    }
    dev_dbg(&IP2DEV(mwdev), "Freeing tx-scatter channel dma memory\n");
    if ((mwdev->tx !=NULL) && (mwdev->tx->scatter !=NULL) && (&mwdev->tx->scatter->list != NULL)) 
    {
        mwadma_free_channel(mwdev, mwdev->tx);
    }
    dev_info(&IP2DEV(mwdev), "%s : free and release memory\n", np->name);    
    dev_info(&IP2DEV(mwdev),"Removing sysfs entries");
    sysfs_remove_group(&dev->kobj, &mwdma_attr_group);

    if(mwdev->regs) {
        iounmap(mwdev->regs);
    }
    if(mwdev->mem->start) {
        release_mem_region(mwdev->mem->start, resource_size(mwdev->mem));
    } else {
        dev_err(&pdev->dev, "Invalid address\n");
    }
    np->data = NULL;
    device_num--;
    if(&mwdev->cdev) {
        dev_dbg(&IP2DEV(mwdev),  "Destroy character dev\n");
        device_destroy(mwadma_class, mwdev->dev_id);
        cdev_del(&mwdev->cdev);
    }
    cur_minor--;
    if(device_num == 0)  {
        dev_dbg(&IP2DEV(mwdev),  "destroy mwadma class\n");
        if (mwadma_class) {
            class_destroy(mwadma_class);
        }
        dev_dbg(&IP2DEV(mwdev),  "release device region\n");
        unregister_chrdev_region(mwadma_dev_id, 16);
        mwadma_dev_id = 0;
    }
    return 0;
}

/*
 * @brief mwadma_of_match
 */
static const struct of_device_id mwadma_of_match[]  = {
    { .compatible = "mathworks,mwipcore-v2.00",},
    { .compatible = "mathworks,mwipcore-axi4lite-v1.00",},
    {},
};

static struct platform_driver mwadma_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = mwadma_of_match,
    },
    .probe = mwadma_of_probe,
    .remove = mwadma_of_remove,
};

module_platform_driver(mwadma_driver);

MODULE_DEVICE_TABLE(of, mwadma_of_match);
MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks AXI4-Lite/AXI4-Stream DMA driver");

/*DMA PARAMS */
