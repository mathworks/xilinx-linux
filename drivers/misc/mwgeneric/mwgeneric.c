/*
 * Copyright 2013 MathWorks, Inc.
 * 
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/of_irq.h>
#include <linux/debugfs.h>
#include <linux/init.h>

#include <linux/mm.h>

#include "mwgeneric.h"
#include "mwgeneric_ioctl.h"

#define DRIVER_NAME "mwgeneric"
#define MWGENERIC_INTERRUPT_OFFSET 0x108

/*Device structure for IPCore information*/
dev_t mwgeneric_dev_id = 0;
static unsigned int device_num = 0;
static struct class *mwgeneric_class = NULL;
static unsigned int g_mw_intr_offset = 0x108;

static struct mwgeneric_info dev_table[MWGENERIC_MAX_DEVTYPE] = {{{0}}};

static int mwgeneric_fasync_impl(int fd, struct file* fp, int mode)
{
    struct ipcore_info *thisIpcore = fp->private_data;
    
    return fasync_helper(fd, fp, mode, &thisIpcore->asyncq);
  
}

static int mwgeneric_open(struct inode *inode, struct file *fp)
{
    struct ipcore_info *thisIpcore;
    thisIpcore = container_of(inode->i_cdev, struct ipcore_info, cdev);
    fp->private_data = thisIpcore;
    
    return 0;
}

static int mwgeneric_close(struct inode *inode, struct file *fp)
{
    mwgeneric_fasync_impl(-1, fp, 0);
    return 0;
}

static irqreturn_t mwgeneric_intr_handler(int irq, void * theIpcore)
{
    struct ipcore_info *thisIpcore = (struct ipcore_info*) theIpcore;
    /*  */ 
    writel(1,thisIpcore->regs + g_mw_intr_offset);
    writel(0,thisIpcore->regs + g_mw_intr_offset);
    if (thisIpcore->asyncq)
    {
        kill_fasync(&thisIpcore->asyncq, SIGIO, POLL_IN);
    }
    /*dev_dbg(&IP2DEV(thisIpcore), "IRQ Handled:in %s at %d\n", __func__, __LINE__);*/
    return IRQ_HANDLED;

}

int	mwgeneric_get_param(struct ipcore_info *thisIpcore, void *arg)
{
	struct mwgeneric_param_info pinfo;
	const void *paramData;
	
	if( copy_from_user(&pinfo, (struct mwgeneric_param_info *)arg, sizeof(struct mwgeneric_param_info)) ) {
		return -EACCES;
	}
	
	paramData = of_get_property(thisIpcore->pdev->dev.of_node,pinfo.name, &pinfo.size);
	
	/* Copy the struct back to user space */
	if( copy_to_user((struct mwgeneric_param_info*)arg, &pinfo, sizeof(struct mwgeneric_param_info)) ) {
		return -EACCES;
	}
	
	/* Copy any data to the user buf */
	if (paramData) {
		if( copy_to_user((void *)pinfo.buf, paramData, pinfo.size) ){
			return -EACCES;
		}	
	} else {
		return -ENODEV;
	}
	
	return 0;
}

int mwgeneric_dma_alloc(struct ipcore_info *thisIpcore, size_t size) {

	struct mw_dma_info *dinfo = &thisIpcore->dma_info;
	
	if (dinfo->size != 0) {
		dev_err(&IP2DEV(thisIpcore), "DMA memory already allocated\n");		
		return -EEXIST;
	}
	
	dinfo->virt = dma_alloc_coherent(&IP2DEV(thisIpcore), size,
						&dinfo->phys, GFP_KERNEL);
	if(!dinfo->virt){
		dev_err(&IP2DEV(thisIpcore), "failed to allocate DMA memory\n");		
		return -ENOMEM;
	}
	dinfo->size = size;
	
	return 0;

}

int	mwgeneric_dma_info(struct ipcore_info *thisIpcore, void *arg)
{
	
	struct mwgeneric_dma_info dinfo;
	
	/* Copy the struct from user space */
	if( copy_from_user(&dinfo, (struct mwgeneric_dma_info *)arg, sizeof(struct mwgeneric_dma_info)) ) {
		return -EACCES;
	}
	
	/* Populate the struct with information */
	dinfo.size = thisIpcore->dma_info.size;
	dinfo.phys = (void *)thisIpcore->dma_info.phys;
	
	/* Copy the struct back to user space */
	if( copy_to_user((struct mwgeneric_dma_info*)arg, &dinfo, sizeof(struct mwgeneric_dma_info)) ) {
		return -EACCES;
	}
	
	return 0;

}

int	mwgeneric_reg_info(struct ipcore_info *thisIpcore, void *arg)
{
	struct mwgeneric_reg_info rinfo;

	/* Copy the struct from user space */
	if( copy_from_user(&rinfo, (struct mwgeneric_reg_info *)arg, sizeof(struct mwgeneric_reg_info)) ) {
		return -EACCES;
	}

	/* Populate the struct with information */
	rinfo.size = resource_size(thisIpcore->mem);
	rinfo.phys = (void *)thisIpcore->mem->start;

	/* Copy the struct back to user space */
	if( copy_to_user((struct mwgeneric_reg_info*)arg, &rinfo, sizeof(struct mwgeneric_reg_info)) ) {
		return -EACCES;
	}

	return 0;
}

static void mwgeneric_mmap_dma_open(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(&IP2DEV(thisIpcore), "DMA VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mwgeneric_mmap_dma_close(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(&IP2DEV(thisIpcore), "DMA VMA close.\n");
	
	/* Free the memory DMA */
	dma_free_coherent(&IP2DEV(thisIpcore),thisIpcore->dma_info.size,
				thisIpcore->dma_info.virt, thisIpcore->dma_info.phys);
	
	/* Set the size to zero to indicate no memory is allocated */
	thisIpcore->dma_info.size = 0;
}

static void mwgeneric_mmap_open(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(&IP2DEV(thisIpcore), "Simple VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mwgeneric_mmap_close(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(&IP2DEV(thisIpcore), "Simple VMA close.\n");
}


static int mwgeneric_mmap_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
    struct page *thisPage;
    unsigned long offset;
    offset = (vmf->pgoff - vma->vm_pgoff) << PAGE_SHIFT;
    thisPage = virt_to_page(thisIpcore->mem->start + offset);
    get_page(thisPage);
    vmf->page = thisPage;
    return 0;
}

struct vm_operations_struct mwgeneric_mmap_ops = {
    .open   = mwgeneric_mmap_open,
    .close  = mwgeneric_mmap_close,
    .fault = mwgeneric_mmap_fault,
}; 

struct vm_operations_struct mwgeneric_mmap_dma_ops = {
    .open   = mwgeneric_mmap_dma_open,
    .close  = mwgeneric_mmap_dma_close,
}; 

int mwgeneric_mmap(struct file *fp, struct vm_area_struct *vma)
{
    struct ipcore_info *thisIpcore = fp->private_data;
    size_t	size = vma->vm_end - vma->vm_start;
	int status = 0;
	vma->vm_private_data = thisIpcore;
	
	if (thisIpcore->memtype == MWGENERIC_MEMTYPE_NOMEM) {
		return -ENOMEM;
	}
	dev_info(&IP2DEV(thisIpcore), "[MMAP] size:%X pgoff: %lx\n", size, vma->vm_pgoff);
 
	switch(vma->vm_pgoff) {
		case 0: 
			/* mmap the MMIO base address */
			vma->vm_flags |= VM_IO | VM_DONTDUMP | VM_DONTDUMP; // may be redundant with call to remap_pfn_range below
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			if (remap_pfn_range(vma, vma->vm_start,
					thisIpcore->mem->start >> PAGE_SHIFT,
					size,
					vma->vm_page_prot))
			{
				return -EAGAIN;
			}
			vma->vm_ops = &mwgeneric_mmap_ops;
			break;
		default:
			/* mmap the DMA region */
			
			status = mwgeneric_dma_alloc(thisIpcore, size);
			if (status != 0)
				return status;
						
			if (thisIpcore->dma_info.size == 0 || size != thisIpcore->dma_info.size)
				return -EINVAL;
			/* We want to mmap the whole buffer */
			vma->vm_pgoff = 0; 
			status =  dma_mmap_coherent(&IP2DEV(thisIpcore),vma,
						thisIpcore->dma_info.virt, thisIpcore->dma_info.phys, size);
			vma->vm_ops = &mwgeneric_mmap_dma_ops;
			break;
	} 
	//vma->vm_ops->open(vma);
	
	return status;
}

static long mwgeneric_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    /* struct ipcore_info *thisIpcore = fp->private_data; */
    int status;
	struct ipcore_info *thisIpcore = fp->private_data;
    
    if (NULL==thisIpcore) {
        return -ENODEV;
    }

    switch(cmd) {
	case MWGENERIC_GET_PARAM:
  
		status = mwgeneric_get_param(thisIpcore, (void *)arg);
		break;
		
	case MWGENERIC_DMA_INFO:
		
		status = mwgeneric_dma_info(thisIpcore, (void *)arg);
		break;
	
	case MWGENERIC_REG_INFO:

		status = mwgeneric_reg_info(thisIpcore, (void *)arg);
		break;

	default:
		status = -EINVAL;
    }
    return status;
}




struct file_operations mwgeneric_cdev_fops = {
    .owner 		= THIS_MODULE,
    .open 		= mwgeneric_open,
    .fasync 		= mwgeneric_fasync_impl,
    .release 		= mwgeneric_close,
    .mmap		= mwgeneric_mmap,
    .unlocked_ioctl	= mwgeneric_ioctl,
};

int mwgeneric_get_info(struct ipcore_info *thisIpcore, struct mwgeneric_info **thisDev)
{
	int i, devname_len, status;
	const char *of_devname = of_get_property(thisIpcore->pdev->dev.of_node,"mwgen,devname", &devname_len);
	char *devname;

	devname_len = min(devname_len,MWGENERIC_DEVNAME_LEN);
	for (i = 0; i < MWGENERIC_MAX_DEVTYPE; i++)
	{
		/* Search for the device in the table */
		*thisDev = &dev_table[i];
		devname=(*thisDev)->devname;
		if(*devname == 0){
			dev_info(&IP2DEV(thisIpcore), "'%s' device not found, creating\n", of_devname);
			break;
		}
		if(strncasecmp(devname,of_devname,devname_len) == 0)
		{
			dev_info(&IP2DEV(thisIpcore), "'%s' device found as '%s'\n", of_devname, devname);
			return 0;
		}
	}
	if ((*devname == 0) && i < MWGENERIC_MAX_DEVTYPE)
	{
		/* Add in a new device to the table */
		strncpy((*thisDev)->devname,of_devname,devname_len);

		status = alloc_chrdev_region(&(*thisDev)->devid, 0, 16, of_devname);
		if (status)
		{
			dev_err(&IP2DEV(thisIpcore), "Character dev. region not allocated: %d\n", status);
			return status;
		}
		printk(KERN_INFO DRIVER_NAME ": Char dev region registered: major num:%d\n", MAJOR((*thisDev)->devid));
		dev_info(&IP2DEV(thisIpcore), "'%s' device created as '%s'\n", of_devname, devname);
		return 0;
	}
	
	/* Not found and table full */
	return -ENOMEM;
	
}

static int mwgeneric_setup_cdev(struct ipcore_info *thisIpcore, dev_t *dev_id)
{
    int status = 0;
    struct device * thisDevice;
	struct mwgeneric_info *dev_entry;
   
   cdev_init(&thisIpcore->cdev, &mwgeneric_cdev_fops);
   thisIpcore->cdev.owner = THIS_MODULE;
   thisIpcore->cdev.ops = &mwgeneric_cdev_fops;
   
   /* Find the device name */
   status = mwgeneric_get_info(thisIpcore, &dev_entry); 
   if (status)
   {
		return status;
   }

   *dev_id = MKDEV(MAJOR(dev_entry->devid), dev_entry->devcnt);
   status = cdev_add(&thisIpcore->cdev, *dev_id, 1);

   if (status < 0) {
       unregister_chrdev_region(dev_entry->devid, 16);
	   return status;
   } 
   
   thisDevice = device_create(mwgeneric_class, NULL, *dev_id, NULL, "%s%d", dev_entry->devname, dev_entry->devcnt++);
   
   if(IS_ERR(thisDevice)) 
   {
       status = PTR_ERR(thisDevice);
       dev_err(&thisIpcore->pdev->dev, "Error: failed to create device node %s, err %d\n", thisIpcore->name, status);
       cdev_del(&thisIpcore->cdev);
   }
   thisIpcore->class_device = thisDevice;
   return status;
}

static void mwgeneric_init(struct ipcore_info *thisIpcore)
{
    printk(KERN_INFO DRIVER_NAME ": Initialization done.\n");
}


static const struct of_device_id mwgeneric_of_match[] = {
    { .compatible = "mathworks,mwgeneric-v1.00",},
    {},

};

MODULE_DEVICE_TABLE(of, mwgeneric_of_match);



static int mwgeneric_of_probe(struct platform_device *pdev)
{
    int status = 0;
    const char *pm;
	struct ipcore_info *thisIpcore;
    struct device_node *nodePointer = pdev->dev.of_node;
    struct device_node *slave_node;

    thisIpcore = (struct ipcore_info*) kzalloc(sizeof(*thisIpcore), GFP_KERNEL);
    
    if (!thisIpcore)
    {
        status = -ENOMEM;
        goto allocation_error;
    }
    
	thisIpcore->memtype = MWGENERIC_MEMTYPE_NORMAL; /* default device type */
	status = of_property_read_string(nodePointer, "mwgen,type", &pm);
	if (status >= 0) {
		if(!strcasecmp(pm,MWGENERIC_MEMTYPE_NOMEM_STR)){
			thisIpcore->memtype = MWGENERIC_MEMTYPE_NOMEM; /*no memory */
		}
	}
	
	if (thisIpcore->memtype == MWGENERIC_MEMTYPE_NORMAL) {
		thisIpcore->mem = platform_get_resource(pdev, IORESOURCE_MEM,0);
		if(!thisIpcore->mem) 
		{
			status = -ENOENT;
			dev_err(&pdev->dev, "Failed to obtain the resource for platform device\n");
			goto invalid_platform_res;
		}

		printk(KERN_INFO DRIVER_NAME " : Dev memory resource found at %08X %08X. \n", thisIpcore->mem->start, resource_size(thisIpcore->mem));


		thisIpcore->mem = request_mem_region(thisIpcore->mem->start, resource_size(thisIpcore->mem), pdev->name);

		if (!thisIpcore->mem)
		{
			status = -ENODEV;
			dev_err(&pdev->dev, "Error while request_mem_region call\n");
			goto mem_request_err;
		}


		thisIpcore->regs = ioremap(thisIpcore->mem->start, resource_size(thisIpcore->mem));
		if(!thisIpcore->regs)
		{
			status = -ENODEV;
			dev_err(&pdev->dev, "Failed while ioremap\n"); 
			goto ioremap_failure;
		}
	}
    thisIpcore->pdev = pdev;
    thisIpcore->name = nodePointer->name;
    dev_dbg(&pdev->dev,"IPCore name :%s\n", thisIpcore->name);

    if (nodePointer->data == NULL)
         nodePointer->data = thisIpcore; 

    if(mwgeneric_class == NULL)
    {
        mwgeneric_class = class_create(THIS_MODULE, DRIVER_NAME);
        if(IS_ERR(mwgeneric_class))
        {
            status = PTR_ERR(mwgeneric_class);
            goto class_create_err;
        }
        printk(KERN_INFO DRIVER_NAME ": mwipcore class registration success\n");
    }

    status = mwgeneric_setup_cdev(thisIpcore, &(thisIpcore->dev_id));
    if(status)
    {
        dev_err(&pdev->dev, "mwipcore device addition failed: %d\n", status);
        goto dev_add_err;
    }

    thisIpcore->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
    /* It is possible that we have not required any interrupt */
    if (thisIpcore->irq)
    {
        status = request_irq(thisIpcore->irq, mwgeneric_intr_handler, IRQF_SHARED, DRIVER_NAME, thisIpcore);
        if(status)
        {
            dev_err(&pdev->dev, "mwipcore interrupt request addition failed.\n");
            goto dev_add_err;
        }
    }

    slave_node = of_parse_phandle(nodePointer, "i2c-controller", 0);
    if (slave_node) {
		dev_info(&IP2DEV(thisIpcore), "%s : creating i2c link\n", nodePointer->name);
		thisIpcore->i2c = of_find_i2c_device_by_node(slave_node);
		if(thisIpcore->i2c == NULL){
			dev_info(&IP2DEV(thisIpcore), "%s : could not find i2c device\n", nodePointer->name);
		} else {		
			dev_info(&IP2DEV(thisIpcore), "%s : Adding link to %s[%s]\n", nodePointer->name, thisIpcore->i2c->adapter->name, thisIpcore->i2c->name);
			
			/* add a link to the i2c device */
			status = sysfs_create_link(&thisIpcore->class_device->kobj, &thisIpcore->i2c->dev.kobj, "i2c_device");
			if (status < 0)
				goto dev_add_err;
			
			/* add a link to the i2c bus */			
			status = sysfs_create_link(&thisIpcore->i2c->dev.kobj, &thisIpcore->i2c->adapter->dev.kobj, "i2c_adapter");
			if (status < 0)
				goto dev_add_err;
			
		}
		of_node_put(slave_node);
	} else {
		thisIpcore->i2c = NULL;
	}
	
    mwgeneric_init(thisIpcore);


    device_num++;
    return status;
	
dev_add_err:
    if(mwgeneric_class){
         class_destroy(mwgeneric_class);    
    }
class_create_err:
	if (thisIpcore->memtype == MWGENERIC_MEMTYPE_NORMAL){
		iounmap(thisIpcore->regs);
	}
ioremap_failure:
	if (thisIpcore->memtype == MWGENERIC_MEMTYPE_NORMAL){
		release_mem_region(thisIpcore->mem->start, resource_size(thisIpcore->mem));
	}
mem_request_err:
invalid_platform_res:
    kfree(thisIpcore);
allocation_error:
    return status;
}


static int mwgeneric_of_remove(struct platform_device *pdev)
{
    struct ipcore_info *thisIpcore;
    struct device_node *nodePointer =  pdev->dev.of_node;
	struct mwgeneric_info *dev_entry;

    if(nodePointer->data == NULL)
    {
        dev_err(&pdev->dev, "MWGENERIC device not found.\n");
        return -ENOSYS;
    }

    thisIpcore = (struct ipcore_info *) (nodePointer->data);


    dev_info(&IP2DEV(thisIpcore), "%s : free and release memory\n", nodePointer->name);
    
	if (thisIpcore->i2c != NULL) {
		/* Remove the i2c adapter link */
		sysfs_remove_link(&thisIpcore->i2c->dev.kobj, "i2c_adapter");
		/* Remove the i2c device link */
		sysfs_remove_link(&thisIpcore->class_device->kobj, "i2c_device");
	}
	
    if(thisIpcore->regs)
    {
        iounmap(thisIpcore->regs);
    } 

	if (thisIpcore->dma_info.size != 0) {
		/* Free the memory DMA */
		dev_info(&IP2DEV(thisIpcore), "Free the DMA buffer\n");
		dma_free_coherent(&IP2DEV(thisIpcore),thisIpcore->dma_info.size,
					thisIpcore->dma_info.virt, thisIpcore->dma_info.phys);
	}
	
	if(thisIpcore->mem){
		if(thisIpcore->mem->start)
		{
			release_mem_region(thisIpcore->mem->start, resource_size(thisIpcore->mem));

		}
	}
    nodePointer->data = NULL;
    device_num--;

    
    if(&thisIpcore->cdev)
    {
        dev_info(&IP2DEV(thisIpcore), "Destroy character dev\n");
        device_destroy(mwgeneric_class, thisIpcore->dev_id);
        cdev_del(&thisIpcore->cdev);
    }
	mwgeneric_get_info(thisIpcore, &dev_entry);
	dev_entry->devcnt--;	
	
    if(device_num == 0)
    {
        dev_info(&IP2DEV(thisIpcore), "destroy mwipcore class\n");
        if (mwgeneric_class)
        {
             class_destroy(mwgeneric_class);
        }

        dev_info(&IP2DEV(thisIpcore), "release device region\n");
        unregister_chrdev_region(mwgeneric_dev_id, 16);
        mwgeneric_dev_id  = 0;
    }
	
	kfree(thisIpcore);
	
    return 0;
}



static struct platform_driver mwgeneric_driver = {
    .driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE, 
		.of_match_table = mwgeneric_of_match,
		},
    .probe = mwgeneric_of_probe,
    .remove = mwgeneric_of_remove,
};

module_platform_driver(mwgeneric_driver);


MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks Generic driver");
MODULE_ALIAS(DRIVER_NAME);
