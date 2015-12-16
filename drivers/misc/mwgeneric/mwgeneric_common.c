#include "mwgeneric_common.h"

/*Device structure for IPCore information*/
dev_t mwgeneric_dev_id = 0;
unsigned int device_num = 0;
struct class *mwgeneric_class = NULL;
struct mwgeneric_info dev_table[MWGENERIC_MAX_DEVTYPE] = {{{0}}};

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

int mwgeneric_dma_alloc(struct ipcore_info *thisIpcore, size_t size) {

	struct mw_dma_info *dinfo = &thisIpcore->dma_info;
	
	if (dinfo->size != 0) {
		dev_err(thisIpcore->dev, "DMA memory already allocated\n");		
		return -EEXIST;
	}
	
	dinfo->virt = dma_alloc_coherent(thisIpcore->dev, size,
						&dinfo->phys, GFP_KERNEL);
	if(!dinfo->virt){
		dev_err(thisIpcore->dev, "failed to allocate DMA memory\n");		
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
	dev_info(thisIpcore->dev, "DMA VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mwgeneric_mmap_dma_close(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "DMA VMA close.\n");
	
	/* Free the memory DMA */
	dma_free_coherent(thisIpcore->dev,thisIpcore->dma_info.size,
				thisIpcore->dma_info.virt, thisIpcore->dma_info.phys);
	
	/* Set the size to zero to indicate no memory is allocated */
	thisIpcore->dma_info.size = 0;
}

static void mwgeneric_mmap_open(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "Simple VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mwgeneric_mmap_close(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "Simple VMA close.\n");
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
	
	dev_info(thisIpcore->dev, "[MMAP] size:%X pgoff: %lx\n", (unsigned int)size, vma->vm_pgoff);
 
	switch(vma->vm_pgoff) {
		case 0: 
            if (thisIpcore->memtype == MWGENERIC_MEMTYPE_NOMEM) {
        		return -ENOMEM;
        	}
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
			status =  dma_mmap_coherent(thisIpcore->dev,vma,
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

int mwgeneric_setup_cdev(struct ipcore_info *thisIpcore, dev_t *dev_id)
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
       dev_err(thisIpcore->dev, "Error: failed to create device node %s, err %d\n", thisIpcore->name, status);
       cdev_del(&thisIpcore->cdev);
   }
   return status;
}
