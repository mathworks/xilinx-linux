#include "mwgeneric_pci.h"

/****************************************
* Unsupported functions
****************************************/

int	mwgeneric_get_param(struct ipcore_info *thisIpcore, void *arg)
{
	return -ENODEV;
}

/****************************************
* Bus Specific Functions
****************************************/

int mwgeneric_get_info(struct ipcore_info *thisIpcore, struct mwgeneric_info **thisDev)
{
	int i, status;
	u16 ss_vid, ss_did;		
	char subdev[MWGENERIC_DEVNAME_LEN];
	char *devname;
	struct pci_dev *pdev = ip_to_pdev(thisIpcore);

	pci_get_ss_vid(pdev, &ss_vid);
	pci_get_ss_did(pdev, &ss_did);
	snprintf(subdev,MWGENERIC_DEVNAME_LEN, "mwgeneric_%04X_%04X_dev", ss_vid, ss_did); 

	for (i = 0; i < MWGENERIC_MAX_DEVTYPE; i++)
	{
		/* Search for the device in the table */
		*thisDev = &dev_table[i];
		devname=(*thisDev)->devname;
		
		if((*thisDev)->ss_vid == 0 && (*thisDev)->ss_did == 0){
			dev_info(thisIpcore->dev, "'%s' device not found, creating\n", subdev);
			break;
		}
		if (ss_vid == (*thisDev)->ss_vid && ss_did == (*thisDev)->ss_did)
		{
			dev_info(thisIpcore->dev, "'%s' device found as '%s'\n", subdev, devname);
			return 0;
		}
	}
	if ((*thisDev)->ss_vid == 0 && (*thisDev)->ss_did == 0 && i < MWGENERIC_MAX_DEVTYPE)
	{
		/* Add in a new device to the table */
		strncpy((*thisDev)->devname,subdev,MWGENERIC_DEVNAME_LEN);
		(*thisDev)->ss_vid = ss_vid;
		(*thisDev)->ss_did = ss_did;
		status = alloc_chrdev_region(&(*thisDev)->devid, 0, 16, subdev);
		if (status)
		{
			dev_err(thisIpcore->dev, "Character dev. region not allocated: %d\n", status);
			return status;
		}
		printk(KERN_INFO DRIVER_NAME ": Char dev region registered: major num:%d\n", MAJOR((*thisDev)->devid));
		dev_info(thisIpcore->dev, "'%s' device created as '%s'\n", subdev, devname);
		return 0;
	}
	
	/* Not found and table full */
	return -ENOMEM;
	
}


static int mw_generic_pci_probe(struct pci_dev *pdev, 
				const struct pci_device_id *ident)
{
    int status = 0;
	struct ipcore_info *thisIpcore;

    thisIpcore = (struct ipcore_info*) kzalloc(sizeof(*thisIpcore), GFP_KERNEL);

    if (!thisIpcore)
    {
        status = -ENOMEM;
        goto allocation_error;
    }

	if (dev_get_drvdata(&pdev->dev) != NULL){
		status  = -EALREADY;
		goto invalid_platform_res;
	}
	dev_set_drvdata(&pdev->dev, thisIpcore);

    status = pci_enable_device(pdev);
    if (status)
		goto invalid_platform_res;
	pci_set_master(pdev); /* enable DMA */
	thisIpcore->memtype = MWGENERIC_MEMTYPE_NORMAL; /* default device type */	

	/* reserve resources */
	status = pci_request_regions(pdev, DRIVER_NAME);
	if (status) {
		dev_err(&pdev->dev, "Unable to request regions\n");
		goto invalid_platform_res;
	}

	thisIpcore->mem = &pdev->resource[0];
	printk(KERN_INFO DRIVER_NAME " : Dev memory resource found at %08X %08X. \n", (unsigned int)thisIpcore->mem->start, (unsigned int)resource_size(thisIpcore->mem));

	/*
	thisIpcore->regs = ioremap(thisIpcore->mem->start, resource_size(thisIpcore->mem));
	if(!thisIpcore->regs)
	{
		status = -ENODEV;
		dev_err(&pdev->dev, "Failed while ioremap\n"); 
		goto ioremap_failure;
	}
	*/
	thisIpcore-> regs = NULL;

    thisIpcore->dev = &pdev->dev;
    thisIpcore->name = DRIVER_NAME;
    dev_dbg(&pdev->dev,"IPCore name :%s\n", thisIpcore->name);

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

	// Setup IRQs
	
    device_num++;
    return status;
	
dev_add_err:
    if(mwgeneric_class){
         class_destroy(mwgeneric_class);    
    }
class_create_err:
	if (thisIpcore->memtype == MWGENERIC_MEMTYPE_NORMAL){
		if (thisIpcore->regs) {
			iounmap(thisIpcore->regs);
		}
	}
/*
ioremap_failure:
	if (thisIpcore->memtype == MWGENERIC_MEMTYPE_NORMAL){
		release_mem_region(thisIpcore->mem->start, resource_size(thisIpcore->mem));
	}
mem_request_err:
*/
invalid_platform_res:
    kfree(thisIpcore);
allocation_error:
    return status;
}

static void mw_generic_pci_remove(struct pci_dev *pdev)
{
    struct ipcore_info *thisIpcore = dev_get_drvdata(&pdev->dev);
	struct mwgeneric_info *dev_entry;

    if(thisIpcore == NULL)
    {
        dev_err(&pdev->dev, "MWGENERIC device not found.\n");
        return;
    }

    dev_info(thisIpcore->dev, "%s : free and release memory\n", thisIpcore->name);
    	
    if(thisIpcore->regs)
    {
        iounmap(thisIpcore->regs);
    } 

	if (thisIpcore->dma_info.size != 0) {
		/* Free the memory DMA */
		dev_info(thisIpcore->dev, "Free the DMA buffer\n");
		dma_free_coherent(thisIpcore->dev,thisIpcore->dma_info.size,
					thisIpcore->dma_info.virt, thisIpcore->dma_info.phys);
	}
	
	pci_release_regions(pdev);
	dev_set_drvdata(&pdev->dev, NULL);	
    device_num--;

    
    if(&thisIpcore->cdev)
    {
        dev_info(thisIpcore->dev, "Destroy character dev\n");
		sysfs_remove_link(&thisIpcore->char_device->kobj, "driver");     
        device_destroy(mwgeneric_class, thisIpcore->dev_id);
        cdev_del(&thisIpcore->cdev);
    }
	mwgeneric_get_info(thisIpcore, &dev_entry);
	dev_entry->devcnt--;	
	
    if(device_num == 0)
    {
        dev_info(thisIpcore->dev, "destroy mwipcore class\n");
        if (mwgeneric_class)
        {
             class_destroy(mwgeneric_class);
        }

        dev_info(thisIpcore->dev, "release device region\n");
        unregister_chrdev_region(mwgeneric_dev_id, 16);
        mwgeneric_dev_id  = 0;
    }
	
	kfree(thisIpcore);
}

static struct pci_device_id mw_generic_ids[ ] = {
 { PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022) },
 { 0, },
};

MODULE_DEVICE_TABLE(pci, mw_generic_ids);

static struct pci_driver mw_generic_pci_driver = {
 .name = DRIVER_NAME,
 .id_table = mw_generic_ids,
 .probe = mw_generic_pci_probe,
 .remove = mw_generic_pci_remove,
};

module_pci_driver(mw_generic_pci_driver);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks Generic driver");
MODULE_ALIAS(DRIVER_NAME);
