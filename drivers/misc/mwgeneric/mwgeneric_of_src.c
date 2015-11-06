/*
 * Copyright 2013 MathWorks, Inc.
 * 
 *
 */
#include "mwgeneric_of.h"
#include "mwgeneric_ioctl.h"
#include "mwgeneric_common.h"

static irqreturn_t mwgeneric_intr_handler(int irq, void * theIpcore)
{
    struct ipcore_info *thisIpcore = (struct ipcore_info*) theIpcore;
    
    dev_dbg(thisIpcore->dev, "IRQ Handled:in %s at %d\n", __func__, __LINE__);
    return IRQ_HANDLED;

}


int	mwgeneric_get_param(struct ipcore_info *thisIpcore, void *arg)
{
	struct mwgeneric_param_info pinfo;
	const void *paramData;
	
	if( copy_from_user(&pinfo, (struct mwgeneric_param_info *)arg, sizeof(struct mwgeneric_param_info)) ) {
		return -EACCES;
	}
	
	paramData = of_get_property(thisIpcore->dev->of_node,pinfo.name, &pinfo.size);
	
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

int mwgeneric_get_info(struct ipcore_info *thisIpcore, struct mwgeneric_info **thisDev)
{
	int i, devname_len, status;
	const char *of_devname = of_get_property(thisIpcore->dev->of_node,"mwgen,devname", &devname_len);
	char *devname;

	devname_len = min(devname_len,MWGENERIC_DEVNAME_LEN);
	for (i = 0; i < MWGENERIC_MAX_DEVTYPE; i++)
	{
		/* Search for the device in the table */
		*thisDev = &dev_table[i];
		devname=(*thisDev)->devname;
		if(*devname == 0){
			dev_info(thisIpcore->dev, "'%s' device not found, creating\n", of_devname);
			break;
		}
		if(strncasecmp(devname,of_devname,devname_len) == 0)
		{
			dev_info(thisIpcore->dev, "'%s' device found as '%s'\n", of_devname, devname);
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
			dev_err(thisIpcore->dev, "Character dev. region not allocated: %d\n", status);
			return status;
		}
		printk(KERN_INFO DRIVER_NAME ": Char dev region registered: major num:%d\n", MAJOR((*thisDev)->devid));
		dev_info(thisIpcore->dev, "'%s' device created as '%s'\n", of_devname, devname);
		return 0;
	}
	
	/* Not found and table full */
	return -ENOMEM;
	
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
    thisIpcore->dev = &pdev->dev;
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
		dev_info(thisIpcore->dev, "%s : creating i2c link\n", nodePointer->name);
		thisIpcore->i2c = of_find_i2c_device_by_node(slave_node);
		if(thisIpcore->i2c == NULL){
			dev_info(thisIpcore->dev, "%s : could not find i2c device\n", nodePointer->name);
		} else {		
			dev_info(thisIpcore->dev, "%s : Adding link to %s[%s]\n", nodePointer->name, thisIpcore->i2c->adapter->name, thisIpcore->i2c->name);
			
			/* add a link to the i2c device */
			status = sysfs_create_link(&thisIpcore->char_device->kobj, &thisIpcore->i2c->dev.kobj, "i2c_device");
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


    dev_info(thisIpcore->dev, "%s : free and release memory\n", nodePointer->name);
    
	if (thisIpcore->i2c != NULL) {
		/* Remove the i2c adapter link */
		sysfs_remove_link(&thisIpcore->i2c->dev.kobj, "i2c_adapter");
		/* Remove the i2c device link */
		sysfs_remove_link(&thisIpcore->char_device->kobj, "i2c_device");
	}
	
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
