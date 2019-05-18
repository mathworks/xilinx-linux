/*
 * MathWorks Shared Memory Channel
 *
 * Copyright 2019 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/idr.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/mutex.h>

#include <linux/string.h>
#include <linux/mathworks/mathworks_ip.h>
#include "mw_sharedmem_iio_channel.h"
#include "mathworks_ipcore.h"

static DEFINE_IDA(mw_sharedmem_iio_channel_ida);

#define MWDEV_TO_MWIP(mwdev)			(mwdev->mw_ip_info)
#define IP2DEVP(mwdev)  (MWDEV_TO_MWIP(mwdev)->dev)

#define MW_SHAREDMEM_IIO_ENUM IIO_ENUM
#define MW_SHAREDMEM_IIO_ENUM_AVAILABLE(_name, _shared_by, _e) \
{ \
	.name = (_name "_available"), \
	.shared = (_shared_by), \
	.read = iio_enum_available_read, \
	.private = (uintptr_t)(_e), \
}

struct mw_sharedmem_iio_channel_info {
	enum iio_device_direction 		iio_direction;
};

struct mw_sharedmem_region {
	void 			*virt;
	phys_addr_t  	phys;
	size_t 			size;
};

struct mw_sharedmem_iio_chandev {
	struct mathworks_ipcore_dev 			*mwdev;
	struct device							dev;
	enum iio_device_direction 				iio_direction;
	int										num_data_chan;
	size_t 									offset;
	struct mw_sharedmem_region				region;
};

struct mw_sharedmem_buffer {
	struct iio_buffer 					buffer;
	struct mw_sharedmem_iio_chandev		*mwchan;
	bool 								enabled;
	struct mutex 						lock;
};

static struct mw_sharedmem_buffer *buffer_to_mw_sharedmem_buffer(struct iio_buffer *buffer)
{
	return container_of(buffer, struct mw_sharedmem_buffer, buffer);
}

static void mw_sharedmem_iio_chan_ida_remove(void *opaque){
	struct mw_sharedmem_iio_chandev* mwchan = opaque;
	ida_simple_remove(&mw_sharedmem_iio_channel_ida, mwchan->dev.id);
}

static int mw_sharedmem_iio_buffer_preenable(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer preenable\n");

	return 0;
}
static int mw_sharedmem_iio_buffer_postenable(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer postenable\n");
	
	return 0;
}

static int mw_sharedmem_iio_buffer_predisable(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer predisable\n");

	return 0;
}

static int mw_sharedmem_iio_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer postdisable\n");
	
	return 0;
}


static const struct iio_buffer_setup_ops mw_sharedmem_iio_buffer_setup_ops = {

	.preenable = &mw_sharedmem_iio_buffer_preenable,
	.postenable = &mw_sharedmem_iio_buffer_postenable,
	.predisable = &mw_sharedmem_iio_buffer_predisable,
	.postdisable = &mw_sharedmem_iio_buffer_postdisable,
};

static int mw_sharedmem_iio_channel_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		mw_ip_write32(mwchan->mwdev->mw_ip_info, reg & 0xFFFF, writeval);
	} else {
		*readval = mw_ip_read32(mwchan->mwdev->mw_ip_info, reg & 0xFFFF);
	}
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_info mw_sharedmem_iio_dev_info = {
	.driver_module = THIS_MODULE,
	.debugfs_reg_access = &mw_sharedmem_iio_channel_reg_access,
};



/** ******** MW SharedMem Buffer functions ********/

void mw_sharedmem_buffer_free(struct iio_buffer *buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	/* maybe do something with sharedmem_buff? */
	iio_buffer_put(buffer);
}

static int mw_sharedmem_buffer_write(struct iio_buffer *buffer, size_t n,
	const char __user *user_buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_region *region = &sharedmem_buff->mwchan->region;
	size_t offset = sharedmem_buff->mwchan->offset;
	int ret;

	if (n < buffer->bytes_per_datum)
		return -EINVAL;

	mutex_lock(&sharedmem_buff->lock);
	
	n = ALIGN(n, buffer->bytes_per_datum);
	if (n > region->size - offset)
		n = region->size - offset;
	
	if (copy_from_user(region->virt + offset, user_buffer, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}
	
	ret = n;
	
out_unlock:
	mutex_unlock(&sharedmem_buff->lock);

	return ret;
}

static int mw_sharedmem_buffer_read(struct iio_buffer *buffer, size_t n, 
	char __user *user_buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_region *region = &sharedmem_buff->mwchan->region;
	size_t offset = sharedmem_buff->mwchan->offset;
	int ret;
	
	if (n < buffer->bytes_per_datum)
		return -EINVAL;

	mutex_lock(&sharedmem_buff->lock);
	
	n = rounddown(n, buffer->bytes_per_datum);
	if (n > region->size - offset)
		n = region->size - offset;
	
	if (copy_to_user(user_buffer, region->virt + offset, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}
	
	ret = n;
	
out_unlock:
	mutex_unlock(&sharedmem_buff->lock);
	
	return ret;
}

int mw_sharedmem_buffer_set_bytes_per_datum(struct iio_buffer *buffer, size_t bpd)
{
	buffer->bytes_per_datum = bpd;
	return 0;
}

static int mw_sharedmem_buffer_set_length(struct iio_buffer *buffer, int length)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_region *region = &sharedmem_buff->mwchan->region;
	if (length < 1)
		length = 1;
	if (length > region->size)
		length = region->size;
	buffer->length = length;
	return 0;
}

static int mw_sharedmem_buffer_enable(struct iio_buffer *buffer, struct iio_dev *indio_dev)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	mutex_lock(&sharedmem_buff->lock);
	sharedmem_buff->enabled = true;
	mutex_unlock(&sharedmem_buff->lock);
	return 0;
}

static int mw_sharedmem_buffer_disable(struct iio_buffer *buffer, struct iio_dev *indio_dev)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	mutex_lock(&sharedmem_buff->lock);
	sharedmem_buff->enabled = false;
	mutex_unlock(&sharedmem_buff->lock);
	return 0;
}

static size_t mw_sharedmem_buffer_data_available(struct iio_buffer *buf)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_region *region = &sharedmem_buff->mwchan->region;
	return region->size;
}

static bool mw_sharedmem_buffer_space_available(struct iio_buffer *buf)
{
	return true;
}

static void mw_sharedmem_buffer_release(struct iio_buffer *buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	mutex_destroy(&sharedmem_buff->lock);
	kfree(sharedmem_buff);
}

static const struct iio_buffer_access_funcs mw_sharedmem_buffer_access = {
	.read 					= mw_sharedmem_buffer_read,
	.write 					= mw_sharedmem_buffer_write,
	.set_bytes_per_datum 	= mw_sharedmem_buffer_set_bytes_per_datum,
	.set_length 			= mw_sharedmem_buffer_set_length,
	.enable 				= mw_sharedmem_buffer_enable,
	.disable 				= mw_sharedmem_buffer_disable,
	.data_available 		= mw_sharedmem_buffer_data_available,
	.space_available 		= mw_sharedmem_buffer_space_available,
	.release 				= mw_sharedmem_buffer_release,

	.modes = INDIO_BUFFER_HARDWARE,
}

static int mw_sharedmem_buffer_init(struct mw_sharedmem_buffer *sharedmem_buff)
{
	iio_buffer_init(&sharedmem_buff->buffer);
	mutex_init(&sharedmem_buff->lock);
	sharedmem_buff->buffer.access = &mw_sharedmem_buffer_access;
	return 0;
}

struct iio_buffer *mw_sharedmem_buffer_alloc(struct device *dev)
{
	struct mw_sharedmem_buffer *sharedmem_buff;
	
	sharedmem_buff = kzalloc(sizeof(*sharedmem_buff), GFP_KERNEL);
	if (!sharedmem_buff)
		return ERR_PTR(-ENOMEM);
	
	mw_sharedmem_buffer_init(sharedmem_buffer);
	
	return sharedmem_buff->buffer;
	
err_free:
	kfree(sharedmem_buff);
	return ERR_PTR(status);	
}

/** *************************************** */


static int devm_mw_sharedmem_configure_buffer(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	struct iio_buffer *buffer;
	int status;

	buffer = mw_sharedmem_buffer_alloc(indio_dev->dev.parent);
	if (IS_ERR(buffer)) {
		dev_err(&indio_dev->dev, "Failed to configure IIO buffer: %ld\n", PTR_ERR(buffer));
		return PTR_ERR(buffer);
	}

	status = devm_add_action(indio_dev->dev.parent,(devm_action_fn)mw_sharedmem_buffer_free, buffer);
	if(status){
		mw_sharedmem_buffer_free(buffer);
		return status;
	}

	iio_device_attach_buffer(indio_dev, buffer);

	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->direction = mwchan->iio_direction;
	indio_dev->setup_ops = &mw_sharedmem_iio_buffer_setup_ops;

	return 0;
}

static const char mw_sharedmem_iio_data_channel_compat[] = "mathworks,iio-data-channel-v1.00";

static int mw_sharedmem_count_data_channels(struct iio_dev *indio_dev) {
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	struct device_node *data_node;
	int count = 0;
	for_each_child_of_node(mwchan->dev.of_node,data_node) {
		if(of_device_is_compatible(data_node, mw_sharedmem_iio_data_channel_compat))
			count++;
	}
	return count;
}

static int mw_sharedmem_setup_scan_type(struct iio_dev *indio_dev, struct device_node *node, struct iio_chan_spec *channel) {
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;
	unsigned int storagebits, realbits, shift;
	char sign;
	const char *fmt;
	status = of_property_read_string(node, "mathworks,data-format", &fmt);
	if(status) {
		dev_err(&mwchan->dev, "Missing data-format specifier for %s\n", node->name);
		return status;
	}
	status = sscanf(fmt, "%c%u/%u>>%u", &sign, &storagebits, &realbits, &shift);

	if (status != 4) {
		dev_err(&mwchan->dev, "Invalid data-format specifier for %s\n", node->name);
		return -EINVAL;
	}
	channel->scan_type.sign = sign;
	channel->scan_type.storagebits = storagebits;
	channel->scan_type.realbits = realbits;
	channel->scan_type.shift = shift;
	return 0;
}

static int mw_sharedmem_setup_data_channels(struct iio_dev *indio_dev){
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	struct iio_chan_spec *channel;
	struct device_node *data_node;
	int status;
	u32 scan_index = 0;
	unsigned long *available_scan_masks;


	for_each_child_of_node(mwchan->dev.of_node,data_node) {
		status = of_device_is_compatible(data_node, mw_sharedmem_iio_data_channel_compat);
		if(!status)
			continue;
		status = of_property_read_u32(data_node, "reg", &scan_index);
		if(status){
			dev_err(&mwchan->dev, "Missing 'reg' property in node %s\n", data_node->name);
			return -EINVAL;
		}
		if (scan_index >= mwchan->num_data_chan){
			dev_err(&mwchan->dev, "Invalid 'reg' property in node %s: %d\n", data_node->name, scan_index);
			return -EINVAL;
		}
		channel = (struct iio_chan_spec *)&indio_dev->channels[scan_index];
		if(channel->indexed == 1) {
			dev_err(&mwchan->dev, "Duplicate 'reg' property in node %s: %d\n", data_node->name, scan_index);
			return -EINVAL;
		}
		channel->indexed = 1;
		channel->type = IIO_GENERIC_DATA;
		if (mwchan->iio_direction == IIO_DEVICE_DIRECTION_OUT)
			channel->output = 1;
		channel->channel = scan_index;
		channel->scan_index = scan_index;
		status = of_property_read_string(data_node, "mathworks,chan-name", &channel->extend_name);
		if (status)
			channel->extend_name = NULL;
		status = mw_stream_setup_scan_type(indio_dev, data_node, channel);
		if(status)
			return status;
	}

	/* Only allow all channels or no channels */
	available_scan_masks = devm_kzalloc(&mwchan->dev, sizeof(unsigned long)*2, GFP_KERNEL);
	if(!available_scan_masks)
		return -ENOMEM;
	available_scan_masks[0] = (1 << mwchan->num_data_chan) -1;
	indio_dev->available_scan_masks = available_scan_masks;

	return 0;
}

static void mw_sharedmem_iio_unregister(void *opaque) {
	struct device *dev = opaque;

	/* Unregister the IIO device */
	devres_release_group(dev, mw_sharedmem_iio_unregister);
}

static int devm_mw_sharedmem_iio_register(struct iio_dev *indio_dev) {
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;
	int chIdx = 0;

	if(!devres_open_group(&mwchan->dev, mw_sharedmem_iio_unregister, GFP_KERNEL))
		return -ENOMEM;

	indio_dev->dev.parent = &mwchan->dev;
	indio_dev->name = dev_name(&mwchan->dev);
	indio_dev->info = &mw_sharedmem_iio_dev_info;

	mwchan->num_data_chan = mw_stream_count_data_channels(indio_dev);

	indio_dev->num_channels = mwchan->num_data_chan;
	indio_dev->num_channels++; /* info channel */

	indio_dev->channels = devm_kzalloc(&mwchan->dev, (indio_dev->num_channels) * sizeof(struct iio_chan_spec), GFP_KERNEL);
	if(!indio_dev->channels)
		return -ENOMEM;
	
	status = mw_sharedmem_setup_data_channels(indio_dev);
	if(status)
		return status;
	chIdx += mwchan->num_data_chan;

	status = devm_mw_sharedmem_configure_buffer(indio_dev);
	if (status){
		return status;
	}

	status = devm_iio_device_register(&mwchan->dev, indio_dev);
	if(status)
		return status;

	devres_close_group(&mwchan->dev, mw_sharedmem_iio_unregister);

	/* Setup the parent device to tear us down on removal */
	status = devm_add_action(mwchan->dev.parent, mw_sharedmem_iio_unregister, &mwchan->dev);
	if(status){
		mw_sharedmem_iio_unregister(&mwchan->dev);
		return status;
	}

	return 0;
}

/* Nothing to actually do upon release */
static void mw_sharedmem_iio_channel_release(struct device *dev)
{
}

static struct iio_dev *devm_mw_sharedmem_iio_alloc(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node,
		struct mw_sharedmem_iio_channel_info *info)
{
	struct iio_dev *indio_dev;
	struct mw_sharedmem_iio_chandev *mwchan;
	const char *devname;
	struct device_node *np;
	struct resource r;
	int status;


	if(!devres_open_group(IP2DEVP(mwdev), devm_mw_sharedmem_iio_alloc, GFP_KERNEL))
		return ERR_PTR(-ENOMEM);

	indio_dev = devm_iio_device_alloc(IP2DEVP(mwdev), sizeof(struct mw_sharedmem_iio_chandev));
	if (!indio_dev){
		dev_err(IP2DEVP(mwdev), "Failed to allocate memory for channel %s\n",node->name);
		return ERR_PTR(-ENOMEM);
	}

	mwchan = iio_priv(indio_dev);
	mwchan->mwdev = mwdev;
	mwchan->iio_direction = info->iio_direction;

	/* Find reserved memory region node */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(IP2DEVP(mwdev), "Missing memory-region property for node: %s\n", node->name);
		return ERR_PTR(-ENODEV);
	}
	
	/* Get the address assigned to the memory region */
	status = of_address_to_resource(np, 0, &r);
	if (status) {
		dev_err(IP2DEVP(mwdev), "No memory address assigned to region: %s\n",np->name);
		return ERR_PTR(status);
	}
	
	/* Copy the memory region information and get a virtual address */
	mwchan->region.phys = (phys_addr_t)r.start;
	mwchan->region.size =  (size_t)resource_size(&r);
	mwchan->region.virt = memremap(mwchan->region.phys, mwchan->region.size, MEMREMAP_WB);
	dev_info(IP2DEVP(mwdev), "Allocated reserved memory, vaddr: 0x%0llX, paddr: 0x%0llX\n", (u64)mwchan->region.vaddr, mwchan->region.paddr);
	
	/* Find the name of the DMA channel, there should only be one per node */
/*	status = of_property_read_string_index(node, "dma-names", 0, &mwchan->dmaname);
	if (status) {
		dev_err(IP2DEVP(mwdev), "Missing dma-names property for node: %s\n",node->name);
		return ERR_PTR(status);
	}
	if (mwchan->iio_direction == IIO_DEVICE_DIRECTION_IN) {
		status = of_property_read_u32(node, "mathworks,sample-cnt-reg", &mwchan->tlast_cntr_addr);
		if(status)
			mwchan->tlast_cntr_addr = -EINVAL;
	} else {
		mwchan->tlast_cntr_addr = -EINVAL;
	}*/

	device_initialize(&mwchan->dev);

	mwchan->dev.parent = IP2DEVP(mwdev);
	mwchan->dev.of_node = node;
	mwchan->dev.id = ida_simple_get(&mw_sharedmem_iio_channel_ida, 0, 0, GFP_KERNEL);
	if (mwchan->dev.id < 0) {
		return ERR_PTR(mwchan->dev.id);
	}
	status = devm_add_action(IP2DEVP(mwdev),mw_sharedmem_iio_chan_ida_remove, mwchan);
	if(status){
		mw_sharedmem_iio_chan_ida_remove(mwchan);
		return ERR_PTR(status);
	}
	mwchan->dev.release = mw_sharedmem_iio_channel_release;
	/* clone the parent's DMA config */
	memcpy(&mwchan->dev.archdata, &IP2DEVP(mwdev)->archdata, sizeof(struct dev_archdata));
	mwchan->dev.coherent_dma_mask = IP2DEVP(mwdev)->coherent_dma_mask;
	mwchan->dev.dma_mask = IP2DEVP(mwdev)->dma_mask;
	mwchan->dev.dma_pfn_offset = IP2DEVP(mwdev)->dma_pfn_offset;


	status = of_property_read_string(node, "mathworks,dev-name", &devname);
	if (!status) {
		/* Use the specified channel name */
		status = dev_set_name(&mwchan->dev, "%s:%s", dev_name(mwchan->mwdev->mw_ip_info->char_device), devname);
	} else {
		/* Use the node name + dev ID */
		status = dev_set_name(&mwchan->dev, "%s:%s%d", dev_name(mwchan->mwdev->mw_ip_info->char_device), node->name, mwchan->dev.id);
	}
	if (status)
		return ERR_PTR(status);

	status = device_add(&mwchan->dev);
	if (status)
		return ERR_PTR(status);

	status = devm_add_action(IP2DEVP(mwdev), (devm_action_fn)device_unregister, &mwchan->dev);
	if (status) {
		device_unregister(&mwchan->dev);
		return ERR_PTR(status);
	}

	devres_close_group(IP2DEVP(mwdev), devm_mw_sharedmem_iio_alloc);

	return indio_dev;
}

static int mw_sharedmem_iio_channel_probe(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node,
		struct mw_sharedmem_iio_channel_info *info)
{
	int status;
	struct iio_dev *indio_dev;

	indio_dev = devm_mw_sharedmem_iio_alloc(mwdev, node, info);
	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);

	status = devm_mw_sharedmem_iio_register(indio_dev);
	if (status)
		return status;

	return 0;
}

static struct mw_sharedmem_iio_channel_info mw_sharedmem_iio_write_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_OUT,
};

static struct mw_sharedmem_iio_channel_info mw_sharedmem_iio_read_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_IN,
};

static const struct of_device_id mw_sharedmem_iio_channel_of_match[] = {
	{ .compatible = "mathworks,sharedmem-write-channel-v1.00", .data = &mw_sharedmem_iio_write_info},
	{ .compatible = "mathworks,sharedmem-read-channel-v1.00", .data = &mw_sharedmem_iio_read_info},
    {},
};

int mw_sharedmem_iio_channels_probe(struct mathworks_ipcore_dev *mwdev)
{
	int status;

	struct device_node *child;
	const struct of_device_id *match;


	for_each_child_of_node(IP2DEVP(mwdev)->of_node,child) {
		match = of_match_node(mw_sharedmem_iio_channel_of_match, child);
		if(match){
			status = mw_sharedmem_iio_channel_probe(mwdev, child, (struct mw_sharedmem_iio_channel_info *)match->data);
			if(status)
				return status;
		}
	}

	return 0;
}

EXPORT_SYMBOL_GPL(mw_sharedmem_iio_channels_probe);

static int __init mw_sharedmem_iio_channel_init(void)
{
	return 0;
}

static void __exit mw_sharedmem_iio_channel_exit(void)
{

}

module_init(mw_sharedmem_iio_channel_init);
module_exit(mw_sharedmem_iio_channel_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MathWorks Shared Memory IIO Channel");
MODULE_ALIAS(DRIVER_NAME);
