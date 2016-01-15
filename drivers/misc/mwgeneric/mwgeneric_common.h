#include <linux/types.h>
#include "mwgeneric.h"
#include "mwgeneric_ioctl.h"

/*Device structure for IPCore information*/
extern dev_t mwgeneric_dev_id;
extern unsigned int device_num;
extern struct class *mwgeneric_class;
extern struct mwgeneric_info dev_table[MWGENERIC_MAX_DEVTYPE];

/*********************************************************
* Bus Specific Functions
*********************************************************/
int mwgeneric_get_info(struct ipcore_info *thisIpcore, struct mwgeneric_info **thisDev);
int	mwgeneric_get_param(struct ipcore_info *thisIpcore, void *arg);

/*********************************************************
* Common functions
*********************************************************/
int mwgeneric_setup_cdev(struct ipcore_info *thisIpcore, dev_t *dev_id);

