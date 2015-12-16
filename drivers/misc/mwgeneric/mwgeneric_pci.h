#include "mwgeneric_common.h"
#include <linux/pci.h>

#define ip_to_pdev(ip)  container_of(ip->dev, struct pci_dev, dev)


#define pci_get_ss_vid(pdev, val) pci_read_config_word(pdev,0x2C,val)
#define pci_get_ss_did(pdev, val) pci_read_config_word(pdev,0x2E,val)

#define DRIVER_NAME "mwgeneric_pci"
