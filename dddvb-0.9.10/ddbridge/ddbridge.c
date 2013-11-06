/*
 * ddbridge.c: Digital Devices PCIe bridge driver
 *
 * Copyright (C) 2010-2013 Digital Devices GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

//#define DDB_ALT_DMA
#define DDB_USE_WORK
//#define DDB_TEST_THREADED
#undef CONFIG_PCI_MSI

#include "ddbridge.h"
#include "ddbridge-regs.h"

#include "tda18271c2dd.h"
#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"
#include "drxk.h"
#include "stv0367.h"
#include "stv0367dd.h"
#include "tda18212.h"
#include "tda18212dd.h"
#include "cxd2843.h"

static struct workqueue_struct *ddb_wq;

static int adapter_alloc;
module_param(adapter_alloc, int, 0444);
MODULE_PARM_DESC(adapter_alloc, "0-one adapter per io, 1-one per tab with io, 2-one per tab, 3-one for all");

#include "ddbridge-core.c"

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void ddb_unmap(struct ddb *dev)
{
	if (dev->regs)
		iounmap(dev->regs);
	vfree(dev);
}


static void __devexit ddb_remove(struct pci_dev *pdev)
{
	struct ddb *dev = (struct ddb *) pci_get_drvdata(pdev);

	ddb_ports_detach(dev);
	ddb_i2c_release(dev);

	ddbwritel(dev, 0, INTERRUPT_ENABLE);
	ddbwritel(dev, 0, MSI1_ENABLE);
	if (dev->msi == 2)
		free_irq(dev->pdev->irq + 1, dev);
	free_irq(dev->pdev->irq, dev);
#ifdef CONFIG_PCI_MSI
	if (dev->msi)
		pci_disable_msi(dev->pdev);
#endif
	ddb_ports_release(dev);
	ddb_buffers_free(dev);
	ddb_device_destroy(dev);

	ddb_unmap(dev);
	pci_set_drvdata(pdev, 0);
	pci_disable_device(pdev);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
#define __devinit 
#define __devinitdata 
#endif 

static int __devinit ddb_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	struct ddb *dev;
	int stat = 0;
	int irq_flag = IRQF_SHARED;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;

	dev = vzalloc(sizeof(struct ddb));
	if (dev == NULL)
		return -ENOMEM;

	dev->has_dma = 1;
	dev->pdev = pdev;
	dev->dev = &pdev->dev;
	pci_set_drvdata(pdev, dev);
	dev->id = id;
	dev->info = (struct ddb_info *) id->driver_data;
	printk(KERN_INFO "DDBridge driver detected: %s\n", dev->info->name);

	dev->regs_len = pci_resource_len(dev->pdev, 0);
	dev->regs = ioremap(pci_resource_start(dev->pdev, 0),
			    pci_resource_len(dev->pdev, 0));
	if (!dev->regs) {
		printk("DDBridge: not enough memory for register map\n");
		stat = -ENOMEM;
		goto fail;
	}
	if (ddbreadl(dev, 0) == 0xffffffff) {
		printk("DDBridge: cannot read registers\n");
		stat = -ENODEV;
		goto fail;
	}

	dev->hwid = ddbreadl(dev, 0);
	dev->regmapid = ddbreadl(dev, 4);
	
	printk(KERN_INFO "HW %08x REGMAP %08x\n",
	       dev->hwid, dev->regmapid);
	
	ddbwritel(dev, 0x00000000, INTERRUPT_ENABLE);
	ddbwritel(dev, 0x00000000, MSI1_ENABLE);
	ddbwritel(dev, 0x00000000, MSI2_ENABLE);
	ddbwritel(dev, 0x00000000, MSI3_ENABLE);
	ddbwritel(dev, 0x00000000, MSI4_ENABLE);
	ddbwritel(dev, 0x00000000, MSI5_ENABLE);
	ddbwritel(dev, 0x00000000, MSI6_ENABLE);
	ddbwritel(dev, 0x00000000, MSI7_ENABLE);

#ifdef CONFIG_PCI_MSI
	if (pci_msi_enabled()) {
		stat = pci_enable_msi_block(dev->pdev, 2);
		if (stat == 0) {
			dev->msi = 1;
			printk("DDBrige using 2 MSI interrupts\n");
		}
		if (stat == 1) 
			stat = pci_enable_msi(dev->pdev);
		if (stat < 0) {
			printk(KERN_INFO ": MSI not available.\n");
		} else {
			irq_flag = 0;
			dev->msi++;
		}
	}
#endif
	if (dev->msi == 2) {
		stat = request_irq(dev->pdev->irq, irq_handler0,
				   irq_flag, "ddbridge", (void *) dev);
		if (stat < 0)
			goto fail0;
		stat = request_irq(dev->pdev->irq + 1, irq_handler1,
				   irq_flag, "ddbridge", (void *) dev);
		if (stat < 0) {
			free_irq(dev->pdev->irq, dev);
			goto fail0;
		} 
	} else {
#ifdef DDB_TEST_THREADED
		stat = request_threaded_irq(dev->pdev->irq, irq_handler, 
					    irq_thread,
					    irq_flag, 
					    "ddbridge", (void *) dev);
#else
		stat = request_irq(dev->pdev->irq, irq_handler, 
				   irq_flag, "ddbridge", (void *) dev);
#endif
		if (stat < 0)
			goto fail0;
	}
	ddbwritel(dev, 0, DMA_BASE_READ);
	if (dev->info->type != DDB_MOD)
		ddbwritel(dev, 0, DMA_BASE_WRITE);
	
	//ddbwritel(dev, 0xffffffff, INTERRUPT_ACK);
	if (dev->msi == 2) {
		ddbwritel(dev, 0x0fffff00, INTERRUPT_ENABLE);
		ddbwritel(dev, 0x0000000f, MSI1_ENABLE);
	} else {
		ddbwritel(dev, 0x0fffff0f, INTERRUPT_ENABLE);
		ddbwritel(dev, 0x00000000, MSI1_ENABLE);
	}
	if (ddb_i2c_init(dev) < 0)
		goto fail1;
	ddb_ports_init(dev);
	if (ddb_buffers_alloc(dev) < 0) {
		printk(KERN_INFO ": Could not allocate buffer memory\n");
		goto fail2;
	}
	if (ddb_ports_attach(dev) < 0)
		goto fail3;

	/* ignore if this fails */
	ddb_device_create(dev);

	if (dev->info->fan_num)	{
		ddbwritel(dev, 1, GPIO_DIRECTION);
		ddbwritel(dev, 1, GPIO_OUTPUT);
	}
	if (dev->info->type == DDB_MOD)
		ddbridge_mod_init(dev);

	return 0;

fail3:
	ddb_ports_detach(dev);
	printk(KERN_ERR "fail3\n");
	ddb_ports_release(dev);
fail2:
	printk(KERN_ERR "fail2\n");
	ddb_buffers_free(dev);
	ddb_i2c_release(dev);
fail1:
	printk(KERN_ERR "fail1\n");
	ddbwritel(dev, 0, INTERRUPT_ENABLE);
	ddbwritel(dev, 0, MSI1_ENABLE);
	free_irq(dev->pdev->irq, dev);
	if (dev->msi == 2) 
		free_irq(dev->pdev->irq + 1, dev);
fail0:
	printk(KERN_ERR "fail0\n");
	if (dev->msi)
		pci_disable_msi(dev->pdev);
fail:
	printk(KERN_ERR "fail\n");
	ddb_unmap(dev);
	pci_set_drvdata(pdev, 0);
	pci_disable_device(pdev);
	return -1;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

struct ddb_regset octopus_i2c = {
	.base = 0x80,
	.num  = 0x04,
	.size = 0x20,
};

static struct ddb_regmap octopus_map = {
//	.i2c = octopus_i2c,
};

static struct ddb_info ddb_none = {
	.type     = DDB_NONE,
	.name     = "unknown Digital Devices PCIe card, install newer driver",
//	.regmap   = octopus_map,
};

static struct ddb_info ddb_octopus = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus DVB adapter",
	.port_num = 4,
	.i2c_num  = 4,
};

static struct ddb_info ddb_octopusv3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus V3 DVB adapter",
	.port_num = 4,
	.i2c_num  = 4,
};

static struct ddb_info ddb_octopus_le = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus LE DVB adapter",
	.port_num = 2,
	.i2c_num  = 2,
};

static struct ddb_info ddb_octopus_oem = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus OEM",
	.port_num = 4,
	.i2c_num  = 4,
	.led_num  = 1,
	.fan_num  = 1,
	.temp_num = 1,
	.temp_bus = 0,
};

static struct ddb_info ddb_octopus_mini = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus Mini",
	.port_num = 4,
	.i2c_num  = 4,
};

static struct ddb_info ddb_v6 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6 DVB adapter",
	.port_num = 3,
	.i2c_num  = 3,
};

static struct ddb_info ddb_v6_5 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6.5 DVB adapter",
	.port_num = 4,
	.i2c_num  = 4,
};

static struct ddb_info ddb_satixS2v3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Mystique SaTiX-S2 V3 DVB adapter",
	.port_num = 3,
	.i2c_num  = 3,
};

static struct ddb_info ddb_ci = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI",
	.port_num = 4,
	.i2c_num  = 2,
};

static struct ddb_info ddb_cis = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI single",
	.port_num = 3,
	.i2c_num  = 2,
};

static struct ddb_info ddb_dvbct = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices DVBCT V6.1 DVB adapter",
	.port_num = 3,
	.i2c_num  = 3,
};

static struct ddb_info ddb_mod = {
	.type     = DDB_MOD,
	.name     = "Digital Devices DVB-C modulator",
	.port_num = 10,
	.temp_num = 1,
};

static struct ddb_info ddb_octonet = {
	.type     = DDB_OCTONET,
	.name     = "Digital Devices Octopus Net",
	.port_num = 4,
	.i2c_num  = 4,
};

#define DDVID 0xdd01 /* Digital Devices Vendor ID */

#define DDB_ID(_vend, _dev, _subvend, _subdev, _driverdata) { \
	.vendor      = _vend,    .device    = _dev, \
	.subvendor   = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long)&_driverdata }

static const struct pci_device_id ddb_id_tbl[] __devinitdata = {
	DDB_ID(DDVID, 0x0002, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0005, DDVID, 0x0004, ddb_octopusv3),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0002, ddb_octopus_le),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0003, ddb_octopus_oem),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0010, ddb_octopus_mini),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0020, ddb_v6),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0021, ddb_v6_5),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0030, ddb_dvbct),
	DDB_ID(DDVID, 0x0003, DDVID, 0xdb03, ddb_satixS2v3),
	DDB_ID(DDVID, 0x0011, DDVID, 0x0040, ddb_ci),
	DDB_ID(DDVID, 0x0011, DDVID, 0x0041, ddb_cis),
	DDB_ID(DDVID, 0x0201, DDVID, 0x0001, ddb_mod),
	/* in case sub-ids got deleted in flash */
	DDB_ID(DDVID, 0x0003, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0011, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0201, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	{0}
};
MODULE_DEVICE_TABLE(pci, ddb_id_tbl);

static struct pci_driver ddb_pci_driver = {
	.name        = "ddbridge",
	.id_table    = ddb_id_tbl,
	.probe       = ddb_probe,
	.remove      = ddb_remove,
};

static __init int module_init_ddbridge(void)
{
	int stat = -1;

	printk(KERN_INFO "Digital Devices PCIE bridge driver 0.9.9, "
	       "Copyright (C) 2010-13 Digital Devices GmbH\n");
	if (ddb_class_create() < 0)
		return -1;
	ddb_wq = create_workqueue("ddbridge");
	if (ddb_wq == NULL)
		goto exit1;
	stat = pci_register_driver(&ddb_pci_driver);
	if (stat < 0)
		goto exit2;
	return stat;
exit2:
	destroy_workqueue(ddb_wq);
exit1:
	ddb_class_destroy();
	return stat;
}

static __exit void module_exit_ddbridge(void)
{
	pci_unregister_driver(&ddb_pci_driver);
	destroy_workqueue(ddb_wq);
	ddb_class_destroy();
}

module_init(module_init_ddbridge);
module_exit(module_exit_ddbridge);

MODULE_DESCRIPTION("Digital Devices PCIe Bridge");
MODULE_AUTHOR("Ralph Metzler, Metzler Brothers Systementwicklung");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.9.9");
