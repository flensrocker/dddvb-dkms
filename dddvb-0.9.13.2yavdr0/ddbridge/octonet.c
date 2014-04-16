/*
 * octonet.c: Digital Devices network tuner driver
 *
 * Copyright (C) 2012-14 Digital Devices GmbH
 *                       Ralph Metzler <rmetzler@digitaldevices.de>
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

#include "ddbridge.h"
#include "ddbridge-regs.h"

#include "tda18271c2dd.h"
#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"
#include "drxk.h"
#include "stv0367.h"
#include "stv0367dd.h"
#include "tda18212dd.h"
#include "cxd2843.h"

#include <asm-generic/pci-dma-compat.h>

static int adapter_alloc = 3;
module_param(adapter_alloc, int, 0444);
MODULE_PARM_DESC(adapter_alloc,
"0-one adapter per io, 1-one per tab with io, 2-one per tab, 3-one for all");

#define DVB_NETSTREAM

#include "ddbridge-core.c"

static struct ddb_info ddb_octonet = {
	.type     = DDB_OCTONET,
	.name     = "Digital Devices OctopusNet network DVB adapter",
	.port_num = 4,
	.i2c_num  = 4,
};

static void octonet_unmap(struct ddb *dev)
{
	if (dev->regs)
		iounmap(dev->regs);
	vfree(dev);
}

static int __exit octonet_remove(struct platform_device *pdev)
{
	struct ddb *dev;

	dev = platform_get_drvdata(pdev);

	ddb_nsd_detach(dev);
	ddb_ports_detach(dev);
	ddb_i2c_release(dev);

	ddbwritel(dev, 0, ETHER_CONTROL);
	ddbwritel(dev, 0, INTERRUPT_ENABLE);
	free_irq(platform_get_irq(dev->pfdev, 0), dev);

	ddb_device_destroy(dev);
	octonet_unmap(dev);
	platform_set_drvdata(pdev, 0);
	return 0;
}

static int __init octonet_probe(struct platform_device *pdev)
{
	struct ddb *dev;
	struct resource *regs;

	dev = vzalloc(sizeof(struct ddb));
	if (!dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, dev);
	dev->dev = &pdev->dev;
	dev->pfdev = pdev;
	dev->info = &ddb_octonet;

	mutex_init(&dev->mutex);
	regs = platform_get_resource(dev->pfdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;
	dev->regs_len = (regs->end - regs->start) + 1;
	dev_info(dev->dev, "regs_start=%08x regs_len=%08x\n",
		 regs->start, dev->regs_len);
	dev->regs = ioremap(regs->start, dev->regs_len);
	if (!dev->regs) {
		dev_err(dev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	dev->ids.hwid = ddbreadl(dev, 0);
	dev->ids.regmapid = ddbreadl(dev, 4);
	dev->ids.devid = ddbreadl(dev, 8);
	dev->ids.mac = ddbreadl(dev, 12);

	dev->ids.vendor = dev->ids.devid & 0xffff;
	dev->ids.device = dev->ids.devid >> 16;
	dev->ids.subvendor = dev->ids.devid & 0xffff;
	dev->ids.subdevice = dev->ids.devid >> 16;

	pr_info("HW  %08x REGMAP %08x\n", dev->ids.hwid, dev->ids.regmapid);
	pr_info("MAC %08x DEVID  %08x\n", dev->ids.mac, dev->ids.devid);

	ddbwritel(dev, 0x00000000, INTERRUPT_ENABLE);

	if (request_irq(platform_get_irq(dev->pfdev, 0), irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"octonet-dvb", (void *) dev) < 0)
		goto fail;
	ddbwritel(dev, 0x0fffff0f, INTERRUPT_ENABLE);
	ddbwritel(dev, 0x1, ETHER_CONTROL);
	ddbwritel(dev, 14 + (vlan ? 4 : 0), ETHER_LENGTH);


	mutex_init(&dev->octonet_i2c_lock);
	if (ddb_i2c_init(dev) < 0)
		goto fail1;

	ddb_ports_init(dev);
	if (ddb_ports_attach(dev) < 0)
		goto fail3;

	ddb_nsd_attach(dev);

	ddb_device_create(dev);

	return 0;

fail3:
	ddb_ports_detach(dev);
	dev_err(dev->dev, "fail3\n");
fail1:
	dev_err(dev->dev, "fail1\n");
fail:
	dev_err(dev->dev, "fail\n");
	ddbwritel(dev, 0, ETHER_CONTROL);
	ddbwritel(dev, 0, INTERRUPT_ENABLE);
	octonet_unmap(dev);
	platform_set_drvdata(pdev, 0);
	return 0;
}

static struct platform_driver octonet_driver = {
	.remove	= __exit_p(octonet_remove),
	.probe	= octonet_probe,
	.driver		= {
		.name	= "octonet-dvb",
		.owner	= THIS_MODULE,
	},
};

static __init int init_octonet(void)
{
	int res;

	pr_info("Digital Devices OctopusNet driver " DDBRIDGE_VERSION 
		", Copyright (C) 2010-14 Digital Devices GmbH\n");
	res = ddb_class_create();
	if (res)
		return res;
	res = platform_driver_probe(&octonet_driver, octonet_probe);
	if (res) {
		ddb_class_destroy();
		return res;
	}
	return 0;
}

static __exit void exit_octonet(void)
{
	platform_driver_unregister(&octonet_driver);
	ddb_class_destroy();
}

module_init(init_octonet);
module_exit(exit_octonet);

MODULE_DESCRIPTION("GPL");
MODULE_AUTHOR("Ralph Metzler, Metzler Brothers Systementwicklung");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.5");
