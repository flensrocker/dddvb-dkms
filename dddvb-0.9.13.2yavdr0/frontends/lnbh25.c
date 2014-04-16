/*
 * lnbh25.c
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "dvb_frontend.h"
#include "lnbh25.h"

struct lnbh25 {
	struct i2c_adapter	*i2c;
	u8			 adr;
};

static int lnbh25_set_voltage(struct dvb_frontend *fe,
			      fe_sec_voltage_t voltage)
{
	struct lnbh25 *lnbh = (struct lnbh25 *) fe->sec_priv;

	return 0;
}

static int lnbh25_enable_high_lnb_voltage(struct dvb_frontend *fe, long arg)
{
	struct lnbh25 *lnbh = (struct lnbh25 *) fe->sec_priv;

	return 0;
}

static int lnbh25_set_tone(struct dvb_frontend *fe,
			   fe_sec_tone_mode_t tone)
{
	struct lnbh25 *lnbh = (struct lnbh25 *) fe->sec_priv;

	return 0;
}

static void lnbh25_release(struct dvb_frontend *fe)
{
	kfree(fe->sec_priv);
	fe->sec_priv = NULL;
}

struct dvb_frontend *lnbh25_attach(struct dvb_frontend *fe,
				   struct i2c_adapter *i2c)
{
	struct lnbh25 *lnbh = kmalloc(sizeof(struct lnbh25), GFP_KERNEL);
	if (!lnbh)
		return NULL;

	fe->sec_priv = lnbh;

	fe->ops.set_voltage = lnbh25_set_voltage;
	fe->ops.enable_high_lnb_voltage = lnbh25_enable_high_lnb_voltage;
	fe->ops.set_tone = lnbh25_set_tone;
	fe->ops.release_sec = lnbh_release;

	pr_info("LNB25 on %02x\n", lnbh->adr);

	return fe;
}
EXPORT_SYMBOL(lnbh25_attach);

MODULE_DESCRIPTION("LNBH25");
MODULE_AUTHOR("Ralph Metzler");
MODULE_LICENSE("GPL");
