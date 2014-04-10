#ifndef _STV0910_H_
#define _STV0910_H_

#include <linux/types.h>
#include <linux/i2c.h>

struct stv0910_cfg {
	u8  adr;
	u32 ts_clock;
	u8  parallel;
};

#if defined(CONFIG_DVB_STV0910) || (defined(CONFIG_DVB_STV0910_MODULE) && defined(MODULE))

extern struct dvb_frontend *stv0910_attach(struct i2c_adapter *i2c,
					   struct stv0910_cfg *cfg);
#else

static inline struct dvb_frontend *stv0910_attach(struct i2c_adapter *i2c,
					   struct stv0910_cfg *cfg) 
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif

#endif
