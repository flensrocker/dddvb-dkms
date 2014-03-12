#ifndef _CXD2843_H_
#define _CXD2843_H_

#include <linux/types.h>
#include <linux/i2c.h>

struct cxd2843_cfg {
	u8  adr;
	u32 ts_clock;
};

extern struct dvb_frontend *cxd2843_attach(struct i2c_adapter *i2c,
					   struct cxd2843_cfg *cfg);

#endif
