/*
 * ddbridge.h: Digital Devices PCIe bridge driver
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

#ifndef _DDBRIDGE_H_
#define _DDBRIDGE_H_

#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
#define __devexit
#define __devinit
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/swab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/completion.h>

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/dvb/ca.h>
#include <linux/socket.h>
#include <linux/device.h>
#include <linux/io.h>

#include "dvb_netstream.h"
#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_ringbuffer.h"
#include "dvb_ca_en50221.h"
#include "dvb_net.h"
#include "cxd2099.h"

#define DDB_MAX_I2C     4
#define DDB_MAX_PORT   10
#define DDB_MAX_INPUT   8
#define DDB_MAX_OUTPUT 10

struct ddb_regset {
	uint32_t base;
	uint32_t num;
	uint32_t size;
};

struct ddb_regmap {
	struct ddb_regset i2c;
	struct ddb_regset i2c_buf;
	struct ddb_regset dma;
	struct ddb_regset dma_buf;
	struct ddb_regset input;
	struct ddb_regset output;
	struct ddb_regset channel;
	struct ddb_regset ci;
	struct ddb_regset pid_filter;
};

struct ddb_info {
	int   type;
#define DDB_NONE         0
#define DDB_OCTOPUS      1
#define DDB_OCTOPUS_CI   2
#define DDB_MOD          3
#define DDB_OCTONET      4
	char *name;
	int   port_num;
	int   i2c_num;
	int   led_num;
	int   fan_num;
	int   temp_num;
	int   temp_bus;
	struct ddb_regmap regmap;
};


/* DMA_SIZE MUST be smaller than 256k and 
   MUST be divisible by 188 and 128 !!! */

#define DMA_MAX_BUFS 32      /* hardware table limit */

#define INPUT_DMA_BUFS 8
#define INPUT_DMA_SIZE (128*47*21)
#define INPUT_DMA_IRQ_DIV 1

#define OUTPUT_DMA_BUFS 8
#define OUTPUT_DMA_SIZE (128*47*21)
#define OUTPUT_DMA_IRQ_DIV 1

struct ddb;
struct ddb_port;

struct ddb_dma {
	void                  *io;
	u32                    nr;
	dma_addr_t             pbuf[DMA_MAX_BUFS];
	u8                    *vbuf[DMA_MAX_BUFS];
	u32                    num;
	u32                    size;
	u32                    div;
	u32                    bufreg;

#ifdef DDB_USE_WORK
	struct work_struct     work;
#else
	struct tasklet_struct  tasklet;
#endif
	spinlock_t             lock;
	wait_queue_head_t      wq;
	int                    running;
	u32                    stat;
	u32                    ctrl;
	u32                    cbuf;
	u32                    coff;
};

struct ddb_dvb {
	struct dvb_adapter    *adap;
	int                    adap_registered;
	struct dvb_device     *dev;
	struct dvb_frontend   *fe;
	struct dvb_frontend   *fe2;
	struct dmxdev          dmxdev;
	struct dvb_demux       demux;
	struct dvb_net         dvbnet;
	struct dvb_netstream   dvbns;
	struct dmx_frontend    hw_frontend;
	struct dmx_frontend    mem_frontend;
	int                    users;
	int (*gate_ctrl)(struct dvb_frontend *, int);
	int                    attached;
};

struct ddb_ci {
	struct dvb_ca_en50221  en;
	struct ddb_port       *port;
	u32                    nr;
	struct mutex           lock;
};

struct ddb_io {
	struct ddb_port       *port;
	u32                    nr;
	struct ddb_dma        *dma;
//	struct ddb_io         *redirect;
	struct ddb_io         *redo;
	struct ddb_io         *redi;
};

#if 0
struct ddb_input {
	struct ddb_port       *port;
	u32                    nr;
	struct ddb_dma        *dma;
	struct ddb_output     *redo;
	struct ddb_input      *redi;
};

struct ddb_output {
	struct ddb_port       *port;
	u32                    nr;
	struct ddb_dma        *dma;
	struct ddb_output     *redo;
	struct ddb_input      *redi;
};
#else
#define ddb_output ddb_io
#define ddb_input ddb_io
#endif

struct ddb_i2c {
	struct ddb            *dev;
	u32                    nr;
	struct i2c_adapter     adap;
	u32                    regs;
	u32                    rbuf;
	u32                    wbuf;
//	int                    done;
//	wait_queue_head_t      wq;
	struct completion      completion;
};

struct ddb_port {
	struct ddb            *dev;
	u32                    nr;
	struct ddb_i2c        *i2c;
	struct mutex           i2c_gate_lock;
	u32                    class;
#define DDB_PORT_NONE           0
#define DDB_PORT_CI             1
#define DDB_PORT_TUNER          2
#define DDB_PORT_LOOP           3
#define DDB_PORT_MOD            4
	u32                    type;
#define DDB_TUNER_NONE          0
#define DDB_TUNER_DVBS_ST       1
#define DDB_TUNER_DVBS_ST_AA    2
#define DDB_TUNER_DVBCT_TR      3
#define DDB_TUNER_DVBCT_ST      4
#define DDB_CI_INTERNAL         5
#define DDB_CI_EXTERNAL_SONY    6
#define DDB_TUNER_XO2           16
#define DDB_TUNER_DVBS          16
#define DDB_TUNER_DVBCT2_SONY   17
#define DDB_TUNER_ISDBT_SONY    18
#define DDB_TUNER_DVBC2T2_SONY  19
#define DDB_TUNER_ATSC_ST       20
#define DDB_TUNER_DVBC2T2_ST    21

	u32                    adr;

	struct ddb_input      *input[2];
	struct ddb_output     *output;
	struct dvb_ca_en50221 *en;
	struct ddb_dvb         dvb[2];
	u32                    gap;
	u32                    obr;
};


struct mod_base {
	u32                    frequency;

	u32                    flat_start;
	u32                    flat_end;
};

struct mod_state {
	u32                    modulation;

	u32                    do_handle;

	u32                    rate_inc;
	u32                    Control;
	u32                    State;
	u32                    StateCounter;
	s32                    LastPCRAdjust;
	s32                    PCRAdjustSum;
	s32                    InPacketsSum;
	s32                    OutPacketsSum;
	s64                    PCRIncrement;
	s64                    PCRDecrement;
	s32                    PCRRunningCorr;
	u32                    OutOverflowPacketCount;
	u32                    InOverflowPacketCount;
	u32                    LastOutPacketCount;
	u32                    LastInPacketCount;
	u64                    LastOutPackets;
	u64                    LastInPackets;
	u32                    MinInputPackets;
};

#define CM_STARTUP_DELAY 2
#define CM_AVERAGE  20
#define CM_GAIN     10

#define HW_LSB_SHIFT    12
#define HW_LSB_MASK     0x1000

#define CM_IDLE    0
#define CM_STARTUP 1
#define CM_ADJUST  2

#define TS_CAPTURE_LEN  (21*188)

/* net streaming hardware block */

#define DDB_NS_MAX 15

struct ddb_ns {
	struct ddb_input      *input; 
	int                    nr;
	int                    fe;
	u32                    rtcp_udplen;
	u32                    rtcp_len;
	u32                    ts_offset;
	u32                    udplen;
	u8                     p[512];
};

struct ddb {
	struct pci_dev        *pdev;
	struct platform_device *pfdev;
	struct device         *dev;
	const struct pci_device_id *id;
	struct ddb_info       *info;
	int                    msi;
	struct workqueue_struct *wq;
	u32                    has_dma;
	u32                    has_ns;

	struct ddb_regmap      regmap;
	unsigned char         *regs;
	u32                    regs_len;
	struct ddb_port        port[DDB_MAX_PORT];
	struct ddb_i2c         i2c[DDB_MAX_I2C];
	struct ddb_input       input[DDB_MAX_INPUT];
	struct ddb_output      output[DDB_MAX_OUTPUT];
	struct dvb_adapter     adap[DDB_MAX_INPUT];
	struct ddb_dma         dma[DDB_MAX_INPUT + DDB_MAX_OUTPUT];

	void                   (*handler[32])(unsigned long);
	unsigned long          handler_data[32];

	struct device         *ddb_dev;
	u32                    ddb_dev_users;
	u32                    nr;
	u8                     iobuf[1028];

	u8                     leds;
	u32                    ts_irq;
	u32                    i2c_irq;

	u32                    hwid;
	u32                    regmapid;
	u32                    mac;
	u32                    devid;

	int                    ns_num;
	struct ddb_ns          ns[DDB_NS_MAX];
	struct mutex           mutex;

	struct dvb_device     *nsd_dev;
	u8                     tsbuf[TS_CAPTURE_LEN];

	struct mod_base        mod_base;
	struct mod_state       mod[10];
};


/******************************************************************************/

static inline void ddbwriteb(struct ddb *dev, u32 val, u32 adr)
{
	writeb(val, (char *) (dev->regs+(adr)));
}

static inline void ddbwritel(struct ddb *dev, u32 val, u32 adr)
{
	writel(val, (char *) (dev->regs+(adr)));
}

static inline void ddbwritew(struct ddb *dev, u16 val, u32 adr)
{
	writew(val, (char *) (dev->regs+(adr)));
}

static inline u32 ddbreadl(struct ddb *dev, u32 adr)
{
	return readl((char *) (dev->regs+(adr)));
}

static inline u32 ddbreadb(struct ddb *dev, u32 adr)
{
	return readb((char *) (dev->regs+(adr)));
}

#define ddbcpyto(_dev, _adr, _src, _count) \
	memcpy_toio((char *) (_dev->regs + (_adr)), (_src), (_count))

#define ddbcpyfrom(_dev, _dst, _adr, _count) \
	memcpy_fromio((_dst), (char *) (_dev->regs + (_adr)), (_count))

#define ddbmemset(_dev, _adr, _val, _count) \
	memset_io((char *) (_dev->regs + (_adr)), (_val), (_count))


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#define dd_uint8    u8
#define dd_uint16   u16
#define dd_int16    s16
#define dd_uint32   u32
#define dd_int32    s32
#define dd_uint64   u64
#define dd_int64    s64

#define DDMOD_FLASH_START  0x1000

struct DDMOD_FLASH_DS {
	dd_uint32   Symbolrate;             /* kSymbols/s */
	dd_uint32   DACFrequency;           /* kHz        */
	dd_uint16   FrequencyResolution;    /* kHz        */
	dd_uint16   IQTableLength;
	dd_uint16   FrequencyFactor;
	dd_int16    PhaseCorr;              /* TBD        */
	dd_uint32   Control2;
	dd_uint16   PostScaleI;   
	dd_uint16   PostScaleQ;
	dd_uint16   PreScale;
	dd_int16    EQTap[11];
	dd_uint16   FlatStart;   
	dd_uint16   FlatEnd;
	dd_uint32   FlashOffsetPrecalculatedIQTables;       /* 0 = none */
	dd_uint8    Reserved[28];

};

struct DDMOD_FLASH {
	dd_uint32   Magic;
	dd_uint16   Version;
	dd_uint16   DataSets;
	
	dd_uint16   VCORefFrequency;    /* MHz */
	dd_uint16   VCO1Frequency;      /* MHz */
	dd_uint16   VCO2Frequency;      /* MHz */
	
	dd_uint16   DACAux1;    /* TBD */
	dd_uint16   DACAux2;    /* TBD */
	
	dd_uint8    Reserved1[238];
	
	struct DDMOD_FLASH_DS DataSet[1];
};

#define DDMOD_FLASH_MAGIC   0x5F564d5F


int ddbridge_mod_do_ioctl(struct file *file, unsigned int cmd, void *parg);
int ddbridge_mod_init(struct ddb *dev);
void ddbridge_mod_output_stop(struct ddb_output *output);
void ddbridge_mod_output_start(struct ddb_output *output);
void ddbridge_mod_rate_handler(unsigned long data);


int ddbridge_flashread(struct ddb *dev, u8 *buf, u32 addr, u32 len);


#endif


