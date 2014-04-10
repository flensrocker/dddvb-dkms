/*
 * ddbridge-core.c: Digital Devices bridge core functions
 *
 * Copyright (C) 2010-2013 Digital Devices GmbH
 *                         Ralph Metzler <rmetzler@digitaldevices.de>
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

DEFINE_MUTEX(redirect_lock);

static int ci_bitrate = 72000;
module_param(ci_bitrate, int, 0444);
MODULE_PARM_DESC(ci_bitrate, " Bitrate for output to CI.");

static int ts_loop = -1;
module_param(ts_loop, int, 0444);
MODULE_PARM_DESC(ts_loop, "TS in/out test loop on port ts_loop");

static int vlan;
module_param(vlan, int, 0444);
MODULE_PARM_DESC(vlan, "VLAN and QoS IDs enabled");

static int tt;
module_param(tt, int, 0444);
MODULE_PARM_DESC(tt, "");

#define DDB_MAX_ADAPTER 32
static struct ddb *ddbs[DDB_MAX_ADAPTER];

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

#include "ddbridge-mod.c"
#include "ddbridge-i2c.c"
#include "ddbridge-ns.c"


static void ddb_set_dma_table(struct ddb *dev, struct ddb_dma *dma)
{
	u32 i, base;
	u64 mem;

	if (!dma)
		return;
	base = DMA_BASE_ADDRESS_TABLE + dma->nr * 0x100;
	for (i = 0; i < dma->num; i++) {
		mem = dma->pbuf[i];
		ddbwritel(dev, mem & 0xffffffff, base + i * 8);
		ddbwritel(dev, mem >> 32, base + i * 8 + 4);
	}
	dma->bufreg = (dma->div << 16) |
		((dma->num & 0x1f) << 11) |
		((dma->size >> 7) & 0x7ff);
}

static void ddb_set_dma_tables(struct ddb *dev)
{
	u32 i;

	for (i = 0; i < dev->info->port_num * 2; i++)
		ddb_set_dma_table(dev, dev->input[i].dma);
	for (i = 0; i < dev->info->port_num; i++)
		ddb_set_dma_table(dev, dev->output[i].dma);
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void ddb_redirect_dma(struct ddb *dev,
			     struct ddb_dma *sdma,
			     struct ddb_dma *ddma)
{
	u32 i, base;
	u64 mem;

	sdma->bufreg = ddma->bufreg;
	base = DMA_BASE_ADDRESS_TABLE + sdma->nr * 0x100;
	for (i = 0; i < ddma->num; i++) {
		mem = ddma->pbuf[i];
		ddbwritel(dev, mem & 0xffffffff, base + i * 8);
		ddbwritel(dev, mem >> 32, base + i * 8 + 4);
	}
}

static int ddb_unredirect(struct ddb_port *port)
{
	struct ddb_input *oredi, *iredi = 0;
	struct ddb_output *iredo = 0;

	/*pr_info("unredirect %d.%d\n", port->dev->nr, port->nr);*/
	mutex_lock(&redirect_lock);
	if (port->output->dma->running) {
		mutex_unlock(&redirect_lock);
		return -EBUSY;
	}
	oredi = port->output->redi;
	if (!oredi)
		goto done;
	if (port->input[0]) {
		iredi = port->input[0]->redi;
		iredo = port->input[0]->redo;

		if (iredo) {
			iredo->port->output->redi = oredi;
			if (iredo->port->input[0]) {
				iredo->port->input[0]->redi = iredi;
				ddb_redirect_dma(oredi->port->dev,
						 oredi->dma, iredo->dma);
			}
			port->input[0]->redo = 0;
			ddb_set_dma_table(port->dev, port->input[0]->dma);
		}
		oredi->redi = iredi;
		port->input[0]->redi = 0;
	}
	oredi->redo = 0;
	port->output->redi = 0;

	ddb_set_dma_table(oredi->port->dev, oredi->dma);
done:
	mutex_unlock(&redirect_lock);
	return 0;
}

static int ddb_redirect(u32 i, u32 p)
{
	struct ddb *idev = ddbs[(i >> 4) & 0x1f];
	struct ddb_input *input, *input2;
	struct ddb *pdev = ddbs[(p >> 4) & 0x1f];
	struct ddb_port *port;

	if (!idev->has_dma || !pdev->has_dma)
		return -EINVAL;
	if (!idev || !pdev)
		return -EINVAL;

	port = &pdev->port[p & 0x0f];
	if (!port->output)
		return -EINVAL;
	if (ddb_unredirect(port))
		return -EBUSY;

	if (i == 8)
		return 0;

	input = &idev->input[i & 7];
	if (!input)
		return -EINVAL;

	mutex_lock(&redirect_lock);
	if (port->output->dma->running || input->dma->running) {
		mutex_unlock(&redirect_lock);
		return -EBUSY;
	}
	input2 = port->input[0];
	if (input2) {
		if (input->redi) {
			input2->redi = input->redi;
			input->redi = 0;
		} else
			input2->redi = input;
	}
	input->redo = port->output;
	port->output->redi = input;

	ddb_redirect_dma(input->port->dev, input->dma, port->output->dma);
	mutex_unlock(&redirect_lock);
	return 0;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#ifdef DDB_ALT_DMA
static void dma_free(struct pci_dev *pdev, struct ddb_dma *dma, int dir)
{
	int i;

	if (!dma)
		return;
	for (i = 0; i < dma->num; i++) {
		if (dma->vbuf[i]) {
			dma_unmap_single(&pdev->dev, dma->pbuf[i],
					 dma->size,
					 dir ? DMA_TO_DEVICE :
					 DMA_FROM_DEVICE);
			kfree(dma->vbuf[i]);
			dma->vbuf[i] = 0;
		}
	}
}

static int dma_alloc(struct pci_dev *pdev, struct ddb_dma *dma, int dir)
{
	int i;

	if (!dma)
		return 0;
	for (i = 0; i < dma->num; i++) {
		dma->vbuf[i] = kmalloc(dma->size, __GFP_REPEAT);
		if (!dma->vbuf[i])
			return -ENOMEM;
		dma->pbuf[i] = dma_map_single(&pdev->dev, dma->vbuf[i],
					      dma->size,
					      dir ? DMA_TO_DEVICE :
					      DMA_FROM_DEVICE);
		if (dma_mapping_error(&pdev->dev, dma->pbuf[i])) {
			kfree(dma->vbuf[i]);
			return -ENOMEM;
		}
	}
	return 0;
}
#else

static void dma_free(struct pci_dev *pdev, struct ddb_dma *dma, int dir)
{
	int i;

	if (!dma)
		return;
	for (i = 0; i < dma->num; i++) {
		if (dma->vbuf[i]) {
			pci_free_consistent(pdev, dma->size,
					    dma->vbuf[i], dma->pbuf[i]);
			dma->vbuf[i] = 0;
		}
	}
}

static int dma_alloc(struct pci_dev *pdev, struct ddb_dma *dma, int dir)
{
	int i;

	if (!dma)
		return 0;
	for (i = 0; i < dma->num; i++) {
		dma->vbuf[i] = pci_alloc_consistent(pdev, dma->size,
						    &dma->pbuf[i]);
		if (!dma->vbuf[i])
			return -ENOMEM;
	}
	return 0;
}
#endif

static int ddb_buffers_alloc(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			if (dma_alloc(dev->pdev, port->input[0]->dma, 0) < 0)
				return -1;
			if (dma_alloc(dev->pdev, port->input[1]->dma, 0) < 0)
				return -1;
			break;
		case DDB_PORT_CI:
		case DDB_PORT_LOOP:
			if (dma_alloc(dev->pdev, port->input[0]->dma, 0) < 0)
				return -1;
		case DDB_PORT_MOD:
			if (dma_alloc(dev->pdev, port->output->dma, 1) < 0)
				return -1;
			break;
		default:
			break;
		}
	}
	ddb_set_dma_tables(dev);
	return 0;
}

static void ddb_buffers_free(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];

		if (port->input[0])
			dma_free(dev->pdev, port->input[0]->dma, 0);
		if (port->input[1])
			dma_free(dev->pdev, port->input[1]->dma, 0);
		if (port->output)
			dma_free(dev->pdev, port->output->dma, 1);
	}
}

static void ddb_output_start(struct ddb_output *output)
{
	struct ddb *dev = output->port->dev;
	u32 con2;

	con2 = ((output->port->obr << 13) + 71999) / 72000;
	con2 = (con2 << 16) | output->port->gap;

	if (output->dma) {
		spin_lock_irq(&output->dma->lock);
		output->dma->cbuf = 0;
		output->dma->coff = 0;
		ddbwritel(dev, 0, DMA_BUFFER_CONTROL(output->dma->nr));
	}
	if (output->port->class == DDB_PORT_MOD)
		ddbridge_mod_output_start(output);
	else {
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, 2, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, 0x3c, TS_OUTPUT_CONTROL(output->nr));
		ddbwritel(dev, con2, TS_OUTPUT_CONTROL2(output->nr));
	}
	if (output->dma) {
		ddbwritel(dev, output->dma->bufreg,
			  DMA_BUFFER_SIZE(output->dma->nr));
		ddbwritel(dev, 0, DMA_BUFFER_ACK(output->dma->nr));
		ddbwritel(dev, 1, DMA_BASE_READ);
		ddbwritel(dev, 3, DMA_BUFFER_CONTROL(output->dma->nr));
	}
	if (output->port->class != DDB_PORT_MOD) {
		if (output->port->input[0]->port->class == DDB_PORT_LOOP)
			/*ddbwritel(dev, 0x15, TS_OUTPUT_CONTROL(output->nr));
			  ddbwritel(dev, 0x45,
			  TS_OUTPUT_CONTROL(output->nr));*/
			ddbwritel(dev, (1 << 13) | 0x15,
				  TS_OUTPUT_CONTROL(output->nr));
		else
			ddbwritel(dev, 0x1d, TS_OUTPUT_CONTROL(output->nr));
	}
	if (output->dma) {
		output->dma->running = 1;
		spin_unlock_irq(&output->dma->lock);
	}
}

static void ddb_output_stop(struct ddb_output *output)
{
	struct ddb *dev = output->port->dev;

	if (output->dma)
		spin_lock_irq(&output->dma->lock);
	if (output->port->class == DDB_PORT_MOD)
		ddbridge_mod_output_stop(output);
	else
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(output->nr));
	if (output->dma) {
		ddbwritel(dev, 0, DMA_BUFFER_CONTROL(output->dma->nr));
		output->dma->running = 0;
		spin_unlock_irq(&output->dma->lock);
	}
}

static void ddb_input_stop(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;

	if (input->dma)
		spin_lock_irq(&input->dma->lock);
	ddbwritel(dev, 0, TS_INPUT_CONTROL(input->nr));
	if (input->dma) {
		ddbwritel(dev, 0, DMA_BUFFER_CONTROL(input->dma->nr));
		input->dma->running = 0;
		spin_unlock_irq(&input->dma->lock);
	}
	/*pr_info("input_stop %d.%d\n", dev->nr, input->nr);*/
}

static void ddb_input_start(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;

	if (input->dma) {
		spin_lock_irq(&input->dma->lock);
		input->dma->cbuf = 0;
		input->dma->coff = 0;
		ddbwritel(dev, 0, DMA_BUFFER_CONTROL(input->dma->nr));
	}
	ddbwritel(dev, 0, TS_INPUT_CONTROL2(input->nr));
	ddbwritel(dev, 0, TS_INPUT_CONTROL(input->nr));
	ddbwritel(dev, 2, TS_INPUT_CONTROL(input->nr));
	ddbwritel(dev, 0, TS_INPUT_CONTROL(input->nr));

	if (input->dma) {
		ddbwritel(dev, input->dma->bufreg,
			  DMA_BUFFER_SIZE(input->dma->nr));
		ddbwritel(dev, 0, DMA_BUFFER_ACK(input->dma->nr));
		ddbwritel(dev, 1, DMA_BASE_WRITE);
		ddbwritel(dev, 3, DMA_BUFFER_CONTROL(input->dma->nr));
	}
	if (dev->info->type == DDB_OCTONET)
		ddbwritel(dev, 0x01, TS_INPUT_CONTROL(input->nr));
	else
		ddbwritel(dev, 0x09, TS_INPUT_CONTROL(input->nr));
	if (input->dma) {
		input->dma->running = 1;
		spin_unlock_irq(&input->dma->lock);
	}
	/*pr_info("input_start %d.%d\n", dev->nr, input->nr);*/
}


static int ddb_dvb_input_start(struct ddb_input *input)
{
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (!dvb->users)
		ddb_input_start(input);

	return ++dvb->users;
}

static int ddb_dvb_input_stop(struct ddb_input *input)
{
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (--dvb->users)
		return dvb->users;

	ddb_input_stop(input);
	return 0;
}

static void ddb_input_start_all(struct ddb_input *input)
{
	struct ddb_input *i = input;
	struct ddb_output *o;

	mutex_lock(&redirect_lock);
	while (i && (o = i->redo)) {
		ddb_output_start(o);
		i = o->port->input[0];
		if (i)
			ddb_input_start(i);
	}
	ddb_input_start(input);
	mutex_unlock(&redirect_lock);
}

static void ddb_input_stop_all(struct ddb_input *input)
{
	struct ddb_input *i = input;
	struct ddb_output *o;

	mutex_lock(&redirect_lock);
	ddb_input_stop(input);
	while (i && (o = i->redo)) {
		ddb_output_stop(o);
		i = o->port->input[0];
		if (i)
			ddb_input_stop(i);
	}
	mutex_unlock(&redirect_lock);
}

static u32 ddb_output_free(struct ddb_output *output)
{
	u32 idx, off, stat = output->dma->stat;
	s32 diff;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (output->dma->cbuf != idx) {
		if ((((output->dma->cbuf + 1) % output->dma->num) == idx) &&
		    (output->dma->size - output->dma->coff <= 188))
			return 0;
		return 188;
	}
	diff = off - output->dma->coff;
	if (diff <= 0 || diff > 188)
		return 188;
	return 0;
}

static u32 ddb_dma_free(struct ddb_dma *dma)
{
	u32 idx, off, stat = dma->stat;
	s32 p1, p2, diff;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	p1 = idx * dma->size + off;
	p2 = dma->cbuf * dma->size + dma->coff;

	diff = p1 - p2;
	if (diff <= 0)
		diff += dma->num * dma->size;
	return diff;
}

static ssize_t ddb_output_write(struct ddb_output *output,
				const u8 *buf, size_t count)
{
	struct ddb *dev = output->port->dev;
	u32 idx, off, stat = output->dma->stat;
	u32 left = count, len;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	while (left) {
		len = output->dma->size - output->dma->coff;
		if ((((output->dma->cbuf + 1) % output->dma->num) == idx) &&
		    (off == 0)) {
			if (len <= 188)
				break;
			len -= 188;
		}
		if (output->dma->cbuf == idx) {
			if (off > output->dma->coff) {
				len = off - output->dma->coff;
				len -= (len % 188);
				if (len <= 188)
					break;
				len -= 188;
			}
		}
		if (len > left)
			len = left;
		if (copy_from_user(output->dma->vbuf[output->dma->cbuf] +
				   output->dma->coff,
				   buf, len))
			return -EIO;
#ifdef DDB_ALT_DMA
		dma_sync_single_for_device(dev->dev,
					   output->dma->pbuf[
						   output->dma->cbuf],
					   output->dma->size, DMA_TO_DEVICE);
#endif
		left -= len;
		buf += len;
		output->dma->coff += len;
		if (output->dma->coff == output->dma->size) {
			output->dma->coff = 0;
			output->dma->cbuf = ((output->dma->cbuf + 1) %
					     output->dma->num);
		}
		ddbwritel(dev,
			  (output->dma->cbuf << 11) |
			  (output->dma->coff >> 7),
			  DMA_BUFFER_ACK(output->dma->nr));
	}
	return count - left;
}

static u32 ddb_input_free_bytes(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;
	u32 idx, off, stat = input->dma->stat;
	u32 ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(input->dma->nr));

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (ctrl & 4)
		return 0;
	if (input->dma->cbuf != idx)
		return 1;
	return 0;
}

static s32 ddb_output_used_bufs(struct ddb_output *output)
{
	u32 idx, off, stat, ctrl;
	s32 diff;

	spin_lock_irq(&output->dma->lock);
	stat = output->dma->stat;
	ctrl = output->dma->ctrl;
	spin_unlock_irq(&output->dma->lock);

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (ctrl & 4)
		return 0;
	diff = output->dma->cbuf - idx;
	if (diff == 0 && off < output->dma->coff)
		return 0;
	if (diff <= 0)
		diff += output->dma->num;
	return diff;
}

static s32 ddb_input_free_bufs(struct ddb_input *input)
{
	u32 idx, off, stat, ctrl;
	s32 free;

	spin_lock_irq(&input->dma->lock);
	ctrl = input->dma->ctrl;
	stat = input->dma->stat;
	spin_unlock_irq(&input->dma->lock);
	if (ctrl & 4)
		return 0;
	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;
	free = input->dma->cbuf - idx;
	if (free == 0 && off < input->dma->coff)
		return 0;
	if (free <= 0)
		free += input->dma->num;
	return free - 1;
}

static u32 ddb_output_ok(struct ddb_output *output)
{
	struct ddb_input *input = output->port->input[0];
	s32 diff;

	diff = ddb_input_free_bufs(input) - ddb_output_used_bufs(output);
	if (diff > 0)
		return 1;
	return 0;
}

static u32 ddb_input_avail(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;
	u32 idx, off, stat = input->dma->stat;
	u32 ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(input->dma->nr));

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (ctrl & 4) {
		pr_err("IA %d %d %08x\n", idx, off, ctrl);
		ddbwritel(dev, stat, DMA_BUFFER_ACK(input->dma->nr));
		return 0;
	}
	if (input->dma->cbuf != idx || off < input->dma->coff)
		return 188;
	return 0;
}

static size_t ddb_input_read(struct ddb_input *input, u8 *buf, size_t count)
{
	struct ddb *dev = input->port->dev;
	u32 left = count;
	u32 idx, off, free, stat = input->dma->stat;
	int ret;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	while (left) {
		if (input->dma->cbuf == idx)
			return count - left;
		free = input->dma->size - input->dma->coff;
		if (free > left)
			free = left;
#ifdef DDB_ALT_DMA
		dma_sync_single_for_cpu(dev->dev,
					input->dma->pbuf[input->dma->cbuf],
					input->dma->size, DMA_FROM_DEVICE);
#endif
		ret = copy_to_user(buf, input->dma->vbuf[input->dma->cbuf] +
				   input->dma->coff, free);
		if (ret)
			return -EFAULT;
		input->dma->coff += free;
		if (input->dma->coff == input->dma->size) {
			input->dma->coff = 0;
			input->dma->cbuf = (input->dma->cbuf + 1) %
				input->dma->num;
		}
		left -= free;
		ddbwritel(dev,
			  (input->dma->cbuf << 11) | (input->dma->coff >> 7),
			  DMA_BUFFER_ACK(input->dma->nr));
	}
	return count;
}

/****************************************************************************/
/****************************************************************************/

static ssize_t ts_write(struct file *file, const char *buf,
			size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb *dev = output->port->dev;
	size_t left = count;
	int stat;

	if (!dev->has_dma)
		return -EINVAL;
	while (left) {
		if (ddb_output_free(output) < 188) {
			if (file->f_flags & O_NONBLOCK)
				break;
			if (wait_event_interruptible(
				    output->dma->wq,
				    ddb_output_free(output) >= 188) < 0)
				break;
		}
		stat = ddb_output_write(output, buf, left);
		if (stat < 0)
			return stat;
		buf += stat;
		left -= stat;
	}
	return (left == count) ? -EAGAIN : (count - left);
}

static ssize_t ts_read(struct file *file, char *buf,
		       size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];
	struct ddb *dev = output->port->dev;
	int left, read;

	if (!dev->has_dma)
		return -EINVAL;
	count -= count % 188;
	left = count;
	while (left) {
		if (ddb_input_avail(input) < 188) {
			if (file->f_flags & O_NONBLOCK)
				break;
			if (wait_event_interruptible(
				    input->dma->wq,
				    ddb_input_avail(input) >= 188) < 0)
				break;
		}
		read = ddb_input_read(input, buf, left);
		left -= read;
		buf += read;
	}
	return (left == count) ? -EAGAIN : (count - left);
}

static unsigned int ts_poll(struct file *file, poll_table *wait)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];

	unsigned int mask = 0;

	poll_wait(file, &input->dma->wq, wait);
	poll_wait(file, &output->dma->wq, wait);
	if (ddb_input_avail(input) >= 188)
		mask |= POLLIN | POLLRDNORM;
	if (ddb_output_free(output) >= 188)
		mask |= POLLOUT | POLLWRNORM;
	return mask;
}

static int ts_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];

	if ((file->f_flags & O_ACCMODE) == O_RDONLY) {
		if (!input)
			return -EINVAL;
		ddb_input_stop(input);
	} else if ((file->f_flags & O_ACCMODE) == O_WRONLY) {
		if (!output)
			return -EINVAL;
		ddb_output_stop(output);
	}
	return dvb_generic_release(inode, file);
}

static int ts_open(struct inode *inode, struct file *file)
{
	int err;
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];

	if ((file->f_flags & O_ACCMODE) == O_RDONLY) {
		if (!input)
			return -EINVAL;
		if (input->redo || input->redi)
			return -EBUSY;
	} else if ((file->f_flags & O_ACCMODE) == O_WRONLY) {
		if (!output)
			return -EINVAL;
	}
	err = dvb_generic_open(inode, file);
	if (err < 0)
		return err;
	if ((file->f_flags & O_ACCMODE) == O_RDONLY)
		ddb_input_start(input);
	else if ((file->f_flags & O_ACCMODE) == O_WRONLY)
		ddb_output_start(output);
	return err;
}

static int mod_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];

	if ((file->f_flags & O_ACCMODE) == O_WRONLY) {
		if (!output)
			return -EINVAL;
		ddb_output_stop(output);
	}
	return dvb_generic_release(inode, file);
}

static int mod_open(struct inode *inode, struct file *file)
{
	int err;
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;

	if ((file->f_flags & O_ACCMODE) == O_WRONLY) {
		if (!output)
			return -EINVAL;
	}
	err = dvb_generic_open(inode, file);
	if (err < 0)
		return err;
	if ((file->f_flags & O_ACCMODE) == O_WRONLY)
		ddb_output_start(output);
	return err;
}
static const struct file_operations ci_fops = {
	.owner   = THIS_MODULE,
	.read    = ts_read,
	.write   = ts_write,
	.open    = ts_open,
	.release = ts_release,
	.poll    = ts_poll,
	.mmap    = 0,
};

static struct dvb_device dvbdev_ci = {
	.priv    = 0,
	.readers = 1,
	.writers = 1,
	.users   = 2,
	.fops    = &ci_fops,
};


/****************************************************************************/
/****************************************************************************/

static long mod_ioctl(struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	return dvb_usercopy(file, cmd, arg, ddbridge_mod_do_ioctl);
}

static const struct file_operations mod_fops = {
	.owner   = THIS_MODULE,
	.read    = ts_read,
	.write   = ts_write,
	.open    = mod_open,
	.release = mod_release,
	.poll    = ts_poll,
	.mmap    = 0,
	.unlocked_ioctl = mod_ioctl,
};

static struct dvb_device dvbdev_mod = {
	.priv    = 0,
	.readers = 1,
	.writers = 1,
	.users   = 2,
	.fops    = &mod_fops,
};


#if 0
static struct ddb_input *fe2input(struct ddb *dev, struct dvb_frontend *fe)
{
	int i;

	for (i = 0; i < dev->info->port_num * 2; i++) {
		if (dev->input[i].fe == fe)
			return &dev->input[i];
	}
	return NULL;
}
#endif

static int locked_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct ddb_input *input = fe->sec_priv;
	struct ddb_port *port = input->port;
	struct ddb_dvb *dvb = &port->dvb[input->nr & 1];
	int status;

	if (enable) {
		mutex_lock(&port->i2c_gate_lock);
		status = dvb->gate_ctrl(fe, 1);
	} else {
		status = dvb->gate_ctrl(fe, 0);
		mutex_unlock(&port->i2c_gate_lock);
	}
	return status;
}

#ifdef CONFIG_DVB_DRXK
static int demod_attach_drxk(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb->fe = dvb_attach(drxk_attach,
				  i2c, 0x29 + (input->nr&1),
				  &dvb->fe2);
	if (!fe) {
		pr_err("No DRXK found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	dvb->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}
#endif

#if 0
struct stv0367_config stv0367_0 = {
	.demod_address = 0x1f,
	.xtal = 27000000,
	.if_khz = 5000,
	.if_iq_mode = FE_TER_NORMAL_IF_TUNER,
	.ts_mode = STV0367_SERIAL_PUNCT_CLOCK,
	.clk_pol = STV0367_RISINGEDGE_CLOCK,
};

struct stv0367_config stv0367_1 = {
	.demod_address = 0x1e,
	.xtal = 27000000,
	.if_khz = 5000,
	.if_iq_mode = FE_TER_NORMAL_IF_TUNER,
	.ts_mode = STV0367_SERIAL_PUNCT_CLOCK,
	.clk_pol = STV0367_RISINGEDGE_CLOCK,
};


static int demod_attach_stv0367(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb->fe = dvb_attach(stv0367ter_attach,
				  (input->nr & 1) ? &stv0367_1 : &stv0367_0,
				  i2c);
	if (!dvb->fe) {
		pr_err("No stv0367 found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	dvb->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}
#endif

struct cxd2843_cfg cxd2843_0 = {
	.adr = 0x6c,
};

struct cxd2843_cfg cxd2843_1 = {
	.adr = 0x6d,
};

struct cxd2843_cfg cxd2843p_0 = {
	.adr = 0x6c,
	.parallel = 1,
};

struct cxd2843_cfg cxd2843p_1 = {
	.adr = 0x6d,
	.parallel = 1,
};

static int demod_attach_cxd2843(struct ddb_input *input, int par)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	if (par)
		fe = dvb->fe = dvb_attach(cxd2843_attach, i2c,
					  (input->nr & 1) ? &cxd2843p_1 : &cxd2843p_0);
	else
		fe = dvb->fe = dvb_attach(cxd2843_attach, i2c,
					  (input->nr & 1) ? &cxd2843_1 : &cxd2843_0);
	if (!dvb->fe) {
		pr_err("No cxd2837/38/43 found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	dvb->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

struct stv0367_cfg stv0367dd_0 = {
	.adr = 0x1f,
	.xtal = 27000000,
};

struct stv0367_cfg stv0367dd_1 = {
	.adr = 0x1e,
	.xtal = 27000000,
};

static int demod_attach_stv0367dd(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb->fe = dvb_attach(stv0367_attach, i2c,
				  (input->nr & 1) ?
				  &stv0367dd_1 : &stv0367dd_0,
				  &dvb->fe2);
	if (!dvb->fe) {
		pr_err("No stv0367 found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	dvb->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

static int tuner_attach_tda18271(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	if (dvb->fe->ops.i2c_gate_ctrl)
		dvb->fe->ops.i2c_gate_ctrl(dvb->fe, 1);
	fe = dvb_attach(tda18271c2dd_attach, dvb->fe, i2c, 0x60);
	if (dvb->fe->ops.i2c_gate_ctrl)
		dvb->fe->ops.i2c_gate_ctrl(dvb->fe, 0);
	if (!fe) {
		pr_err("No TDA18271 found!\n");
		return -ENODEV;
	}
	return 0;
}

static int tuner_attach_tda18212dd(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;

	fe = dvb_attach(tda18212dd_attach, dvb->fe, i2c,
			(input->nr & 1) ? 0x63 : 0x60);
	if (!fe) {
		pr_err("No TDA18212 found!\n");
		return -ENODEV;
	}
	return 0;
}

#ifdef CONFIG_DVB_TDA18212
struct tda18212_config tda18212_0 = {
	.i2c_address = 0x60,
};

struct tda18212_config tda18212_1 = {
	.i2c_address = 0x63,
};

static int tuner_attach_tda18212(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_frontend *fe;
	struct tda18212_config *cfg;

	cfg = (input->nr & 1) ? &tda18212_1 : &tda18212_0;
	fe = dvb_attach(tda18212_attach, dvb->fe, i2c, cfg);
	if (!fe) {
		pr_err("No TDA18212 found!\n");
		return -ENODEV;
	}
	return 0;
}
#endif

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static struct stv090x_config stv0900 = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x69,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.ts1_tei        = 1,
	.ts2_tei        = 1,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,
};

static struct stv090x_config stv0900_aa = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x68,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.ts1_tei        = 1,
	.ts2_tei        = 1,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,
};

static struct stv6110x_config stv6110a = {
	.addr    = 0x60,
	.refclk	 = 27000000,
	.clk_div = 1,
};

static struct stv6110x_config stv6110b = {
	.addr    = 0x63,
	.refclk	 = 27000000,
	.clk_div = 1,
};

static int demod_attach_stv0900(struct ddb_input *input, int type)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct stv090x_config *feconf = type ? &stv0900_aa : &stv0900;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	dvb->fe = dvb_attach(stv090x_attach, feconf, i2c,
			     (input->nr & 1) ? STV090x_DEMODULATOR_1
			     : STV090x_DEMODULATOR_0);
	if (!dvb->fe) {
		pr_err("No STV0900 found!\n");
		return -ENODEV;
	}
	if (!dvb_attach(lnbh24_attach, dvb->fe, i2c, 0,
			0, (input->nr & 1) ?
			(0x09 - type) : (0x0b - type))) {
		pr_err("No LNBH24 found!\n");
		return -ENODEV;
	}
	return 0;
}

static int tuner_attach_stv6110(struct ddb_input *input, int type)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct stv090x_config *feconf = type ? &stv0900_aa : &stv0900;
	struct stv6110x_config *tunerconf = (input->nr & 1) ?
		&stv6110b : &stv6110a;
	struct stv6110x_devctl *ctl;

	ctl = dvb_attach(stv6110x_attach, dvb->fe, tunerconf, i2c);
	if (!ctl) {
		pr_err("No STV6110X found!\n");
		return -ENODEV;
	}
	pr_info("attach tuner input %d adr %02x\n",
		input->nr, tunerconf->addr);

	feconf->tuner_init          = ctl->tuner_init;
	feconf->tuner_sleep         = ctl->tuner_sleep;
	feconf->tuner_set_mode      = ctl->tuner_set_mode;
	feconf->tuner_set_frequency = ctl->tuner_set_frequency;
	feconf->tuner_get_frequency = ctl->tuner_get_frequency;
	feconf->tuner_set_bandwidth = ctl->tuner_set_bandwidth;
	feconf->tuner_get_bandwidth = ctl->tuner_get_bandwidth;
	feconf->tuner_set_bbgain    = ctl->tuner_set_bbgain;
	feconf->tuner_get_bbgain    = ctl->tuner_get_bbgain;
	feconf->tuner_set_refclk    = ctl->tuner_set_refclk;
	feconf->tuner_get_status    = ctl->tuner_get_status;

	return 0;
}

static int my_dvb_dmx_ts_card_init(struct dvb_demux *dvbdemux, char *id,
				   int (*start_feed)(struct dvb_demux_feed *),
				   int (*stop_feed)(struct dvb_demux_feed *),
				   void *priv)
{
	dvbdemux->priv = priv;

	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	dvbdemux->write_to_decoder = NULL;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING);
	return dvb_dmx_init(dvbdemux);
}

static int my_dvb_dmxdev_ts_card_init(struct dmxdev *dmxdev,
				      struct dvb_demux *dvbdemux,
				      struct dmx_frontend *hw_frontend,
				      struct dmx_frontend *mem_frontend,
				      struct dvb_adapter *dvb_adapter)
{
	int ret;

	dmxdev->filternum = 256;
	dmxdev->demux = &dvbdemux->dmx;
	dmxdev->capabilities = 0;
	ret = dvb_dmxdev_init(dmxdev, dvb_adapter);
	if (ret < 0)
		return ret;

	hw_frontend->source = DMX_FRONTEND_0;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, hw_frontend);
	mem_frontend->source = DMX_MEMORY_FE;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, mem_frontend);
	return dvbdemux->dmx.connect_frontend(&dvbdemux->dmx, hw_frontend);
}

static int start_input(struct ddb_input *input)
{
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (!dvb->users)
		ddb_input_start_all(input);

	return ++dvb->users;
}

static int stop_input(struct ddb_input *input)
{
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (--dvb->users)
		return dvb->users;

	ddb_input_stop_all(input);
	return 0;
}

static int start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ddb_input *input = dvbdmx->priv;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (!dvb->users)
		ddb_input_start_all(input);

	return ++dvb->users;
}

static int stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ddb_input *input = dvbdmx->priv;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];

	if (--dvb->users)
		return dvb->users;

	ddb_input_stop_all(input);
	return 0;
}

static void dvb_input_detach(struct ddb_input *input)
{
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct dvb_demux *dvbdemux = &dvb->demux;

	switch (dvb->attached) {
	case 0x31:
		if (dvb->fe2)
			dvb_unregister_frontend(dvb->fe2);
		if (dvb->fe)
			dvb_unregister_frontend(dvb->fe);
	case 0x30:
		dvb_frontend_detach(dvb->fe);
		dvb->fe = dvb->fe2 = NULL;
#ifdef DVB_NETSTREAM
	case 0x21:
		dvb_netstream_release(&dvb->dvbns);
#endif
	case 0x20:
		dvb_net_release(&dvb->dvbnet);
	case 0x11:
		dvbdemux->dmx.close(&dvbdemux->dmx);
		dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
					      &dvb->hw_frontend);
		dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
					      &dvb->mem_frontend);
		dvb_dmxdev_release(&dvb->dmxdev);
	case 0x10:
		dvb_dmx_release(&dvb->demux);
	case 0x01:
		break;
	}
	dvb->attached = 0x00;
}

static int dvb_register_adapters(struct ddb *dev)
{
	int i, ret = 0;
	struct ddb_port *port;
	struct dvb_adapter *adap;

	if (adapter_alloc == 3 || dev->info->type == DDB_MOD) {
		port = &dev->port[0];
		adap = port->dvb[0].adap;
		ret = dvb_register_adapter(adap, "DDBridge", THIS_MODULE,
					   port->dev->dev,
					   adapter_nr);
		if (ret < 0)
			return ret;
		port->dvb[0].adap_registered = 1;
		for (i = 0; i < dev->info->port_num; i++) {
			port = &dev->port[i];
			port->dvb[0].adap = adap;
			port->dvb[1].adap = adap;
		}
		return 0;
	}

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			adap = port->dvb[0].adap;
			ret = dvb_register_adapter(adap, "DDBridge",
						   THIS_MODULE,
						   port->dev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[0].adap_registered = 1;

			if (adapter_alloc > 0) {
				port->dvb[1].adap = port->dvb[0].adap;
				break;
			}
			adap = port->dvb[1].adap;
			ret = dvb_register_adapter(adap, "DDBridge",
						   THIS_MODULE,
						   port->dev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[1].adap_registered = 1;
			break;

		case DDB_PORT_CI:
		case DDB_PORT_LOOP:
			adap = port->dvb[0].adap;
			ret = dvb_register_adapter(adap, "DDBridge",
						   THIS_MODULE,
						   port->dev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[0].adap_registered = 1;
			break;
		default:
			if (adapter_alloc < 2)
				break;
			adap = port->dvb[0].adap;
			ret = dvb_register_adapter(adap, "DDBridge",
						   THIS_MODULE,
						   port->dev->dev,
						   adapter_nr);
			if (ret < 0)
				return ret;
			port->dvb[0].adap_registered = 1;
			break;
		}
	}
	return ret;
}

static void dvb_unregister_adapters(struct ddb *dev)
{
	int i;
	struct ddb_port *port;
	struct ddb_dvb *dvb;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];

		dvb = &port->dvb[0];
		if (dvb->adap_registered)
			dvb_unregister_adapter(dvb->adap);
		dvb->adap_registered = 0;

		dvb = &port->dvb[1];
		if (dvb->adap_registered)
			dvb_unregister_adapter(dvb->adap);
		dvb->adap_registered = 0;
	}
}

static int dvb_input_attach(struct ddb_input *input)
{
	int ret = 0;
	struct ddb_dvb *dvb = &input->port->dvb[input->nr & 1];
	struct ddb_port *port = input->port;
	struct dvb_adapter *adap = dvb->adap;
	struct dvb_demux *dvbdemux = &dvb->demux;

	dvb->attached = 0x01;

	ret = my_dvb_dmx_ts_card_init(dvbdemux, "SW demux",
				      start_feed,
				      stop_feed, input);
	if (ret < 0)
		return ret;
	dvb->attached = 0x10;

	ret = my_dvb_dmxdev_ts_card_init(&dvb->dmxdev,
					 &dvb->demux,
					 &dvb->hw_frontend,
					 &dvb->mem_frontend, adap);
	if (ret < 0)
		return ret;
	dvb->attached = 0x11;

	ret = dvb_net_init(adap, &dvb->dvbnet, dvb->dmxdev.demux);
	if (ret < 0)
		return ret;
	dvb->attached = 0x20;

#ifdef DVB_NETSTREAM
	ret = netstream_init(input);
	if (ret < 0)
		return ret;
	dvb->attached = 0x21;
#endif
	dvb->fe = dvb->fe2 = 0;
	switch (port->type) {
	case DDB_TUNER_DVBS_ST:
		if (demod_attach_stv0900(input, 0) < 0)
			return -ENODEV;
		if (tuner_attach_stv6110(input, 0) < 0)
			return -ENODEV;
		break;
	case DDB_TUNER_DVBS_ST_AA:
		if (demod_attach_stv0900(input, 1) < 0)
			return -ENODEV;
		if (tuner_attach_stv6110(input, 1) < 0)
			return -ENODEV;
		break;
#ifdef CONFIG_DVB_DRXK
	case DDB_TUNER_DVBCT_TR:
		if (demod_attach_drxk(input) < 0)
			return -ENODEV;
		if (tuner_attach_tda18271(input) < 0)
			return -ENODEV;
		break;
#endif
	case DDB_TUNER_DVBCT_ST:
		if (demod_attach_stv0367dd(input) < 0)
			return -ENODEV;
		if (tuner_attach_tda18212dd(input) < 0)
			return -ENODEV;
		break;
	case DDB_TUNER_DVBCT2_SONY:
	case DDB_TUNER_DVBC2T2_SONY:
	case DDB_TUNER_ISDBT_SONY:
		if (demod_attach_cxd2843(input, 0) < 0)
			return -ENODEV;
		if (tuner_attach_tda18212dd(input) < 0)
			return -ENODEV;
		break;
	case DDB_TUNER_DVBCT2_SONY_P:
	case DDB_TUNER_DVBC2T2_SONY_P:
		if (demod_attach_cxd2843(input, 1) < 0)
			return -ENODEV;
		if (tuner_attach_tda18212dd(input) < 0)
			return -ENODEV;
		break;
	default:
		return 0;
	}
	dvb->attached = 0x30;
	if (dvb->fe) {
		if (dvb_register_frontend(adap, dvb->fe) < 0)
			return -ENODEV;
	}
	if (dvb->fe2) {
		if (dvb_register_frontend(adap, dvb->fe2) < 0)
			return -ENODEV;
		dvb->fe2->tuner_priv = dvb->fe->tuner_priv;
		memcpy(&dvb->fe2->ops.tuner_ops,
		       &dvb->fe->ops.tuner_ops,
		       sizeof(struct dvb_tuner_ops));
	}
	dvb->attached = 0x31;
	return 0;
}


static int port_has_encti(struct ddb_port *port)
{
	u8 val;
	int ret = i2c_read_reg(&port->i2c->adap, 0x20, 0, &val);

	if (!ret)
		pr_info("[0x20]=0x%02x\n", val);

	return ret ? 0 : 1;
}

static int port_has_cxd(struct ddb_port *port, u8 *type)
{
	u8 val;
	u8 probe[4] = { 0xe0, 0x00, 0x00, 0x00 }, data[4];
	struct i2c_msg msgs[2] = {{ .addr = 0x40,  .flags = 0,
				    .buf  = probe, .len   = 4 },
				  { .addr = 0x40,  .flags = I2C_M_RD,
				    .buf  = data,  .len   = 4 } };
	val = i2c_transfer(&port->i2c->adap, msgs, 2);
	if (val != 2)
		return 0;

	if (data[0] == 0x02 && data[1] == 0x2b && data[3] == 0x43)
		*type = 2;
	else
		*type = 1;
	return 1;
}

static int port_has_xo2(struct ddb_port *port, u8 *id)
{
	u8 val;
	u8 probe[1] = { 0x00 }, data[4];
	struct i2c_msg msgs[2] = {{ .addr = 0x10,  .flags = 0,
				    .buf  = probe, .len   = 1 },
				  { .addr = 0x10,  .flags = I2C_M_RD,
				    .buf  = data,  .len   = 4 } };
	val = i2c_transfer(&port->i2c->adap, msgs, 2);
	if (val != 2)
		return 0;

	if (data[0] != 'D' || data[1] != 'F')
		return 0;

	*id = data[2];
	return 1;
}

static int port_has_stv0900(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x69, 0xf100, &val) < 0)
		return 0;
	return 1;
}

static int port_has_stv0900_aa(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x68, 0xf100, &val) < 0)
		return 0;
	return 1;
}

static int port_has_drxks(struct ddb_port *port)
{
	u8 val;
	if (i2c_read(&port->i2c->adap, 0x29, &val) < 0)
		return 0;
	if (i2c_read(&port->i2c->adap, 0x2a, &val) < 0)
		return 0;
	return 1;
}

static int port_has_stv0367(struct ddb_port *port)
{
	u8 val;

	if (i2c_read_reg16(&port->i2c->adap, 0x1e, 0xf000, &val) < 0)
		return 0;
	if (val != 0x60)
		return 0;
	if (i2c_read_reg16(&port->i2c->adap, 0x1f, 0xf000, &val) < 0)
		return 0;
	if (val != 0x60)
		return 0;
	return 1;
}

static int init_xo2_old(struct ddb_port *port)
{
	struct i2c_adapter *i2c = &port->i2c->adap;
	u8 val;
	int res;

	res = i2c_read_reg(i2c, 0x10, 0x04, &val);
	if (res < 0)
		return res;

	if (val != 0x02)  {
		pr_info("Port %d: invalid XO2\n", port->nr);
		return -1;
	}
	i2c_write_reg(i2c, 0x10, 0xc0, 0x00); /* Disable XO2 I2C master */

	i2c_read_reg(i2c, 0x10, 0x08, &val);
	if (val != 0) {
		i2c_write_reg(i2c, 0x10, 0x08, 0x00);
		msleep(100);
	}
	/* Enable tuner power, disable pll, reset demods */
	i2c_write_reg(i2c, 0x10, 0x08, 0x04);
	usleep_range(2000, 3000);
	/* Release demod resets */
	i2c_write_reg(i2c, 0x10, 0x08, 0x07);
	usleep_range(2000, 3000);
	/* Start XO2 PLL */
	i2c_write_reg(i2c, 0x10, 0x08, 0x87);

	return 0;
}

static int init_xo2(struct ddb_port *port)
{
	struct i2c_adapter *i2c = &port->i2c->adap;
	u8 val, data[2];
	int res;

	res = i2c_read_regs(i2c, 0x10, 0x04, data, 2);
	if (res < 0)
		return res;

	if (data[0] != 0x01)  {
		pr_info("Port %d: invalid XO2\n", port->nr);
		return -1;
	}

	i2c_read_reg(i2c, 0x10, 0x08, &val);
	if (val != 0) {
		i2c_write_reg(i2c, 0x10, 0x08, 0x00);
		msleep(100);
	}
	/* Enable tuner power, disable pll, reset demods */
	i2c_write_reg(i2c, 0x10, 0x08, 0x04);
	usleep_range(2000, 3000);
	/* Release demod resets */
	i2c_write_reg(i2c, 0x10, 0x08, 0x07);
	usleep_range(2000, 3000);
	/* Start XO2 PLL */
	i2c_write_reg(i2c, 0x10, 0x08, 0x87);

	return 0;
}

static int port_has_cxd28xx(struct ddb_port *port, u8 *id)
{
	struct i2c_adapter *i2c = &port->i2c->adap;
	int status;

	status = i2c_write_reg(&port->i2c->adap, 0x6c, 0, 0);
	if (status)
		return 0;
	status = i2c_read_reg(i2c, 0x6c, 0xfd, id);
	if (status)
		return 0;
	return 1;
}

static void ddb_port_probe(struct ddb_port *port)
{
	struct ddb *dev = port->dev;
	u8 id;

	port->name = "NO MODULE";
	port->class = DDB_PORT_NONE;

	if (dev->info->type == DDB_MOD) {
		port->name = "MOD";
		port->class = DDB_PORT_MOD;
		return;
	}

	if (port->nr > 1 && dev->info->type == DDB_OCTOPUS_CI) {
		port->name = "CI internal";
		port->class = DDB_PORT_CI;
		port->type = DDB_CI_INTERNAL;
	} else if (port_has_cxd(port, &id)) {
		if (id == 1) {
			port->name = "CI";
			port->class = DDB_PORT_CI;
			port->type = DDB_CI_EXTERNAL_SONY;
			ddbwritel(dev, I2C_SPEED_400,
				  port->i2c->regs + I2C_TIMING);
		} else {
			pr_info(KERN_INFO "Port %d: Uninitialized DuoFlex\n",
			       port->nr);
			return;
		}
	} else if (port_has_xo2(port, &id)) {
		char *xo2names[] = { "DUAL DVB-S2", "DUAL DVB-C/T/T2",
				     "DUAL DVB-ISDBT", "DUAL DVB-C/C2/T/T2",
				     "DUAL ATSC", "DUAL DVB-C/C2/T/T2" };

		ddbwritel(dev, I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
		id >>= 2;
		if (id > 5) {
			port->name = "unknown XO2 DuoFlex";
		} else {
			port->class = DDB_PORT_TUNER;
			port->type = DDB_TUNER_XO2 + id;
			port->name = xo2names[id];
			init_xo2(port);
		}
	} else if (port_has_cxd28xx(port, &id)) {
		switch (id) {
		case 0xa4:
			port->name = "DUAL DVB-CT2 CXD2843";
			port->type = DDB_TUNER_DVBC2T2_SONY_P;
			break;
		case 0xb1:
			port->name = "DUAL DVB-CT2 CXD2837";
			port->type = DDB_TUNER_DVBCT2_SONY_P;
			break;
		case 0xb0:
			port->name = "DUAL ISDB-T CXD2838";
			/* there is no non-xo2 version of this */
			break;
		default:
			return;
		}
		port->class = DDB_PORT_TUNER;
		ddbwritel(dev, I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0900(port)) {
		port->name = "DUAL DVB-S2";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBS_ST;
		ddbwritel(dev, I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0900_aa(port)) {
		port->name = "DUAL DVB-S2";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBS_ST_AA;
		ddbwritel(dev, I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port_has_drxks(port)) {
		port->name = "DUAL DVB-C/T";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBCT_TR;
		ddbwritel(dev, I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0367(port)) {
		port->name = "DUAL DVB-C/T";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBCT_ST;
		ddbwritel(dev, I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port_has_encti(port)) {
		port->name = "ENCTI";
		port->class = DDB_PORT_LOOP;
	} else if (port->nr == ts_loop) {
		port->name = "TS LOOP";
		port->class = DDB_PORT_LOOP;
	}
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static int wait_ci_ready(struct ddb_ci *ci)
{
	u32 count = 10;

	ndelay(500);
	do {
		if (ddbreadl(ci->port->dev,
			     CI_CONTROL(ci->nr)) & CI_READY)
			break;
		usleep_range(1, 2);
		if ((--count) == 0)
			return -1;
	} while (1);
	return 0;
}

static int read_attribute_mem(struct dvb_ca_en50221 *ca,
			      int slot, int address)
{
	struct ddb_ci *ci = ca->data;
	u32 val, off = (address >> 1) & (CI_BUFFER_SIZE-1);

	if (address > CI_BUFFER_SIZE)
		return -1;
	ddbwritel(ci->port->dev, CI_READ_CMD | (1 << 16) | address,
		  CI_DO_READ_ATTRIBUTES(ci->nr));
	wait_ci_ready(ci);
	val = 0xff & ddbreadl(ci->port->dev, CI_BUFFER(ci->nr) + off);
	return val;
}

static int write_attribute_mem(struct dvb_ca_en50221 *ca, int slot,
			       int address, u8 value)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, CI_WRITE_CMD | (value << 16) | address,
		  CI_DO_ATTRIBUTE_RW(ci->nr));
	wait_ci_ready(ci);
	return 0;
}

static int read_cam_control(struct dvb_ca_en50221 *ca,
			    int slot, u8 address)
{
	u32 count = 100;
	struct ddb_ci *ci = ca->data;
	u32 res;

	ddbwritel(ci->port->dev, CI_READ_CMD | address,
		  CI_DO_IO_RW(ci->nr));
	ndelay(500);
	do {
		res = ddbreadl(ci->port->dev, CI_READDATA(ci->nr));
		if (res & CI_READY)
			break;
		usleep_range(1, 2);
		if ((--count) == 0)
			return -1;
	} while (1);
	return 0xff & res;
}

static int write_cam_control(struct dvb_ca_en50221 *ca, int slot,
			     u8 address, u8 value)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, CI_WRITE_CMD | (value << 16) | address,
		  CI_DO_IO_RW(ci->nr));
	wait_ci_ready(ci);
	return 0;
}

static int slot_reset(struct dvb_ca_en50221 *ca, int slot)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, CI_POWER_ON,
		  CI_CONTROL(ci->nr));
	msleep(100);
	ddbwritel(ci->port->dev, CI_POWER_ON | CI_RESET_CAM,
		  CI_CONTROL(ci->nr));
	ddbwritel(ci->port->dev, CI_ENABLE | CI_POWER_ON | CI_RESET_CAM,
		  CI_CONTROL(ci->nr));
	udelay(20);
	ddbwritel(ci->port->dev, CI_ENABLE | CI_POWER_ON,
		  CI_CONTROL(ci->nr));
	return 0;
}

static int slot_shutdown(struct dvb_ca_en50221 *ca, int slot)
{
	struct ddb_ci *ci = ca->data;

	ddbwritel(ci->port->dev, 0, CI_CONTROL(ci->nr));
	msleep(300);
	return 0;
}

static int slot_ts_enable(struct dvb_ca_en50221 *ca, int slot)
{
	struct ddb_ci *ci = ca->data;
	u32 val = ddbreadl(ci->port->dev, CI_CONTROL(ci->nr));

	ddbwritel(ci->port->dev, val | CI_BYPASS_DISABLE,
		  CI_CONTROL(ci->nr));
	return 0;
}

static int poll_slot_status(struct dvb_ca_en50221 *ca, int slot, int open)
{
	struct ddb_ci *ci = ca->data;
	u32 val = ddbreadl(ci->port->dev, CI_CONTROL(ci->nr));
	int stat = 0;

	if (val & CI_CAM_DETECT)
		stat |= DVB_CA_EN50221_POLL_CAM_PRESENT;
	if (val & CI_CAM_READY)
		stat |= DVB_CA_EN50221_POLL_CAM_READY;
	return stat;
}

static struct dvb_ca_en50221 en_templ = {
	.read_attribute_mem  = read_attribute_mem,
	.write_attribute_mem = write_attribute_mem,
	.read_cam_control    = read_cam_control,
	.write_cam_control   = write_cam_control,
	.slot_reset          = slot_reset,
	.slot_shutdown       = slot_shutdown,
	.slot_ts_enable      = slot_ts_enable,
	.poll_slot_status    = poll_slot_status,
};

static void ci_attach(struct ddb_port *port)
{
	struct ddb_ci *ci = 0;

	ci = kzalloc(sizeof(*ci), GFP_KERNEL);
	if (!ci)
		return;
	memcpy(&ci->en, &en_templ, sizeof(en_templ));
	ci->en.data = ci;
	port->en = &ci->en;
	ci->port = port;
	ci->nr = port->nr - 2;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/


struct cxd2099_cfg cxd_cfg = {
	.bitrate =  72000,
	.adr     =  0x40,
	.polarity = 1,
	.clock_mode = 1,
};

static int ddb_ci_attach(struct ddb_port *port)
{
	if (port->type == DDB_CI_EXTERNAL_SONY) {
		cxd_cfg.bitrate = ci_bitrate;
		port->en = cxd2099_attach(&cxd_cfg, port, &port->i2c->adap);
		if (!port->en)
			return -ENODEV;
		dvb_ca_en50221_init(port->dvb[0].adap,
				    port->en, 0, 1);
	}
	if (port->type == DDB_CI_INTERNAL) {
		ci_attach(port);
		if (!port->en)
			return -ENODEV;
		dvb_ca_en50221_init(port->dvb[0].adap, port->en, 0, 1);
	}
	return 0;
}

static int ddb_port_attach(struct ddb_port *port)
{
	int ret = 0;

	switch (port->class) {
	case DDB_PORT_TUNER:
		ret = dvb_input_attach(port->input[0]);
		if (ret < 0)
			break;
		ret = dvb_input_attach(port->input[1]);
		if (ret < 0)
			break;
		port->input[0]->redi = port->input[0];
		port->input[1]->redi = port->input[1];
		break;
	case DDB_PORT_CI:
		ret = ddb_ci_attach(port);
		if (ret < 0)
			break;
	case DDB_PORT_LOOP:
		ret = dvb_register_device(port->dvb[0].adap,
					  &port->dvb[0].dev,
					  &dvbdev_ci, (void *) port->output,
					  DVB_DEVICE_CI);
		break;
	case DDB_PORT_MOD:
		ret = dvb_register_device(port->dvb[0].adap,
					  &port->dvb[0].dev,
					  &dvbdev_mod, (void *) port->output,
					  DVB_DEVICE_MOD);
		break;
	default:
		break;
	}
	if (ret < 0)
		pr_err("port_attach on port %d failed\n", port->nr);
	return ret;
}

static int ddb_ports_attach(struct ddb *dev)
{
	int i, ret = 0;
	struct ddb_port *port;

	if (dev->info->port_num) {
		ret = dvb_register_adapters(dev);
		if (ret < 0) {
			pr_err("Registering adapters failed. Check DVB_MAX_ADAPTERS in config.\n");
			return ret;
		}
	}
	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		ret = ddb_port_attach(port);
		if (ret < 0)
			break;
	}
	return ret;
}

static void ddb_ports_detach(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];

		switch (port->class) {
		case DDB_PORT_TUNER:
			dvb_input_detach(port->input[0]);
			dvb_input_detach(port->input[1]);
			break;
		case DDB_PORT_CI:
		case DDB_PORT_LOOP:
			if (port->dvb[0].dev)
				dvb_unregister_device(port->dvb[0].dev);
			if (port->en) {
				dvb_ca_en50221_release(port->en);
				kfree(port->en);
				port->en = 0;
			}
			break;
		case DDB_PORT_MOD:
			if (port->dvb[0].dev)
				dvb_unregister_device(port->dvb[0].dev);
			break;
		}
	}
	dvb_unregister_adapters(dev);
}


/* Copy input DMA pointers to output DMA and ACK. */

static void input_write_output(struct ddb_input *input,
			       struct ddb_output *output)
{
	ddbwritel(output->port->dev,
		  input->dma->stat, DMA_BUFFER_ACK(output->dma->nr));
	output->dma->cbuf = (input->dma->stat >> 11) & 0x1f;
	output->dma->coff = (input->dma->stat & 0x7ff) << 7;
}

static void output_ack_input(struct ddb_output *output,
			     struct ddb_input *input)
{
	ddbwritel(input->port->dev,
		  output->dma->stat, DMA_BUFFER_ACK(input->dma->nr));
}

static void input_write_dvb(struct ddb_input *input,
			    struct ddb_input *input2)
{
	struct ddb_dvb *dvb = &input2->port->dvb[input2->nr & 1];
	struct ddb_dma *dma, *dma2;
	struct ddb *dev = input->port->dev;
	int noack = 0;

	dma = dma2 = input->dma;
	/* if there also is an output connected, do not ACK.
	   input_write_output will ACK. */
	if (input->redo) {
		dma2 = input->redo->dma;
		noack = 1;
	}
	while (dma->cbuf != ((dma->stat >> 11) & 0x1f)
	       || (4 & dma->ctrl)) {
		if (4 & dma->ctrl) {
			/*pr_err("Overflow dma %d\n", dma->nr);*/
			if (noack)
				noack = 0;
		}
#ifdef DDB_ALT_DMA
		dma_sync_single_for_cpu(dev->dev, dma2->pbuf[dma->cbuf],
					dma2->size, DMA_FROM_DEVICE);
#endif
		dvb_dmx_swfilter_packets(&dvb->demux,
					 dma2->vbuf[dma->cbuf],
					 dma2->size / 188);
		dma->cbuf = (dma->cbuf + 1) % dma2->num;
		if (!noack)
			ddbwritel(dev, (dma->cbuf << 11),
				  DMA_BUFFER_ACK(dma->nr));
		dma->stat = ddbreadl(dev, DMA_BUFFER_CURRENT(dma->nr));
		dma->ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(dma->nr));
	}
}

#ifdef DDB_USE_WORK
static void input_work(struct work_struct *work)
{
	struct ddb_dma *dma = container_of(work, struct ddb_dma, work);
	struct ddb_input *input = (struct ddb_input *) dma->io;
#else
static void input_tasklet(unsigned long data)
{
	struct ddb_input *input = (struct ddb_input *) data;
	struct ddb_dma *dma = input->dma;
#endif
	struct ddb *dev = input->port->dev;

	spin_lock(&dma->lock);
	if (!dma->running) {
		spin_unlock(&dma->lock);
		return;
	}
	dma->stat = ddbreadl(dev, DMA_BUFFER_CURRENT(dma->nr));
	dma->ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(dma->nr));

	/*pr_err("IT %d.%d %08x\n", dev->nr, dma->nr, dma->ctrl);*/
#if 0
	if (4 & dma->ctrl)
		pr_err("Overflow dma %d\n", dma->nr);
#endif
	if (input->redi)
		input_write_dvb(input, input->redi);
	if (input->redo)
		input_write_output(input, input->redo);
	wake_up(&dma->wq);
	spin_unlock(&dma->lock);
}

static void input_handler(unsigned long data)
{
	struct ddb_input *input = (struct ddb_input *) data;
	struct ddb_dma *dma = input->dma;


	/* If there is no input connected, input_tasklet() will
	   just copy pointers and ACK. So, there is no need to go
	   through the tasklet scheduler. */
#ifdef DDB_USE_WORK
	if (input->redi)
		queue_work(ddb_wq, &dma->work);
	else
		input_work(&dma->work);
#else
	if (input->redi)
		tasklet_schedule(&dma->tasklet);
	else
		input_tasklet(data);
#endif
}

/* hmm, don't really need this anymore.
   The output IRQ just copies some pointers, acks and wakes. */

static void output_work(struct work_struct *work)
{
}

static void output_tasklet(unsigned long data)
{
}

static void output_handler(unsigned long data)
{
	struct ddb_output *output = (struct ddb_output *) data;
	struct ddb_dma *dma = output->dma;
	struct ddb *dev = output->port->dev;

	spin_lock(&dma->lock);
	if (!dma->running) {
		spin_unlock(&dma->lock);
		return;
	}
	dma->stat = ddbreadl(dev, DMA_BUFFER_CURRENT(dma->nr));
	dma->ctrl = ddbreadl(dev, DMA_BUFFER_CONTROL(dma->nr));
	if (output->redi)
		output_ack_input(output, output->redi);
	wake_up(&dma->wq);
	spin_unlock(&dma->lock);
}


/****************************************************************************/
/****************************************************************************/


static void ddb_dma_init(struct ddb_dma *dma, int nr, void *io, int out)
{
#ifndef DDB_USE_WORK
	unsigned long priv = (unsigned long) io;
#endif

	dma->io = io;
	dma->nr = nr;
	spin_lock_init(&dma->lock);
	init_waitqueue_head(&dma->wq);
	if (out) {
#ifdef DDB_USE_WORK
		INIT_WORK(&dma->work, output_work);
#else
		tasklet_init(&dma->tasklet, output_tasklet, priv);
#endif
		dma->num = OUTPUT_DMA_BUFS;
		dma->size = OUTPUT_DMA_SIZE;
		dma->div = OUTPUT_DMA_IRQ_DIV;
	} else {
#ifdef DDB_USE_WORK
		INIT_WORK(&dma->work, input_work);
#else
		tasklet_init(&dma->tasklet, input_tasklet, priv);
#endif
		dma->num = INPUT_DMA_BUFS;
		dma->size = INPUT_DMA_SIZE;
		dma->div = INPUT_DMA_IRQ_DIV;
	}
}

static void ddb_input_init(struct ddb_port *port, int nr, int pnr, int dma_nr)
{
	struct ddb *dev = port->dev;
	struct ddb_input *input = &dev->input[nr];

	if (dev->has_dma) {
		dev->handler[dma_nr + 8] = input_handler;
		dev->handler_data[dma_nr + 8] = (unsigned long) input;
	}
	port->input[pnr] = input;
	input->nr = nr;
	input->port = port;
	if (dev->has_dma) {
		input->dma = &dev->dma[dma_nr];
		ddb_dma_init(input->dma, dma_nr, (void *) input, 0);
	}
	ddbwritel(dev, 0, TS_INPUT_CONTROL(nr));
	ddbwritel(dev, 2, TS_INPUT_CONTROL(nr));
	ddbwritel(dev, 0, TS_INPUT_CONTROL(nr));
	if (dev->has_dma)
		ddbwritel(dev, 0, DMA_BUFFER_ACK(input->dma->nr));
}

static void ddb_output_init(struct ddb_port *port, int nr, int dma_nr)
{
	struct ddb *dev = port->dev;
	struct ddb_output *output = &dev->output[nr];

	if (dev->has_dma) {
		dev->handler[dma_nr + 8] = output_handler;
		dev->handler_data[dma_nr + 8] = (unsigned long) output;
	}
	port->output = output;
	output->nr = nr;
	output->port = port;
	if (dev->has_dma) {
		output->dma = &dev->dma[dma_nr];
		ddb_dma_init(output->dma, dma_nr, (void *) output, 1);
	}
	if (output->port->class == DDB_PORT_MOD) {
		/*ddbwritel(dev, 0, CHANNEL_CONTROL(output->nr));*/
	} else {
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(nr));
		ddbwritel(dev, 2, TS_OUTPUT_CONTROL(nr));
		ddbwritel(dev, 0, TS_OUTPUT_CONTROL(nr));
	}
	if (dev->has_dma)
		ddbwritel(dev, 0, DMA_BUFFER_ACK(output->dma->nr));
}

static void ddb_ports_init(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	if (dev->info->board_control) {
		ddbwritel(dev, 0, BOARD_CONTROL);
		msleep(100);
		ddbwritel(dev, 4, BOARD_CONTROL);
		usleep_range(2000, 3000);
		ddbwritel(dev, 4 | dev->info->board_control, BOARD_CONTROL);
		usleep_range(2000, 3000);
	}

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		port->dev = dev;
		port->nr = i;
		port->i2c = &dev->i2c[i];
		port->gap = 4;
		port->obr = ci_bitrate;
		mutex_init(&port->i2c_gate_lock);
		ddb_port_probe(port);
		pr_info("Port %d (TAB %d): %s\n",
			port->nr, port->nr + 1, port->name);

		port->dvb[0].adap = &dev->adap[2 * i];
		port->dvb[1].adap = &dev->adap[2 * i + 1];

		if ((dev->info->type == DDB_OCTOPUS_CI) ||
		    (dev->info->type == DDB_OCTONET) ||
		    (dev->info->type == DDB_OCTOPUS)) {
			if (i >= 2 && dev->info->type == DDB_OCTOPUS_CI) {
				ddb_input_init(port, 2 + i, 0, 2 + i);
				ddb_input_init(port, 4 + i, 1, 4 + i);
			} else {
				ddb_input_init(port, 2 * i, 0, 2 * i);
				ddb_input_init(port, 2 * i + 1, 1, 2 * i + 1);
			}
			ddb_output_init(port, i, i + 8);
		}
		if (dev->info->type == DDB_MOD) {
			ddb_output_init(port, i, i);
			dev->handler[i + 18] = ddbridge_mod_rate_handler;
			dev->handler_data[i + 18] =
				(unsigned long) &dev->output[i];
		}
	}
}

static void ddb_ports_release(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
#ifdef DDB_USE_WORK
		if (port->input[0])
			cancel_work_sync(&port->input[0]->dma->work);
		if (port->input[1])
			cancel_work_sync(&port->input[1]->dma->work);
		if (port->output)
			cancel_work_sync(&port->output->dma->work);
#else
		if (port->input[0])
			tasklet_kill(&port->input[0]->dma->tasklet);
		if (port->input[1])
			tasklet_kill(&port->input[1]->dma->tasklet);
		if (port->output)
			tasklet_kill(&port->output->dma->tasklet);
#endif
	}
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define IRQ_HANDLE(_nr) \
	do { if ((s & (1UL << _nr)) && dev->handler[_nr]) \
		dev->handler[_nr](dev->handler_data[_nr]); } \
	while (0)

static void irq_handle_msg(struct ddb *dev, u32 s)
{
	dev->i2c_irq++;
	IRQ_HANDLE(0);
	IRQ_HANDLE(1);
	IRQ_HANDLE(2);
	IRQ_HANDLE(3);
}

static void irq_handle_io(struct ddb *dev, u32 s)
{
	dev->ts_irq++;
	IRQ_HANDLE(8);
	IRQ_HANDLE(9);
	IRQ_HANDLE(10);
	IRQ_HANDLE(11);
	IRQ_HANDLE(12);
	IRQ_HANDLE(13);
	IRQ_HANDLE(14);
	IRQ_HANDLE(15);
	IRQ_HANDLE(16);
	IRQ_HANDLE(17);
	IRQ_HANDLE(18);
	IRQ_HANDLE(19);
	if (dev->info->type != DDB_MOD)
		return;
	IRQ_HANDLE(20);
	IRQ_HANDLE(21);
	IRQ_HANDLE(22);
	IRQ_HANDLE(23);
	IRQ_HANDLE(24);
	IRQ_HANDLE(25);
	IRQ_HANDLE(26);
	IRQ_HANDLE(27);
}

static irqreturn_t irq_handler0(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);

	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		if (!(s & 0xfff00))
			return IRQ_NONE;
		ddbwritel(dev, s, INTERRUPT_ACK);
		irq_handle_io(dev, s);
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));

	return IRQ_HANDLED;
}

static irqreturn_t irq_handler1(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);

	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		if (!(s & 0x0000f))
			return IRQ_NONE;
		ddbwritel(dev, s, INTERRUPT_ACK);
		irq_handle_msg(dev, s);
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));

	return IRQ_HANDLED;
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);
	int ret = IRQ_HANDLED;

	if (!s)
		return IRQ_NONE;
	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		ddbwritel(dev, s, INTERRUPT_ACK);

		if (s & 0x0000000f)
			irq_handle_msg(dev, s);
		if (s & 0x0fffff00) {
			irq_handle_io(dev, s);
#ifdef DDB_TEST_THREADED
			ret = IRQ_WAKE_THREAD;
#endif
		}
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));

	return ret;
}

static irqreturn_t irq_thread(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;

	/*pr_info("%s\n", __func__);*/

	return IRQ_HANDLED;
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static ssize_t nsd_read(struct file *file, char *buf,
			size_t count, loff_t *ppos)
{
	return 0;
}

static unsigned int nsd_poll(struct file *file, poll_table *wait)
{
	return 0;
}

static int nsd_release(struct inode *inode, struct file *file)
{
	return dvb_generic_release(inode, file);
}

static int nsd_open(struct inode *inode, struct file *file)
{
	return dvb_generic_open(inode, file);
}

static int nsd_do_ioctl(struct file *file, unsigned int cmd, void *parg)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb *dev = dvbdev->priv;

	unsigned long arg = (unsigned long) parg;
	int ret = 0;

	switch (cmd) {
	case NSD_START_GET_TS:
	{
		struct dvb_nsd_ts *ts = parg;
		u32 ctrl = ((ts->input & 7) << 8) |
			((ts->filter_mask & 3) << 2);
		u32 to;

		if (ddbreadl(dev, TS_CAPTURE_CONTROL) & 1) {
			pr_info("ts capture busy\n");
			return -EBUSY;
		}
		ddb_dvb_input_start(&dev->input[ts->input & 7]);

		ddbwritel(dev, ctrl, TS_CAPTURE_CONTROL);
		ddbwritel(dev, ts->pid, TS_CAPTURE_PID);
		ddbwritel(dev, (ts->section_id << 16) |
			  (ts->table << 8) | ts->section,
			  TS_CAPTURE_TABLESECTION);
		/* 1024 ms default timeout if timeout set to 0 */
		if (ts->timeout)
			to = ts->timeout;
		else
			to = 1024;
		/* 21 packets default if num set to 0 */
		if (ts->num)
			to |= ((u32) ts->num << 16);
		else
			to |= (21 << 16);
		ddbwritel(dev, to, TS_CAPTURE_TIMEOUT);
		if (ts->mode)
			ctrl |= 2;
		ddbwritel(dev, ctrl | 1, TS_CAPTURE_CONTROL);
		break;
	}
	case NSD_POLL_GET_TS:
	{
		struct dvb_nsd_ts *ts = parg;
		u32 ctrl = ddbreadl(dev, TS_CAPTURE_CONTROL);

		if (ctrl & 1)
			return -EBUSY;
		if (ctrl & (1 << 14)) {
			/*pr_info("ts capture timeout\n");*/
			return -EAGAIN;
		}
		ddbcpyfrom(dev, dev->tsbuf, TS_CAPTURE_MEMORY,
			   TS_CAPTURE_LEN);
		ts->len = ddbreadl(dev, TS_CAPTURE_RECEIVED) & 0x1fff;
		if (copy_to_user(ts->ts, dev->tsbuf, ts->len))
			return -EIO;
		break;
	}
	case NSD_CANCEL_GET_TS:
	{
		u32 ctrl = 0;
		pr_info("cancel ts capture: 0x%x\n", ctrl);
		ddbwritel(dev, ctrl, TS_CAPTURE_CONTROL);
		ctrl = ddbreadl(dev, TS_CAPTURE_CONTROL);
		/*pr_info("control register is 0x%x\n", ctrl);*/
		break;
	}
	case NSD_STOP_GET_TS:
	{
		struct dvb_nsd_ts *ts = parg;
		u32 ctrl = ddbreadl(dev, TS_CAPTURE_CONTROL);

		if (ctrl & 1) {
			pr_info("cannot stop ts capture, while it was neither finished not canceled\n");
			return -EBUSY;
		}
		/*pr_info("ts capture stopped\n");*/
		ddb_dvb_input_stop(&dev->input[ts->input & 7]);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static long nsd_ioctl(struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	return dvb_usercopy(file, cmd, arg, nsd_do_ioctl);
}

static const struct file_operations nsd_fops = {
	.owner   = THIS_MODULE,
	.read    = nsd_read,
	.open    = nsd_open,
	.release = nsd_release,
	.poll    = nsd_poll,
	.unlocked_ioctl = nsd_ioctl,
};

static struct dvb_device dvbdev_nsd = {
	.priv    = 0,
	.readers = 1,
	.writers = 1,
	.users   = 1,
	.fops    = &nsd_fops,
};

static int ddb_nsd_attach(struct ddb *dev)
{
	int ret;

	ret = dvb_register_device(&dev->adap[0],
				  &dev->nsd_dev,
				  &dvbdev_nsd, (void *) dev,
				  DVB_DEVICE_NSD);
	return ret;
}

static void ddb_nsd_detach(struct ddb *dev)
{
	if (dev->nsd_dev->users > 2) {
		wait_event(dev->nsd_dev->wait_queue,
			   dev->nsd_dev->users == 2);
	}
	dvb_unregister_device(dev->nsd_dev);
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static int flashio(struct ddb *dev, u8 *wbuf, u32 wlen, u8 *rbuf, u32 rlen)
{
	u32 data, shift;

	if (wlen > 4)
		ddbwritel(dev, 1, SPI_CONTROL);
	while (wlen > 4) {
		/* FIXME: check for big-endian */
		data = swab32(*(u32 *)wbuf);
		wbuf += 4;
		wlen -= 4;
		ddbwritel(dev, data, SPI_DATA);
		while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
			;
	}

	if (rlen)
		ddbwritel(dev, 0x0001 | ((wlen << (8 + 3)) & 0x1f00),
			  SPI_CONTROL);
	else
		ddbwritel(dev, 0x0003 | ((wlen << (8 + 3)) & 0x1f00),
			  SPI_CONTROL);

	data = 0;
	shift = ((4 - wlen) * 8);
	while (wlen) {
		data <<= 8;
		data |= *wbuf;
		wlen--;
		wbuf++;
	}
	if (shift)
		data <<= shift;
	ddbwritel(dev, data, SPI_DATA);
	while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
		;

	if (!rlen) {
		ddbwritel(dev, 0, SPI_CONTROL);
		return 0;
	}
	if (rlen > 4)
		ddbwritel(dev, 1, SPI_CONTROL);

	while (rlen > 4) {
		ddbwritel(dev, 0xffffffff, SPI_DATA);
		while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
			;
		data = ddbreadl(dev, SPI_DATA);
		*(u32 *) rbuf = swab32(data);
		rbuf += 4;
		rlen -= 4;
	}
	ddbwritel(dev, 0x0003 | ((rlen << (8 + 3)) & 0x1F00), SPI_CONTROL);
	ddbwritel(dev, 0xffffffff, SPI_DATA);
	while (ddbreadl(dev, SPI_CONTROL) & 0x0004)
		;

	data = ddbreadl(dev, SPI_DATA);
	ddbwritel(dev, 0, SPI_CONTROL);

	if (rlen < 4)
		data <<= ((4 - rlen) * 8);

	while (rlen > 0) {
		*rbuf = ((data >> 24) & 0xff);
		data <<= 8;
		rbuf++;
		rlen--;
	}
	return 0;
}

int ddbridge_flashread(struct ddb *dev, u8 *buf, u32 addr, u32 len)
{
	u8 cmd[4] = {0x03, (addr >> 16) & 0xff,
		     (addr >> 8) & 0xff, addr & 0xff};

	return flashio(dev, cmd, 4, buf, len);
}

static int mdio_write(struct ddb *dev, u8 adr, u8 reg, u16 val)
{
	ddbwritel(dev, adr, MDIO_ADR);
	ddbwritel(dev, reg, MDIO_REG);
	ddbwritel(dev, val, MDIO_VAL);
	ddbwritel(dev, 0x03, MDIO_CTRL);
	while (ddbreadl(dev, MDIO_CTRL) & 0x02)
		ndelay(500);
	return 0;
}

static u16 mdio_read(struct ddb *dev, u8 adr, u8 reg)
{
	ddbwritel(dev, adr, MDIO_ADR);
	ddbwritel(dev, reg, MDIO_REG);
	ddbwritel(dev, 0x07, MDIO_CTRL);
	while (ddbreadl(dev, MDIO_CTRL) & 0x02)
		ndelay(500);
	return ddbreadl(dev, MDIO_VAL);
}

#define DDB_MAGIC 'd'

struct ddb_flashio {
	__u8 *write_buf;
	__u32 write_len;
	__u8 *read_buf;
	__u32 read_len;
};

struct ddb_gpio {
	__u32 mask;
	__u32 data;
};

struct ddb_id {
	__u16 vendor;
	__u16 device;
	__u16 subvendor;
	__u16 subdevice;
	__u32 hw;
	__u32 regmap;
};

struct ddb_reg {
	__u32 reg;
	__u32 val;
};

struct ddb_mem {
	__u32  off;
	__u8  *buf;
	__u32  len;
};

struct ddb_mdio {
	__u8   adr;
	__u8   reg;
	__u16  val;
};

#define IOCTL_DDB_FLASHIO    _IOWR(DDB_MAGIC, 0x00, struct ddb_flashio)
#define IOCTL_DDB_GPIO_IN    _IOWR(DDB_MAGIC, 0x01, struct ddb_gpio)
#define IOCTL_DDB_GPIO_OUT   _IOWR(DDB_MAGIC, 0x02, struct ddb_gpio)
#define IOCTL_DDB_ID         _IOR(DDB_MAGIC, 0x03, struct ddb_id)
#define IOCTL_DDB_READ_REG   _IOWR(DDB_MAGIC, 0x04, struct ddb_reg)
#define IOCTL_DDB_WRITE_REG  _IOW(DDB_MAGIC, 0x05, struct ddb_reg)
#define IOCTL_DDB_READ_MEM   _IOWR(DDB_MAGIC, 0x06, struct ddb_mem)
#define IOCTL_DDB_WRITE_MEM  _IOR(DDB_MAGIC, 0x07, struct ddb_mem)
#define IOCTL_DDB_READ_MDIO  _IOWR(DDB_MAGIC, 0x08, struct ddb_mdio)
#define IOCTL_DDB_WRITE_MDIO _IOR(DDB_MAGIC, 0x09, struct ddb_mdio)

#define DDB_NAME "ddbridge"

static u32 ddb_num;
static int ddb_major;
static DEFINE_MUTEX(ddb_mutex);

static int ddb_release(struct inode *inode, struct file *file)
{
	struct ddb *dev = file->private_data;

	dev->ddb_dev_users--;
	return 0;
}

static int ddb_open(struct inode *inode, struct file *file)
{
	struct ddb *dev = ddbs[iminor(inode)];

	if (dev->ddb_dev_users)
		return -EBUSY;
	dev->ddb_dev_users++;
	file->private_data = dev;
	return 0;
}

static long ddb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ddb *dev = file->private_data;
	void *parg = (void *)arg;
	int res;

	switch (cmd) {
	case IOCTL_DDB_FLASHIO:
	{
		struct ddb_flashio fio;
		u8 *rbuf, *wbuf;

		if (copy_from_user(&fio, parg, sizeof(fio)))
			return -EFAULT;
		if (fio.write_len > 1028 || fio.read_len > 1028)
			return -EINVAL;
		if (fio.write_len + fio.read_len > 1028)
			return -EINVAL;

		wbuf = &dev->iobuf[0];
		rbuf = wbuf + fio.write_len;

		if (copy_from_user(wbuf, fio.write_buf, fio.write_len))
			return -EFAULT;
		res = flashio(dev, wbuf, fio.write_len, rbuf, fio.read_len);
		if (res)
			return res;
		if (copy_to_user(fio.read_buf, rbuf, fio.read_len))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_GPIO_OUT:
	{
		struct ddb_gpio gpio;
		if (copy_from_user(&gpio, parg, sizeof(gpio)))
			return -EFAULT;
		ddbwritel(dev, gpio.mask, GPIO_DIRECTION);
		ddbwritel(dev, gpio.data, GPIO_OUTPUT);
		break;
	}
	case IOCTL_DDB_ID:
	{
		struct ddb_id ddbid;

		ddbid.vendor = dev->ids.vendor;
		ddbid.device = dev->ids.device;
		ddbid.subvendor = dev->ids.subvendor;
		ddbid.subdevice = dev->ids.subdevice;
		ddbid.hw = ddbreadl(dev, 0);
		ddbid.regmap = ddbreadl(dev, 4);
		if (copy_to_user(parg, &ddbid, sizeof(ddbid)))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_READ_REG:
	{
		struct ddb_reg reg;

		if (copy_from_user(&reg, parg, sizeof(reg)))
			return -EFAULT;
		if (reg.reg >= dev->regs_len)
			return -EINVAL;
		reg.val = ddbreadl(dev, reg.reg);
		if (copy_to_user(parg, &reg, sizeof(reg)))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_WRITE_REG:
	{
		struct ddb_reg reg;

		if (copy_from_user(&reg, parg, sizeof(reg)))
			return -EFAULT;
		if (reg.reg >= dev->regs_len)
			return -EINVAL;
		ddbwritel(dev, reg.val, reg.reg);
		break;
	}
	case IOCTL_DDB_READ_MDIO:
	{
		struct ddb_mdio mdio;

		if (copy_from_user(&mdio, parg, sizeof(mdio)))
			return -EFAULT;
		mdio.val = mdio_read(dev, mdio.adr, mdio.reg);
		if (copy_to_user(parg, &mdio, sizeof(mdio)))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_WRITE_MDIO:
	{
		struct ddb_mdio mdio;

		if (copy_from_user(&mdio, parg, sizeof(mdio)))
			return -EFAULT;
		mdio_write(dev, mdio.adr, mdio.reg, mdio.val);
		break;
	}
	case IOCTL_DDB_READ_MEM:
	{
		struct ddb_mem mem;
		u8 *buf = &dev->iobuf[0];

		if (copy_from_user(&mem, parg, sizeof(mem)))
			return -EFAULT;
		if ((mem.len + mem.off > dev->regs_len) ||
		    mem.len > 1024)
			return -EINVAL;
		ddbcpyfrom(dev, buf, mem.off, mem.len);
		if (copy_to_user(mem.buf, buf, mem.len))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_WRITE_MEM:
	{
		struct ddb_mem mem;
		u8 *buf = &dev->iobuf[0];

		if (copy_from_user(&mem, parg, sizeof(mem)))
			return -EFAULT;
		if ((mem.len + mem.off > dev->regs_len) ||
		    mem.len > 1024)
			return -EINVAL;
		if (copy_from_user(buf, mem.buf, mem.len))
			return -EFAULT;
		ddbcpyto(dev, mem.off, buf, mem.len);
		break;
	}
	default:
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations ddb_fops = {
	.unlocked_ioctl = ddb_ioctl,
	.open           = ddb_open,
	.release        = ddb_release,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0))
static char *ddb_devnode(struct device *device, mode_t *mode)
#else
static char *ddb_devnode(struct device *device, umode_t *mode)
#endif
{
	struct ddb *dev = dev_get_drvdata(device);

	return kasprintf(GFP_KERNEL, "ddbridge/card%d", dev->nr);
}

#define __ATTR_MRO(_name, _show) {				\
	.attr	= { .name = __stringify(_name), .mode = 0444 },	\
	.show	= _show,					\
}

#define __ATTR_MWO(_name, _store) {				\
	.attr	= { .name = __stringify(_name), .mode = 0222 },	\
	.store	= _store,					\
}

static ssize_t ports_show(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dev->info->port_num);
}

static ssize_t ts_irq_show(struct device *device,
			   struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dev->ts_irq);
}

static ssize_t i2c_irq_show(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dev->i2c_irq);
}

static char *class_name[] = {
	"NONE", "CI", "TUNER", "LOOP"
};

static char *type_name[] = {
	"NONE", "DVBS_ST", "DVBS_ST_AA", "DVBCT_TR",
	"DVBCT_ST", "INTERNAL", "CXD2099", "TYPE07",
	"TYPE08", "TYPE09", "TYPE0A", "TYPE0B",
	"TYPE0C", "TYPE0D", "TYPE0E", "TYPE0F",
	"DVBS", "DVBCT2_SONY", "ISDBT_SONY", "DVBC2T2_SONY",
	"ATSC_ST", "DVBC2T2_ST"
};

static ssize_t fan_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	u32 val;

	val = ddbreadl(dev, GPIO_OUTPUT) & 1;
	return sprintf(buf, "%d\n", val);
}

static ssize_t fan_store(struct device *device, struct device_attribute *d,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device);
	unsigned val;

	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	ddbwritel(dev, 1, GPIO_DIRECTION);
	ddbwritel(dev, val & 1, GPIO_OUTPUT);
	return count;
}

static ssize_t temp_show(struct device *device,
			 struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	struct i2c_adapter *adap;
	int temp, temp2, temp3, i;
	u8 tmp[2];

	if (dev->info->type == DDB_MOD) {
		ddbwritel(dev, 1, TEMPMON_CONTROL);
		for (i = 0; i < 10; i++) {
			if (0 == (1 & ddbreadl(dev, TEMPMON_CONTROL)))
				break;
			usleep_range(1000, 2000);
		}
		temp = ddbreadl(dev, TEMPMON_SENSOR1);
		temp2 = ddbreadl(dev, TEMPMON_SENSOR2);
		temp = (temp * 1000) >> 8;
		temp2 = (temp2 * 1000) >> 8;
		if (ddbreadl(dev, TEMPMON_CONTROL) & 0x8000) {
			temp3 = ddbreadl(dev, TEMPMON_CORE);
			temp3 = (temp3 * 1000) >> 8;
			return sprintf(buf, "%d %d %d\n", temp, temp2, temp3);
		}
		return sprintf(buf, "%d %d\n", temp, temp2);
	}
	if (!dev->info->temp_num)
		return sprintf(buf, "no sensor\n");
	adap = &dev->i2c[dev->info->temp_bus].adap;
	if (i2c_read_regs(adap, 0x48, 0, tmp, 2) < 0)
		return sprintf(buf, "read_error\n");
	temp = (tmp[0] << 3) | (tmp[1] >> 5);
	temp *= 125;
	if (dev->info->temp_num == 2) {
		if (i2c_read_regs(adap, 0x49, 0, tmp, 2) < 0)
			return sprintf(buf, "read_error\n");
		temp2 = (tmp[0] << 3) | (tmp[1] >> 5);
		temp2 *= 125;
		return sprintf(buf, "%d %d\n", temp, temp2);
	}
	return sprintf(buf, "%d\n", temp);
}

static ssize_t qam_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	struct i2c_adapter *adap;
	u8 tmp[4];
	s16 i, q;

	adap = &dev->i2c[1].adap;
	if (i2c_read_regs16(adap, 0x1f, 0xf480, tmp, 4) < 0)
		return sprintf(buf, "read_error\n");
	i = (s16) (((u16) tmp[1]) << 14) | (((u16) tmp[0]) << 6);
	q = (s16) (((u16) tmp[3]) << 14) | (((u16) tmp[2]) << 6);

	return sprintf(buf, "%d %d\n", i, q);
}

static ssize_t mod_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;

	return sprintf(buf, "%s:%s\n",
		       class_name[dev->port[num].class],
		       type_name[dev->port[num].type]);
}

static ssize_t led_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;

	return sprintf(buf, "%d\n", dev->leds & (1 << num) ? 1 : 0);
}


static void ddb_set_led(struct ddb *dev, int num, int val)
{
	if (!dev->info->led_num)
		return;
	switch (dev->port[num].class) {
	case DDB_PORT_TUNER:
		switch (dev->port[num].type) {
		case DDB_TUNER_DVBS_ST:
			i2c_write_reg16(&dev->i2c[num].adap,
					0x69, 0xf14c, val ? 2 : 0);
			break;
		case DDB_TUNER_DVBCT_ST:
			i2c_write_reg16(&dev->i2c[num].adap,
					0x1f, 0xf00e, 0);
			i2c_write_reg16(&dev->i2c[num].adap,
					0x1f, 0xf00f, val ? 1 : 0);
			break;
		case DDB_TUNER_XO2 ... DDB_TUNER_DVBC2T2_ST:
		{
			u8 v;

			i2c_read_reg(&dev->i2c[num].adap, 0x10, 0x08, &v);
			v = (v & ~0x10) | (val ? 0x10 : 0);
			i2c_write_reg(&dev->i2c[num].adap, 0x10, 0x08, v);
			break;
		}
		default:
			break;
		}
		break;
	}
}

static ssize_t led_store(struct device *device,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;
	unsigned val;

	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	if (val)
		dev->leds |= (1 << num);
	else
		dev->leds &= ~(1 << num);
	ddb_set_led(dev, num, val);
	return count;
}

static ssize_t snr_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	char snr[32];
	int num = attr->attr.name[3] - 0x30;
	
	if (dev->port[num].type >= DDB_TUNER_XO2) {
		if (i2c_read_regs(&dev->i2c[num].adap, 0x10, 0x10, snr, 16) < 0)
			return sprintf(buf, "NO SNR\n");
		snr[16] = 0;
	} else {
		/* serial number at 0x100-0x11f */
		if (i2c_read_regs16(&dev->i2c[num].adap, 0x50, 0x100, snr, 32) < 0)
			if (i2c_read_regs16(&dev->i2c[num].adap,
					    0x57, 0x100, snr, 32) < 0)
				return sprintf(buf, "NO SNR\n");
		snr[31] = 0; /* in case it is not terminated on EEPROM */
	}
	return sprintf(buf, "%s\n", snr);
}


static ssize_t snr_store(struct device *device, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;
	u8 snr[34] = { 0x01, 0x00 };

	if (count > 31)
		return -EINVAL;
	if (dev->port[num].type >= DDB_TUNER_XO2)
		return -EINVAL;
	memcpy(snr + 2, buf, count);
	i2c_write(&dev->i2c[num].adap, 0x57, snr, 34);
	i2c_write(&dev->i2c[num].adap, 0x50, snr, 34);
	return count;
}

static ssize_t bsnr_show(struct device *device,
			 struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	char snr[16];

	ddbridge_flashread(dev, snr, 0x10, 15);
	snr[15] = 0; /* in case it is not terminated on EEPROM */
	return sprintf(buf, "%s\n", snr);
}

static ssize_t redirect_show(struct device *device,
			     struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t redirect_store(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned int i, p;
	int res;

	if (sscanf(buf, "%x %x\n", &i, &p) != 2)
		return -EINVAL;
	res = ddb_redirect(i, p);
	if (res < 0)
		return res;
	pr_info("redirect: %02x, %02x\n", i, p);
	return count;
}

static ssize_t gap_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;

	return sprintf(buf, "%d\n", dev->port[num].gap);

}
static ssize_t gap_store(struct device *device, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ddb *dev = dev_get_drvdata(device);
	int num = attr->attr.name[3] - 0x30;
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	if (val > 20)
		return -EINVAL;
	dev->port[num].gap = val;
	return count;
}

static ssize_t version_show(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "%08x %08x\n",
		       ddbreadl(dev, 0), ddbreadl(dev, 4));
}

static ssize_t hwid_show(struct device *device,
			 struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "0x%08X\n", dev->ids.hwid);
}

static ssize_t regmap_show(struct device *device,
			   struct device_attribute *attr, char *buf)
{
	struct ddb *dev = dev_get_drvdata(device);

	return sprintf(buf, "0x%08X\n", dev->ids.regmapid);
}

static struct device_attribute ddb_attrs[] = {
	__ATTR_RO(version),
	__ATTR_RO(ports),
	__ATTR_RO(ts_irq),
	__ATTR_RO(i2c_irq),
	__ATTR(gap0, 0666, gap_show, gap_store),
	__ATTR(gap1, 0666, gap_show, gap_store),
	__ATTR(gap2, 0666, gap_show, gap_store),
	__ATTR(gap3, 0666, gap_show, gap_store),
	__ATTR_RO(hwid),
	__ATTR_RO(regmap),
#if 0
	__ATTR_RO(qam),
#endif
	__ATTR(redirect, 0666, redirect_show, redirect_store),
	__ATTR_MRO(snr,  bsnr_show),
	__ATTR_NULL,
};

static struct device_attribute ddb_attrs_temp[] = {
	__ATTR_RO(temp),
};

static struct device_attribute ddb_attrs_mod[] = {
	__ATTR_MRO(mod0, mod_show),
	__ATTR_MRO(mod1, mod_show),
	__ATTR_MRO(mod2, mod_show),
	__ATTR_MRO(mod3, mod_show),
	__ATTR_MRO(mod4, mod_show),
	__ATTR_MRO(mod5, mod_show),
	__ATTR_MRO(mod6, mod_show),
	__ATTR_MRO(mod7, mod_show),
	__ATTR_MRO(mod8, mod_show),
	__ATTR_MRO(mod9, mod_show),
};

static struct device_attribute ddb_attrs_fan[] = {
	__ATTR(fan, 0666, fan_show, fan_store),
};

static struct device_attribute ddb_attrs_snr[] = {
	__ATTR(snr0, 0666, snr_show, snr_store),
	__ATTR(snr1, 0666, snr_show, snr_store),
	__ATTR(snr2, 0666, snr_show, snr_store),
	__ATTR(snr3, 0666, snr_show, snr_store),
};

static struct device_attribute ddb_attrs_led[] = {
	__ATTR(led0, 0666, led_show, led_store),
	__ATTR(led1, 0666, led_show, led_store),
	__ATTR(led2, 0666, led_show, led_store),
	__ATTR(led3, 0666, led_show, led_store),
};

static struct class ddb_class = {
	.name		= "ddbridge",
	.owner          = THIS_MODULE,
#if 0
	.dev_attrs	= ddb_attrs,
#endif
	.devnode        = ddb_devnode,
};

static int ddb_class_create(void)
{
	ddb_major = register_chrdev(0, DDB_NAME, &ddb_fops);
	if (ddb_major < 0)
		return ddb_major;
	if (class_register(&ddb_class) < 0)
		return -1;
	return 0;
}

static void ddb_class_destroy(void)
{
	class_unregister(&ddb_class);
	unregister_chrdev(ddb_major, DDB_NAME);
}

static void ddb_device_attrs_del(struct ddb *dev)
{
	int i;

	for (i = 0; i < dev->info->temp_num; i++)
		device_remove_file(dev->ddb_dev, &ddb_attrs_temp[i]);
	for (i = 0; i < dev->info->port_num; i++)
		device_remove_file(dev->ddb_dev, &ddb_attrs_mod[i]);
	for (i = 0; i < dev->info->fan_num; i++)
		device_remove_file(dev->ddb_dev, &ddb_attrs_fan[i]);
	for (i = 0; i < dev->info->i2c_num; i++) {
		if (dev->info->led_num)
			device_remove_file(dev->ddb_dev, &ddb_attrs_led[i]);
		device_remove_file(dev->ddb_dev, &ddb_attrs_snr[i]);
	}
	for (i = 0; ddb_attrs[i].attr.name; i++)
		device_remove_file(dev->ddb_dev, &ddb_attrs[i]);
}

static int ddb_device_attrs_add(struct ddb *dev)
{
	int i;
	
	for (i = 0; ddb_attrs[i].attr.name; i++)
		if (device_create_file(dev->ddb_dev, &ddb_attrs[i]))
			goto fail;
	for (i = 0; i < dev->info->temp_num; i++)
		if (device_create_file(dev->ddb_dev, &ddb_attrs_temp[i]))
			goto fail;
	for (i = 0; i < dev->info->port_num; i++)
		if (device_create_file(dev->ddb_dev, &ddb_attrs_mod[i]))
			goto fail;
	for (i = 0; i < dev->info->fan_num; i++)
		if (device_create_file(dev->ddb_dev, &ddb_attrs_fan[i]))
			goto fail;
	for (i = 0; i < dev->info->i2c_num; i++) {
		if (device_create_file(dev->ddb_dev, &ddb_attrs_snr[i]))
			goto fail;
		if (dev->info->led_num)
			if (device_create_file(dev->ddb_dev,
					       &ddb_attrs_led[i]))
				goto fail;
	}
	return 0;
fail:
	return -1;
}

static int ddb_device_create(struct ddb *dev)
{
	int res = 0;

	if (ddb_num == DDB_MAX_ADAPTER)
		return -ENOMEM;
	mutex_lock(&ddb_mutex);
	dev->nr = ddb_num;
	ddbs[dev->nr] = dev;
	dev->ddb_dev = device_create(&ddb_class, dev->dev,
				     MKDEV(ddb_major, dev->nr),
				     dev, "ddbridge%d", dev->nr);
	if (IS_ERR(dev->ddb_dev)) {
		res = PTR_ERR(dev->ddb_dev);
		pr_info("Could not create ddbridge%d\n", dev->nr);
		goto fail;
	}
	res = ddb_device_attrs_add(dev);
	if (res) {
		ddb_device_attrs_del(dev);
		device_destroy(&ddb_class, MKDEV(ddb_major, dev->nr));
		ddbs[dev->nr] = 0;
		dev->ddb_dev = ERR_PTR(-ENODEV);
	} else
		ddb_num++;
fail:
	mutex_unlock(&ddb_mutex);
	return res;
}

static void ddb_device_destroy(struct ddb *dev)
{
	if (IS_ERR(dev->ddb_dev))
		return;
	ddb_device_attrs_del(dev);
	device_destroy(&ddb_class, MKDEV(ddb_major, dev->nr));
}
