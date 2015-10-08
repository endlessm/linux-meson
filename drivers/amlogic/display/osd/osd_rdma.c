/*
 * Amlogic Meson
 * frame buffer driver
 *
 * Copyright (C) 2009 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author: Amlogic Platform-BJ
 *
 * description
 *     rdma table work as REGISTER Cache for read write.
 */
#include "osd_rdma.h"
#include <linux/amlogic/amlog.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

static DEFINE_MUTEX(rdma_mutex);
static DEFINE_SPINLOCK(rdma_lock);

static rdma_table_item_t *rdma_table = NULL;
static u32		   table_paddr = 0;
static u32		   rdma_enable = 0;
static u32		   item_count = 0;
static u32 		   rdma_debug = 0;
static char		   *info;
static bool		osd_rdma_init_flat = false;
static int ctrl_ahb_rd_burst_size = 3;
static int ctrl_ahb_wr_burst_size = 3;

static int  osd_rdma_init(void);
void osd_rdma_start(void);

#define OSD_RDMA_UPDATE_RETRY_COUNT 500

#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
#define osd_rdma_write(adr, val) WRITE_VCBUS_REG(adr, val)
#define osd_rdma_read(adr)    READ_VCBUS_REG(adr)
#define osd_rdma_write_reg_bits(adr, val, start, len)  \
		WRITE_VCBUS_REG_BITS(adr, val, start, len)
#define osd_rdma_write_set_reg_bits_mask(adr, _mask) \
		SET_VCBUS_REG_MASK(adr, _mask)
#define osd_rdma_write_clr_reg_bits_mask(adr, _mask) \
		CLEAR_VCBUS_REG_MASK(adr, _mask)
#else
#define osd_rdma_write(adr, val) WRITE_MPEG_REG(adr, val)
#define osd_rdma_read(adr)    READ_MPEG_REG(adr)
#define osd_rdma_write_reg_bits(adr, val, start, len) \
		WRITE_MPEG_REG_BITS(adr, val, start, len)
#define osd_rdma_write_set_reg_bits_mask(adr, _mask) \
		SET_MPEG_REG_MASK(adr, _mask)
#define osd_rdma_write_clr_reg_bits_mask(adr, _mask) \
		CLEAR_MPEG_REG_MASK(adr, _mask)
#endif

#define osd_rdma_mem_cpy(dst, src, len) \
	asm volatile( \
		"	stmfd sp!,{r5-r7}\n" \
		"@can be remove ?\n" \
		"	cmp %2,#8\n" \
		"@8 means eight byte\n" \
		"	bne	1f\n" \
		"	ldmia %0,{r7,r8}\n" \
		"@load src into regs\n" \
		"	stmia %1,{r7,r8}\n" \
		"@store dst from regs\n" \
		"	b 2f\n" \
		"1:	ldmia %0,{r5-r8}\n" \
		"@16 bytes\n" \
		"	stmia %1,{r5-r8}\n" \
		"2:	nop\n" \
		"	ldmfd sp!,{r5-r7}\n" \
		"@can be remove?\n" \
		: \
		: "r" (src), "r" (dst), "r" (len) \
		: "r5", "r6", "r7", "r8")

/*once init, will update config in every osd rdma interrupt*/
int osd_rdma_update_config(char is_init)
{
	static u32 config;

	if (is_init) {
		config  = 0;
		config |= 1                         << 6;   /* [31: 6] Rsrv.*/
		config |= ctrl_ahb_wr_burst_size    <<
			4;
		/* [ 5: 4] ctrl_ahb_wr_burst_size. 0=16; 1=24; 2=32; 3=48.*/
		config |= ctrl_ahb_rd_burst_size    <<
			2;
		/* [ 3: 2] ctrl_ahb_rd_burst_size. 0=16; 1=24; 2=32; 3=48.*/
		config |= 0                         << 1;
		/* [    1] ctrl_sw_reset.*/
		config |= 0                         << 0;
		/* [    0] ctrl_free_clk_enable.*/
		osd_rdma_write(RDMA_CTRL, config);
	} else {
		osd_rdma_write(RDMA_CTRL, (1<<24)|config);
	}
	return 0;

}
EXPORT_SYMBOL(osd_rdma_update_config);
static void inline reset_rdma_table(void)
{
	unsigned long flags;
	rdma_table_item_t reset_item[2] = {
		{
			.addr = OSD_RDMA_FLAG_REG,
			.val = OSD_RDMA_STATUS_MARK_TBL_RST,
		},
		{
			.addr = OSD_RDMA_FLAG_REG,
			.val = OSD_RDMA_STATUS_MARK_COMPLETE,
		}
	};
	spin_lock_irqsave(&rdma_lock, flags);
	if (!OSD_RDMA_STAUS_IS_DIRTY) {
		/*we reset rdma table only if table is clean.
		if both two process want to reset table.and racing
		here,double clean table is uncorrect.
		1 clean table racing--------
		2 the first one get spin lock ,do clean op
		3 the first exit spin lock and adding one item
		4 the second one get spin lock, clean table(!!wrong)*/
		OSD_RDMA_STAUS_CLEAR_DONE;
		osd_rdma_write(END_ADDR, table_paddr - 1);
		osd_rdma_mem_cpy(rdma_table, &reset_item[0], 16);
		/*memcpy(rdma_table,&reset_item[0],16);	*/
		item_count = 1;
	}
	spin_unlock_irqrestore(&rdma_lock, flags);
}

static int update_table_item(u32 addr, u32 val)
{
	unsigned long flags;
	int retry_count = OSD_RDMA_UPDATE_RETRY_COUNT;
	rdma_table_item_t request_item;
	if (item_count > 500) {
		/*rdma table is full*/
		return -1;
	}
	if (info)
		sprintf(info, "item_count : %d\n"
				"reg_ctrl : %x\n"
				"reg_status : %x\n"
				"reg_auto :0x%x\n"
				"reg_flag :0x%x\n",
				item_count, osd_rdma_read(RDMA_CTRL),
				osd_rdma_read(RDMA_STATUS),
				osd_rdma_read(RDMA_ACCESS_AUTO),
				osd_rdma_read(OSD_RDMA_FLAG_REG));

retry:
	if (0 == (retry_count--)) {
		pr_info("OSD RDMA stuck .....%d,0x%x\n", retry_count,
					osd_rdma_read(RDMA_STATUS));
		return -1;
	}
	if (!OSD_RDMA_STAUS_IS_DIRTY) {
		/*since last HW op,no new wirte request.
		rdma HW op will clear DIRTY flag.
		reset all pointer. set table start margin.*/
		reset_rdma_table();
	}
	/*atom_lock_start:*/
	/*set write op aotmic lock flag.*/
	OSD_RDMA_STAUS_MARK_DIRTY;
	spin_lock_irqsave(&rdma_lock, flags);
	item_count++;
	request_item.addr = OSD_RDMA_FLAG_REG;
	request_item.val = OSD_RDMA_STATUS_MARK_COMPLETE;
	osd_rdma_mem_cpy(&rdma_table[item_count], &request_item, 8);
	if ((osd_rdma_read(RDMA_STATUS)&0xffffff0f) == 0) {
		osd_rdma_write(END_ADDR, (table_paddr + item_count * 8 + 7));
	} else {
		if (osd_rdma_read(RDMA_STATUS)&(1<<28)) {
			osd_rdma_update_config((char)0);/*not init.*/
			pr_info("rdma done detect +++++%d,0x%x\n",
				retry_count, osd_rdma_read(RDMA_STATUS));
		}
		item_count--;
		spin_unlock_irqrestore(&rdma_lock, flags);
		goto retry;
	}
	request_item.addr = addr;
	request_item.val = val;
	osd_rdma_mem_cpy(&rdma_table[item_count-1], &request_item, 8);
	/*memcpy(&rdma_table[item_count],&request_item,8);*/
	/*if dirty flag is cleared, then RDMA hw write and
				cpu sw write is racing.*/
	/*if reject flag is true,then hw RDMA hw write
				start when cpu write.*/
	/*atom_lock_end:*/
	if (!OSD_RDMA_STAUS_IS_DIRTY || OSD_RDMA_STATUS_IS_REJECT) {
		/*spin_lock_irqsave(&rdma_lock, flags);*/
		item_count--;
		spin_unlock_irqrestore(&rdma_lock, flags);
		pr_info("osd_rdma flag ++++++: %x\n",
				osd_rdma_read(OSD_RDMA_FLAG_REG));
		goto retry ;
	}
	spin_unlock_irqrestore(&rdma_lock, flags);
	return 0;
}

u32  VSYNCOSD_RD_MPEG_REG(unsigned long addr)
{
	int  i;

	if (rdma_enable) {
		for (i = (item_count - 1); i >= 0; i--) {
			if (addr == rdma_table[i].addr)
				return rdma_table[i].val;
		}
	}
	return osd_rdma_read(addr);
}
EXPORT_SYMBOL(VSYNCOSD_RD_MPEG_REG);

int VSYNCOSD_WR_MPEG_REG(unsigned long addr, unsigned long val)
{
	if (rdma_enable)
		return update_table_item(addr, val);
	else
		osd_rdma_write(addr, val);
	return 0;
}
EXPORT_SYMBOL(VSYNCOSD_WR_MPEG_REG);

int VSYNCOSD_WR_MPEG_REG_BITS(unsigned long addr, unsigned long val,
			      unsigned long start, unsigned long len)
{
	unsigned long read_val;
	unsigned long write_val;

	if (rdma_enable) {
		read_val = VSYNCOSD_RD_MPEG_REG(addr);
		write_val = (read_val & ~(((1L << (len)) - 1) << (start)))|
			((unsigned int)(val) << (start));
		update_table_item(addr, write_val);
	} else
		osd_rdma_write_reg_bits(addr, val, start, len);

	return 0;
}
EXPORT_SYMBOL(VSYNCOSD_WR_MPEG_REG_BITS);

int VSYNCOSD_SET_MPEG_REG_MASK(unsigned long addr, unsigned long _mask)
{
	unsigned long read_val;
	unsigned long write_val;

	if (rdma_enable) {
		read_val = VSYNCOSD_RD_MPEG_REG(addr);
		write_val = read_val | _mask ;
		update_table_item(addr, write_val);
	} else
		osd_rdma_write_set_reg_bits_mask(addr, _mask);

	return 0;
}
EXPORT_SYMBOL(VSYNCOSD_SET_MPEG_REG_MASK);

int VSYNCOSD_CLR_MPEG_REG_MASK(unsigned long addr, unsigned long _mask)
{
	unsigned long read_val;
	unsigned long write_val;

	if (rdma_enable) {
		read_val = VSYNCOSD_RD_MPEG_REG(addr);
		write_val = read_val & (~_mask) ;
		update_table_item(addr, write_val);
	} else
		osd_rdma_write_clr_reg_bits_mask(addr, _mask);

	return 0;
}
EXPORT_SYMBOL(VSYNCOSD_CLR_MPEG_REG_MASK);
static int start_osd_rdma(char channel)
{
	char intr_bit = 8 * channel;
	char rw_bit = 4 + channel;
	char inc_bit = channel;
	u32 data32;
	char is_init = 1;

	osd_rdma_update_config(is_init);
	data32  = osd_rdma_read(RDMA_ACCESS_AUTO);
	data32 |= 0x1 <<
		  intr_bit;
	/* [23: 16] interrupt inputs enable mask for auto-start
						1: vsync int bit 0*/
	data32 |= 1 <<
		  rw_bit;
	/* [    6] ctrl_cbus_write_1. 1=Register write;
					0=Register read.*/
	data32 &= ~(1 <<
		    inc_bit);
	/* [    2] ctrl_cbus_addr_incr_1. 1=Incremental register access;
						0=Non-incremental.*/

	osd_rdma_write(RDMA_ACCESS_AUTO, data32);
	return 1;
}

static int stop_rdma(char channel)
{
	char intr_bit = 8 * channel;
	u32 data32 = 0x0;

	data32  = osd_rdma_read(RDMA_ACCESS_AUTO);
	data32 &= ~(0x1 <<
		    intr_bit);
	/* [23: 16] interrupt inputs enable mask for auto-start
						1: vsync int bit 0*/
	osd_rdma_write(RDMA_ACCESS_AUTO, data32);
	return 0;
}

int read_rdma_table(void)
{
	int rdma_count = 0;
	if (rdma_debug) {
		for (rdma_count = 0; rdma_count < item_count; rdma_count++)
			pr_info("rdma_table addr is 0x%x, value is 0x%x\n",
					rdma_table[rdma_count].addr,
					rdma_table[rdma_count].val);
	}
	return 0;
}
EXPORT_SYMBOL(read_rdma_table);

int reset_rdma(void)
{
	/*reset mechanism , to clear rdma status.*/
	if (OSD_RDMA_STAUS_IS_DONE) { /*check if it is OSD rdma completed.*/
		OSD_RDMA_STAUS_CLEAR_DONE;
		/*check if no cpu write request since the latest hw rdma op.*/
		if (!OSD_RDMA_STAUS_IS_DIRTY) {
			/*since last HW op,no new wirte request.
			rdma HW op will clear DIRTY flag.*/
			/*reset all pointer. set table start margin.*/
			reset_rdma_table();
		}
	}
	return 0;
}
EXPORT_SYMBOL(reset_rdma);

int osd_rdma_enable(u32  enable)
{
	if (!osd_rdma_init_flat)
		osd_rdma_init();
	mutex_lock(&rdma_mutex);
	if (enable == rdma_enable) {
		mutex_unlock(&rdma_mutex);
		return 0;
	}
	rdma_enable = enable;
	while (osd_rdma_read(RDMA_STATUS) & 0x0fffff0f) {
		pr_info("rdma still work£¬ RDMA_STATUS: 0x%x\n",
				osd_rdma_read(RDMA_STATUS));
		msleep(10);
	}
	if (enable) {
		reset_rdma_table();
		info = kmalloc(GFP_KERNEL, sizeof(char)*200);
		osd_rdma_write(START_ADDR, table_paddr);
		/*enable then start it.*/
		OSD_RDMA_STATUS_CLEAR_ALL;
		start_osd_rdma(RDMA_CHANNEL_INDEX);
	} else{
		stop_rdma(RDMA_CHANNEL_INDEX);
		kfree(info);
	}
	mutex_unlock(&rdma_mutex);
	return 1;
}
EXPORT_SYMBOL(osd_rdma_enable);

static int osd_rdma_init(void)
{
	/* alloc map table .*/
	static ulong table_vaddr = 0;
	osd_rdma_init_flat = true;

	table_vaddr = __get_free_pages(GFP_KERNEL, get_order(PAGE_SIZE));
	if (!table_vaddr) {
		pr_info("%s: failed to alloc rmda_table\n", __func__);
		return -1;
	}

	table_paddr = virt_to_phys((u8 *)table_vaddr);
	/*remap addr nocache.*/
	rdma_table = (rdma_table_item_t *) ioremap_nocache(table_paddr,
								PAGE_SIZE);
	if (NULL == rdma_table) {
		pr_info("%s: failed to remap rmda_table_addr\n", __func__);
		return -1;
	}

	return 0;
}

MODULE_PARM_DESC(item_count, "\n item_count\n");
module_param(item_count, uint, 0664);
MODULE_PARM_DESC(info, "\n info\n");
module_param(info, charp, S_IRUSR);
MODULE_PARM_DESC(table_paddr, "\n table_paddr\n");
module_param(table_paddr, uint, 0664);

MODULE_PARM_DESC(rdma_debug, "\n rdma_debug\n");
module_param(rdma_debug, uint, 0664);
