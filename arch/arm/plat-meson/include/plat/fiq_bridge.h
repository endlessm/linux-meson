#ifndef   FIQ_BRIDGE_H
#define FIQ_BRIDGE_H
#include <linux/list.h>
#include <linux/irqreturn.h>
#include <mach/am_regs.h>
#include <linux/io.h>
typedef   irqreturn_t (*bridge_handle_t)(int irq, void *dev_id);
typedef  struct{
	
	bridge_handle_t  	handle;
	u32				key;
	u32				active;
	const   char*		name;
	struct  list_head	list;
   
}bridge_item_t;

static LIST_HEAD(fiq_bridge_list);  

#define BRIDGE_IRQ INT_TIMER_C
#define BRIDGE_IRQ_SET() WRITE_CBUS_REG(ISA_TIMERC, 1)

extern int fiq_bridge_pulse_trigger(bridge_item_t *c_item);
extern int register_fiq_bridge_handle(bridge_item_t *c_item);
extern int unregister_fiq_bridge_handle(bridge_item_t *c_item);

#endif

