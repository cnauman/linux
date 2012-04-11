/*
 * simple fpga irq driver
 */
#define DEBUG
//#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/gpio.h>

#include <linux/sched.h>
#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>	  /* everything... */
#include <linux/errno.h>  /* error codes */
#include <linux/delay.h>  /* udelay */
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
//#include <linux/devfs_fs_kernel.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#define TEST_POINT (7) //(159)

//////////////////////////////////////////////////////////
//#include "timer1-fiq-h.h"
/* linux/drivers/spi/spi_s3c24xx_fiq.h
 *
 * Copyright 2009 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C24XX SPI - FIQ pseudo-DMA transfer support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define GPIO_BASE_GPIO1 0x48310000		// Registers base for arm gpio 1
#define GPIO_BASE_GPIO5 0x49056000		// Registers base for arm gpio 5
#define GPIO_BASE_GPIO6 0x49058000		// Registers base for arm gpio 6
#define GPIO_CLEARDATAOUT 0x090		    	// gpio clear data out offset
#define GPIO_SETDATAOUT 0x094			// gpio set data out offset
#define INTCPS_BASE 0x48200000                  // irq controller registers base
#define GPIO1_MPU_IRQ (29)
// where priority level and irqnfiq bit are set
#define INTCPS_ILR 0x100                        // irq ILR 
#define INTCPS_ITR(n) (0x80+n*0x20)             // irq status
#define INTCPS_MIR(n) (0x84+n*0x20)             // irq mask base
#define INTCPS_MIR_CLEAR(n) (0x088+n*0x20)      // irq mask clear
#define INTCPS_MIR_SET0 0x8c                    // irq mask set
#define INTCPS_ISR_CLEAR(n) (0x094 + n*0x20)    // irq status clear
// used for software trigger
#define INTCPS_SIR_IRQ (0x040)
#define INTCPS_SIR_FIQ (0x044)

//#define FIRST_DEFERRED 67 / 32
#define FIRST_DEFERRED_FIQ (67) //(85) // classified as reserved
#define DEFERRED_PORT (FIRST_DEFERRED_FIQ/32)
#define DEFERRED_IRQ_MASK(m) (m << (FIRST_DEFERRED_FIQ - (DEFERRED_PORT*32))) //0x200000
#define INTCPS_CONTROL (0x048)
#define INTCPS_ISR_SET(n) (0x90 + (n)*0x20)       /* irq status set*/
#define CLEAR_IRQ(irq) { \
    writel((1 << (irq - 32 * (irq / 32))), regs.base_intcps + INTCPS_ISR_CLEAR(irq/32)); \
}

/* We have R8 through R10 to play with */

#ifdef __ASSEMBLY__
#define __REG_NR(x)     r##x
#else
#define __REG_NR(x)     (x)
#endif

#define fiq_mreg	__REG_NR(8)

typedef struct mem_dat {
    void __iomem *base_intcps;
    void __iomem *base_gpio1;
    u32 irq_state;
} mem_data_t;

static struct mem_dat regs = {
    .base_intcps = 0,
    .base_gpio1 = 0,
//    .base_gpio5 = 0,
    .irq_state = 0,
};
typedef struct irq_helper {
    int irq;
    int soft_irq;
} irq_helper_t;
static struct irq_helper alt_irqs = {
    .irq = -1,
    .soft_irq = -1,
};
//////////////////////////////////////////////////////////

#include <asm/io.h>
#include <linux/semaphore.h> //<asm/semaphore.h>
#include <asm/atomic.h>
#include <asm/fiq.h>
#include <asm/ioctls.h>
//#define USE_SIGS
#define MAJORNUM        255
#define MINORNUM        0
#define MODULE_NAME     "fpga"
#define DRV_VERSION     "1.1"
/*#define IRQ_11          56
#define IRQ_10          57
#define IRQ_10_PD9      9
#define IRQ_11_PD8      8*/
//static int IRQS[2] = {
    //Original board: IRQ_GPIOC(16)/*irq 11*/, IRQ_GPIOC(17) /*irq 10*/
    // soft:
//    IRQ_11/*irq 11*/, IRQ_10 /*irq 10*/
    //direct: IRQ_GPIOD(IRQ_11_PORTNUM)/*irq 11*/, IRQ_GPIOD(IRQ_10_PORTNUM) /*irq 10*/
//};
static char irq_name[][16] = {"fiq_fpga_isa_11", "fiq_fpga_isa_10"};
//GPIO PC16
//GPIO PC17

typedef struct fpga_info {
    unsigned long head;
    unsigned long tail;
    unsigned long addr;
    int state;
    int irqstate;
    int irq;
    int soft_irq;
#ifdef USE_SIGS
    struct task_struct * ts;
    int sig;
#endif
    wait_queue_head_t wait;
    spinlock_t lock;
    unsigned long * queue;
} fpga_info_t;
static inline ssize_t number_of_cols(fpga_info_t * card);
#define NUM_CARDS (2)

static struct fpga_info fpga[NUM_CARDS]={ {
    .head = 0,
    .tail = 0,
    .state = 0,
    .irqstate = 0,
    .irq = -1,
    .soft_irq = -1,
#ifdef USE_SIGS
    .ts = NULL,
    .sig = SIGUSR1,
#endif
    .wait = __WAIT_QUEUE_HEAD_INITIALIZER(fpga[0].wait), 
    .lock = __SPIN_LOCK_UNLOCKED(fpga[0].lock), 
    .queue = NULL
}, {
    .head = 0,
    .tail = 0,
    .state = 0,
    .irqstate = 0,
    .irq = -1,
    .soft_irq = -1,
#ifdef USE_SIGS
    .ts = NULL,
    .sig = SIGUSR2,
#endif
    .wait = __WAIT_QUEUE_HEAD_INITIALIZER(fpga[1].wait), 
    .lock = __SPIN_LOCK_UNLOCKED(fpga[1].lock), 
    .queue = NULL
}};

#define READ_IO            (0x02)
enum {PRODUCT_DETECT=0, IRQ_FLAG};

#define SHIFT_CNTL         (0x0E)
#define DATA_PORT          (0x04)
#define PRINT_STATUS_ADDR  (0x0A)
#define DPI_IRQ            (0x0C)
#define GAL_REG            (0x400) //(0x01)
#define READ_IO            (0x02)

#define BUFCOLS         (200)
#define STACKSZE        (20)
#define STRUCTLEN       (3)
#define COLSZE          (STRUCTLEN*STACKSZE+1)
#define QSZE            (BUFCOLS*COLSZE)
#define ISVALID_ID(id)  (0 <= id && id < NUM_CARDS) //sizeof(IRQS))
#define NOP() { asm volatile("nop\n\t"); }
#define outportl(a, v)  { write_word(v>>16, a); write_word(v, a+2); }
#define write_word(v, a) writew(v, a)
//#define write_word(v, a) { *((volatile unsigned short *)(a)) = (v); }
//#define DEFSHIFT_CNTL 0x01FF0001
#define DEFSHIFT_CNTL   0xFF010100
#define SHIFT           0x01004800 
#define PATTERN         0xff00ff00
#define RDY2PRINT       (0xDB)
#define NXTNDX(a)       (((a) + COLSZE) < QSZE ? (a) + COLSZE : 0)
//#define NXTNDX(a) ((a) % DPI)
//#define MASK (1 << 10)
#define MASK            (1 << 31)
#define COLAVAIL(card)  (NXTNDX((card)->tail) != (card)->head)
/* tenths of usec */
#define DELAY(a) { int x = (a) * 4/*(35+5)*/; while (x--) asm volatile("nop\n\t"); }
#if 0
#define debug(fmt,arg...) printk(fmt, ##arg)
#else
#define debug(fmt,arg...) do { } while (0)
#endif
#ifndef USE_SIGS
#define NOTIFY(a) wake_up_interruptible(&(a)->wait);
#else
#define NOTIFY(a) if ((a)->ts) force_sig((a)->sig, (a)->ts);
#endif
//////////////////////////////////////////////////////////////
// Porting stuff
static int IMX_INTFRCH = 0, IMX_INTTYPEH = 0;
static int /*isr_3=0,*/ imr_3=0, icr1_3 = 0;
#define GPIO_INT_PORTD (32)
#define imx_gpio_mode(a)
#define ISR(x) isr_3
#define IMR(x) imr_3
#define ICR1(x) icr1_3
#define devfs_remove(a, ...)
#define CHNGMUP(x) (&(x))
#define devfs_mk_dir(a)
#define devfs_mk_cdev(a, b, c, d)
#define readreg(a) readb(a)
#define writereg(a, b) writeb(a,b)

//////////////////////////////////////////////////////////////
// ioremaped pointers to base of gpio registers and intcps registers
#define IRQ_VAL_OFFSET (160)
static irqreturn_t fpga_interrupt(int irq, void *dev_id) //, struct pt_regs *regs)
{
    fpga_info_t * card = (fpga_info_t *)dev_id;
    int photo_irq;

    IMX_INTFRCH &= ~(1 << (irq - 32));
    CLEAR_IRQ(irq);
//    gpio_set_value(TEST_POINT, 1);
    if (0 == card->irqstate) {
        if (BUFCOLS/2 < number_of_cols(card)) {
            card->state = 1; //++;
            NOTIFY(card);
        }
    } else if (card->head == card->tail) {
        if (3 > card->irqstate) {
            photo_irq = readreg(card->addr+PRINT_STATUS_ADDR);
            if (2 == card->irqstate) {
                /* get interrupt signal from photocell */
                writereg(photo_irq | 0x08, card->addr+PRINT_STATUS_ADDR);
                writereg(1, card->addr+DPI_IRQ); 
            }
            if (RDY2PRINT == (readreg(card->addr+READ_IO) & RDY2PRINT)) {
                card->irqstate = 0;
                if (photo_irq & 0x08) {
                    card->state = 1;
                    NOTIFY(card);
                }
            }
        }
    }
//    gpio_set_value(TEST_POINT, 0);
    return IRQ_HANDLED;
}
static  u32 fiq_cntr=0;

#define IO_SIGNAL_MASK (1<<TEST_POINT) //0x80000000
#define FPGA2_IO    (24)
#define ENET_IO     (25)
#define FPGA1_IO    (26)
#define ADS_IO      (27)
#define FPGA_MASK ((1<<FPGA1_IO) | (1 << FPGA2_IO))
#define IRQ_MASK ((1<<ADS_IO) | (1<<ENET_IO) | FPGA_MASK)
#define VOL /*volatile*/
#define INTCPS(a) *(u32 *)(regs.base_intcps + a)
#define GPIO1_REG(a) *(u32 *)(regs.base_gpio1 + a)
#define REG //register
#define A(x) //asm("r##x")
#define NOP5x() { NOP(); NOP(); NOP(); NOP(); NOP(); }
#define NOP10x() { NOP5x(); NOP5x(); }
#define FIRST_FIQ   FPGA2_IO
#define LAST_FIQ    ADS_IO
#define NUM_FIQS    (LAST_FIQ - FIRST_FIQ + 1)
static unsigned short/*int*/ fiq_irqs[NUM_FIQS], irq_counter[NUM_FIQS];
///////////////////////////////////////////////////////////////////////////

static void DoCards(int ndx/*mask*/) {
    REG unsigned long shift_cntl A(4);
    REG fpga_info_t * card A(5); 
    REG unsigned long * data A(6); //asm("r7"), buf asm("r6");
    REG u32 wordcnt A(8);
    REG u32 addr A(9);
//    REG u32 ndx A(11);
    REG u32 buf A(12);

    //if (!mask) return;
    DELAY(20);
//    GPIO1_REG(GPIO_SETDATAOUT) = IO_SIGNAL_MASK; // io hi
    /*for (ndx=0; ndx < 2; ndx++)*/ {
        card = &fpga[ndx];
        /*if (0 != (mask & (1 << card->irq)))*/ { /* */
            if (card->head != card->tail) {
                data = &card->queue[card->head]; //wordcnt = 3;
                shift_cntl = /*DEFSHIFT_CNTL; */*data++;
                addr = card->addr; /* */
                while (0 != (wordcnt = *data++)) {
                    buf = *data++; //buf = SHIFT;
                    outportl(addr+SHIFT_CNTL, buf);
                    DELAY(230);
                    while (wordcnt--) {
                        buf = *data++; //buf = PATTERN;
                        outportl(addr+DATA_PORT, buf);
                        DELAY(200);
                    } //break;
                }
                outportl(addr+SHIFT_CNTL, shift_cntl);/* */
                DELAY(200);
                //card->queue[card->head] = 0;
                card->head = NXTNDX(card->head);
            } /* */
        }
    }
}

static void fiq_handler(void) {
    //register u32 mask asm("r0");
    u32 buf, mask, i;

    INTCPS(INTCPS_CONTROL) = 2; // clear for new fiq
    buf = INTCPS(INTCPS_ITR(0)); // ldr r12, [r10, #0x80] // fetch irq status

    mask = GPIO1_REG(OMAP24XX_GPIO_IRQSTATUS1) & IRQ_MASK;
//        GPIO1_REG(OMAP24XX_GPIO_CLEARIRQENABLE1) = mask & ~(FPGA_MASK);
    if (!mask) return; // if spurious, not necessary to continue
    GPIO1_REG(OMAP24XX_GPIO_IRQSTATUS1) = mask;

GPIO1_REG(GPIO_SETDATAOUT) = IO_SIGNAL_MASK; // io hi
    fiq_cntr++;

    for (i=0; i < NUM_FIQS; i++) {
        if (mask & (1 << (i + FIRST_FIQ))) {
            fiq_irqs[i]++;
        }
    }
    for (i=0; i < 2; i++) {
        if (mask & (1 << fpga[i].irq)) DoCards(i);
    }
//    mask = mask >> FIRST_FIQ;
//    regs.irq_state |= mask;
    INTCPS(INTCPS_ISR_SET(DEFERRED_PORT)) = (u32)DEFERRED_IRQ_MASK(1);
GPIO1_REG(GPIO_CLEARDATAOUT) = IO_SIGNAL_MASK; // io low
}

///////////////////////////////////////////////////////////////////////////
static irqreturn_t deferred_irq(int irq, void *dev_id)
{
    u32 i, fiq_count, irq_num = 0/*, mask*//*, status, en_mask=0*/; //addr;
    u32 irqs[] = {FPGA2_IO, ENET_IO, FPGA1_IO, ADS_IO, 0};

    irq_helper_t * alt = (irq_helper_t *)dev_id;

/*	struct irq_desc *irq_desc;
	struct irq_chip *irq_chip = NULL;

	irq_desc = irq_to_desc(GPIO1_MPU_IRQ); //IH_GPIO_BASE);
	if (irq_desc)
		irq_chip = irq_desc->irq_data.chip;*/
    // clear deferred_fiq
    CLEAR_IRQ(irq);
/*    if (alt) {
        generic_handle_irq(gpio_to_irq(alt->irq));
    }*/
#if 1 //0
//bFlag = true;
//    POP_Q(status); //regs.irq_state;

    for (i=0; irqs[i]; i++) {
        irq_num = gpio_to_irq(irqs[i]);
        fiq_count = fiq_irqs[i];
        while (irq_counter[i] != fiq_count) {
        /*mask = 1 << irqs[i];
        if (regs.irq_state & mask) {*/
//            regs.irq_state &= ~mask; //en_mask |= mask;
#if 0
            	if ((irqs[i] != FPGA1_IO) && (irqs[i] != FPGA2_IO)) {
				struct irq_data *d = irq_get_irq_data(irq_num);

				/*
				 * It looks like handle_edge_irq() that
				 * OMAP GPIO edge interrupts default to,
				 * expects interrupt already unmasked.
				 */
				if (irq_chip && irq_chip->irq_unmask)
					irq_chip->irq_unmask(d);
		}
#endif
            generic_handle_irq(irq_num);
            irq_counter[i]++;
        }
//        if ((irqs[i] != FPGA1_IO) && (irqs[i] != FPGA2_IO))
//            writel((1 << irqs[i]), (regs.base_gpio1 + OMAP24XX_GPIO_SETIRQENABLE1));
    }
    /*local_fiq_disable();
    regs.irq_state &= ~en_mask;
    local_fiq_enable();
    //enable_irq(irq_num);
    writel(en_mask, (regs.base_gpio1 + OMAP24XX_GPIO_SETIRQENABLE1));*/
//bFlag = false;
#endif
    return IRQ_HANDLED;
}

/*
 * Open the device.
 */
static int fpga_open(struct inode *inode, struct file *filp)
{
        int id = iminor(inode);
        filp->private_data = (void *)id;
#ifdef USE_SIGS
        fpga[id].ts = get_current();
        debug("Attached %d to pid:%d\n", id, fpga[id].ts->pid);
#endif
	return 0;
}

static inline ssize_t number_of_cols(fpga_info_t * card) {
    int num = 0, tail, head;
    unsigned long flags;
    if (0 != COLAVAIL(card)) {
        spin_lock_irqsave(CHNGMUP(card->lock), flags);
        head = card->head; tail = card->tail; 
        spin_unlock_irqrestore(CHNGMUP(card->lock), flags);
        if (head == tail) num = BUFCOLS - 1;
        else num = ((head + (QSZE - tail)) / COLSZE) % BUFCOLS - 1;
        debug("FPGA cols: %d head:%d, tail:%d\n", num, head, tail);
    } 
    return num;
}

static ssize_t fpga_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
        /////////////////////////////////////////////////////////////
        DECLARE_WAITQUEUE(wait, current);
        unsigned long avail=0, state=0;
        int id;
        ssize_t retval;

        id = ((int)filp->private_data);
        if (!ISVALID_ID(id)) return 0;

        add_wait_queue(&fpga[id].wait, &wait);
        current->state = TASK_INTERRUPTIBLE;
        do {
            avail = number_of_cols(&fpga[id]); /* spin_lock is done inside */
            spin_lock_irq(&fpga[id].lock);
            //if (fpga[id].state) state = fpga[id].state--;
            state = fpga[id].state;
            fpga[id].state = 0;
            spin_unlock_irq(&fpga[id].lock);
            if (0 != avail && 0 != state
                    ) break;
            if (filp->f_flags & O_NONBLOCK) {
                    retval = -EAGAIN;
                    goto out;
            }
            if (signal_pending(current)) {
                    retval = -ERESTARTSYS;
                    goto out;
            }
            schedule();
        } while(1);
        retval = put_user(avail, (unsigned long *)buf);  
        if (!retval)
                retval = sizeof(unsigned long);

out:
        current->state = TASK_RUNNING;
        remove_wait_queue(&fpga[id].wait, &wait);
        return retval;
}

static int fpga_release(struct inode *inode, struct file *filp)
{
        
#ifdef USE_SIGS
    int id = ((int)filp->private_data);
    if (ISVALID_ID(id)) {
        fpga_info_t * card = &fpga[id];
        debug("Released %d from %d\n", card->ts->pid, id);
        card->ts = NULL;
    }
#endif
    return 0;
}

static ssize_t fpga_write(struct file *filp, const char __user *buf, 
        size_t count, loff_t *f_pos)
{
    int i = ((int)filp->private_data); //, x;
    if (ISVALID_ID(i) && (0 != COLAVAIL(&fpga[i])) && 
            count < (COLSZE*sizeof(unsigned long))) {
        fpga_info_t * card = &fpga[i];
        unsigned long * lp = &card->queue[card->tail];
        if (0 != copy_from_user(lp, buf, count))
            return -ENOSPC;
        spin_lock_irq(&card->lock);
        card->tail = NXTNDX(card->tail);
        debug("Wrote col: head: %ld, tail: %ld\n", card->head, card->tail);
        spin_unlock_irq(&card->lock);
        return count;
    }
    return -ENOSPC;
}
int fpga_read_procmem(char *buf, char **start, off_t offset,
                        int count, int *eof, void *data) {
    int i, len = 0;
    int limit = count - 80; /* Don't print more than this */
    len += sprintf(buf+len, "fiq_cntr: %d\n", fiq_cntr);
    for (i=0; i < NUM_FIQS; i++) {
        len += sprintf(buf+len, "fiq[%d]: %d, %d\n", i, 
                                fiq_irqs[i], irq_counter[i]);
    }
    for (i = 0; i < sizeof(fpga)/sizeof(struct fpga_info) && len <= limit; i++) {
        struct fpga_info *f = &fpga[i];
        //if (down_interruptible(&d->sem)) return -ERESTARTSYS;
#define FMT "Card %d 0x%lx %d %d:h %ld,t %ld,cols %d/0x%02x 0x%02x\n"
        len += sprintf(buf+len, FMT, i, f->addr, f->irqstate, f->irq,
                        f->head, f->tail, number_of_cols(f),
                        readreg(f->addr+PRINT_STATUS_ADDR),
                        readreg(f->addr+READ_IO));
        //up(&scull_devices[i].sem);
    }
    *eof = 1;
    return len;
}

#define FPGA_IOC_MAGIC 'f'
#define FPGA_IOQSZE         _IOR(FPGA_IOC_MAGIC, 0, ssize_t)
#define FPGA_IRQ            _IOW(FPGA_IOC_MAGIC, 1, ssize_t)
#define FPGA_CLEARBUFFER    _IOW(FPGA_IOC_MAGIC, 2, ssize_t)
/*static int fpga_ioctl(struct inode *nde, struct file *filp, unsigned int num, unsigned long param) {*/
static long fpga_ioctl(struct file *filp, unsigned int num, unsigned long param) {
    ssize_t avail = 0;
    int retval = -EINVAL;
    int id = ((int)filp->private_data);
    switch (num) {
    case FPGA_IOQSZE:
        avail = number_of_cols(&fpga[id]);
        retval = put_user(avail, (ssize_t *)param);
        break;
    case FPGA_IRQ:
        fpga[id].irqstate = param;
        break;
    case FPGA_CLEARBUFFER:
        fpga[id].tail = NXTNDX(fpga[id].head);
        break;
    }
    return retval;
}

static struct file_operations fops = {
    .read       = fpga_read, 
    .write      = fpga_write,
    .open       = fpga_open,
    .release    = fpga_release,
    .unlocked_ioctl      = fpga_ioctl,
//    .ioctl      = fpga_ioctl,
    .owner      = THIS_MODULE
};
static struct fiq_handler fh = {
    .name = MODULE_NAME "-fiq",
};


int32_t timer_irq;		    // timer's corresponding irq number
void enable_fiq_loc(int irq) 
{
    // following the documentation on the TI omap 35xx Tech Ref manual, this 
    // is the fiq initialization sequence respect to the irq number.
    // Apparently the fiq api does not perform this. Have to double check
	u32 mir1, mask, addr;

        printk("inside %s\n", __FUNCTION__);
        addr = (u32)(regs.base_intcps + INTCPS_ILR + (irq << 2));
	printk("fiq-timer3: enable_fiq: will set mem = 0x%x\n", (u32)addr);
	printk("fiq-timer3: ILR: 0x%x b4\n", readl(addr));
	writel(1, addr);
	printk("fiq-timer3: ILR: 0x%x after\n", readl(addr));

        addr = (u32)(regs.base_intcps + INTCPS_ITR(1));
	printk("fiq-timer3: ITR: 0x%x\n", readl(addr));
        //n = 0x20 * (irq / 32);
	addr = (u32)(regs.base_intcps + INTCPS_MIR(1)); // + 1;
	printk("fiq-timer3: MIR @ 0x%x\n", addr);
	mir1 = readl(addr); //base_intcps + INTCPS_MIR0 + 1);
	printk("fiq-timer3: MIR irq=%d = 0x%x\n", irq, mir1);
//	printk("fiq-timer3: ~31 = 0x%x\n",(~31) );
//	printk("fiq-timer3: (irq & ~31)  = 0x%x\n",( irq & ~31) );
//	printk("fiq-timer3: (irq - (irq & ~31) ) = 0x%x\n",(irq - (irq & ~31) ));
//	printk("fiq-timer3: ( 1 << (irq - (irq & ~31)) = 0x%x\n",  ( 1 << (irq - (irq & ~31))) );
//	printk("fiq-timer3: ~( 1 << (irq - (irq & ~31)) = 0x%x\n",~( 1 << (irq - (irq & ~31))) );
        mask = 1 << (irq - (irq & ~31));
	mir1 = mir1 & ~(mask);
	printk("fiq-timer3: new mir1 = 0x%x\n",mir1);
	printk("fiq-timer3: mask = 0x%x\n", mask);
	printk("fiq-timer3: MIR1 = 0x%x b4\n", readl(addr));
	writel(mask, regs.base_intcps + INTCPS_MIR_CLEAR(1)); //base_intcps + INTCPS_MIR0 + 1);
	printk("fiq-timer3: MIR1 = 0x%x after\n", readl(addr));

//	memset((void *) (MPU_INTC+INTCPS_ILR + (irq << 2)),1,1);
        printk("leaving %s\n", __FUNCTION__);
}

static irqreturn_t dummy_irq(int irq, void *dev_id) //, struct pt_regs *regs)
{
    printk("Should'nt have gotten here!\n");
    return IRQ_HANDLED;
}

/* defined in assembly file */
extern char fiq_handler_start, fiq_handler_end;
//#define fiq_handler_start xfiq_handler_start
//#define fiq_handler_end xfiq_handler_end
#define TEST_IO_PIN TEST_POINT //(186)
static struct class *fpga_class;
static u8 u8aFiqStack[4096]; //1024];
static int __devinit
fpga_probe(struct platform_device *pdev)
{
    int ret, i, irqflags = IRQF_SHARED|IRQF_TRIGGER_FALLING;
	int rc = 0/*,i*/;		// return code and auz variable
    const int sze = QSZE * sizeof(unsigned long)+1;
    void *fiqhandler_start;
    unsigned int fiqhandler_length;
    struct pt_regs FIQ_regs;
    struct resource *addr_res;
    struct resource *irq_res;
	char *label = "timer1";	// label for the gpio_request identification
	unsigned long base;	// base value to call ioremap
    unsigned long addr;

    printk(KERN_INFO MODULE_NAME " driver probe\n");

    for (i=0; i < NUM_FIQS; i++) fiq_irqs[i] = irq_counter[i] = 0;
    /*
     * Check the size of the FIQ,
     * it cannot go beyond 0xffff0200, and is copied to 0xffff001c
     */
    fiqhandler_start = &fiq_handler_start;
    fiqhandler_length = &fiq_handler_end - &fiq_handler_start;

    if ((0x200 - 0x1c) < fiqhandler_length) {
        dev_err(&pdev->dev, "FIQ handler is to large 0x%04x > 0x%04x\n", 
                fiqhandler_length, (0x200 - 0x1c));
        return 0;
    }

    fpga_class = class_create(THIS_MODULE, "fpga");
    if (IS_ERR(fpga_class)) {
	printk(KERN_WARNING "Unable to create fpga class; "
		       "errno = %ld\n", PTR_ERR(fpga_class));
	fpga_class = NULL;
    }

        /////////////////////////////////
	//init gpio output on gpio 186 
        rc = gpio_request( TEST_IO_PIN, label );
	rc = gpio_direction_output( TEST_IO_PIN, 1 );
        gpio_set_value( TEST_IO_PIN, 0);
	// determine iremaped addresses of physical register bases
	base = GPIO_BASE_GPIO1;  // base de gpio registers
	regs.base_gpio1 = ioremap(base, SZ_4K);
        printk(MODULE_NAME " 0x%x\n", readl(regs.base_gpio1+OMAP24XX_GPIO_IRQENABLE1));
	base = GPIO_BASE_GPIO5;  // base de gpio registers
//	regs.base_gpio5 = ioremap(base, SZ_4K);

	base = INTCPS_BASE; // base de interrupt ctrlr registers
	regs.base_intcps = ioremap(base,SZ_4K);
        /////////////////////////////////

    ret = register_chrdev(MAJORNUM, MODULE_NAME, &fops);
    if (ret < 0) {
	    printk(KERN_INFO MODULE_NAME ": unable to register\n");
    	return ret;
    }
    devfs_mk_dir(MODULE_NAME);

//    FIQ_regs.uregs[fiq_mreg] = (u32)&regs; //base_gpio5; //(long)(&ISR(3));
//    FIQ_regs.ARM_r8 = (u32)regs.base_intcps;
//    FIQ_regs.ARM_r9 = (u32)regs.base_gpio1;
    FIQ_regs.ARM_r10 = (u32)&fiq_handler;
//    FIQ_regs.uregs[fiq_rirq] = (u32)base_intcps; //(long)(&IMX_INTFRCH);
//    FIQ_regs.uregs[fiq_rgpio1]	= (u32)base_gpio1;
    FIQ_regs.ARM_sp = (long)u8aFiqStack + sizeof(u8aFiqStack) - 4;

    if (claim_fiq(&fh)) {
        printk("couldn't claim fiq\n");
        return 0;
    }
    ret = request_irq(FIRST_DEFERRED_FIQ, deferred_irq,
                    IRQ_TYPE_EDGE_RISING, "deferred_fiq", 0);
    if (ret < 0) {
            pr_err("Failed to get deferred_irq IRQ, ret=%d\n", ret);
            release_fiq(&fh);
            return 0;
    }
    dev_dbg(&pdev->dev, "Stack installed @0x%08lx len=x%x\n", 
            FIQ_regs.ARM_sp, fiqhandler_length);
    // make deferred_fiq irq edge triggered ??? from ams-delta
    for (i=0; i < 2; i++) {
        fpga[i].queue = kmalloc(sze, GFP_KERNEL);
        memset(fpga[i].queue, 0, sze);
        addr_res = platform_get_resource(pdev, IORESOURCE_MEM, i); //0x200 + 0x20*i;
        irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, i);

        if ((NULL == addr_res) || (NULL == irq_res)) {
            dev_err(&pdev->dev, "insufficient resources\n");
            ret = -ENOENT;
            goto out;
        }
        fpga[i].addr = (unsigned long)ioremap(addr_res->start, resource_size(addr_res));
        fpga[i].irq = irq_res->start;
        fpga[i].soft_irq = irq_res->end;
        dev_dbg(&pdev->dev, "install %d(0x%08x@0x%08lx irq:%d\n", 
                i, addr_res->start, fpga[i].addr, OMAP_GPIO_IRQ(fpga[i].irq));
        irqflags = irq_res->flags & IRQF_TRIGGER_MASK;
        irqflags |= IRQF_SHARED;
        // clear fpga this is done so fpga is in known state
        // otherwise soft reboot may cause problems
        dev_dbg(&pdev->dev, "Clearing fpga... @0x%08x\n", addr_res->start);
        addr = (unsigned long)ioremap(addr_res->start + GAL_REG, SZ_16);
        writereg(0xff, addr);
        writereg(0, addr);
        writereg(0xff, addr);
        dev_dbg(&pdev->dev, "Status fpga... @0x%08x x%x\n", addr_res->start, readreg(addr));
        iounmap((void *)addr);

        device_create(fpga_class, NULL, MKDEV(MAJORNUM, MINORNUM+i), NULL, "fpga%u", i);
        devfs_mk_cdev(MKDEV(MAJORNUM, i), S_IFCHR|S_IRUSR|S_IWUSR, 
                    MODULE_NAME "/%d", i);
        if (request_irq(OMAP_GPIO_IRQ(fpga[i].irq), fpga_interrupt/*dummy_irq*/, irqflags, irq_name[i]/*""*/, &fpga[i]))
            printk(KERN_WARNING "fpga: Can't get IRQ: %d!\n", OMAP_GPIO_IRQ(fpga[i].irq));
/*        if (request_irq(fpga[i].soft_irq, fpga_interrupt, irqflags, irq_name[i], 
                            &fpga[i]))
            printk(KERN_WARNING "fpga: Can't get soft IRQ: %d!\n", fpga[i].soft_irq);*/
        //else
        //    set_irq_type(IRQS[i], IRQT_LOW);
    }


    // Now setup the remainder of soft irqs
    /*irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
    if (NULL != irq_res) {
        alt_irqs.irq = irq_res->start;
        alt_irqs.soft_irq = irq_res->end;
        if (request_irq(alt_irqs.soft_irq, deferred_irq, irqflags, "soft enet", &alt_irqs))
            printk(KERN_WARNING "soft enet irq cannot be obtained: %d\n", alt_irqs.irq);
    } else 
        printk(KERN_WARNING "Unable to locate extra irq resources\n");*/
    // set the IRQ to FIQ level
    IMX_INTTYPEH  |= (1 << (GPIO_INT_PORTD - 32));
    imx_gpio_mode(GPIO_PORTD|GPIO_IN|GPIO_GIUS|GPIO_PUEN|IRQ_11_PD8);
    imx_gpio_mode(GPIO_PORTD|GPIO_IN|GPIO_GIUS|GPIO_PUEN|IRQ_10_PD9);
    printk("set_fiq_regs()\n");
   set_fiq_regs(&FIQ_regs);
    printk("set_fiq_handler()\n");
    set_fiq_handler(fiqhandler_start, fiqhandler_length);
	timer_irq = GPIO1_MPU_IRQ; //OMAP_GPIO_IRQ(26); omap_dm_timer_get_irq(lock.timer_ptr);
    printk("enable_fiq(x%i)\n", timer_irq);
    enable_fiq(timer_irq); //GPIO_INT_PORTD);

    printk("enable_fiq_loc()\n");
	enable_fiq_loc(timer_irq);
    printk("done enable_fiq_loc()\n");
    ICR1(3) &= ~(0x000f0000); // clear the bits
    ICR1(3) |= 0x00050000; //0x000f0000;
    IMR(3)  |= 0x00000300;
    create_proc_read_entry("fpga", 0 /* default mode */,
        NULL /* parent dir */, fpga_read_procmem,
        NULL /* client data */);

    return 0;

out:
    release_fiq(&fh);
    return ret;
}

static int __devexit
fpga_drv_remove(struct platform_device *pdev)
{
        int i;
        disable_fiq(GPIO_INT_PORTD);
        release_fiq(&fh);
        IMR(3) &= ~(0x00000300);

        devfs_remove(MODULE_NAME); // this will go away eventually
        unregister_chrdev(MAJORNUM, MODULE_NAME);
        for (i=0; i < 2; i++) {
            devfs_remove(MODULE_NAME "/%d", i);
            free_irq(OMAP_GPIO_IRQ(fpga[i].irq), &fpga[i]);
//            free_irq(fpga[i].soft_irq, &fpga[i]);
        }
        return 0;
}

static struct platform_driver fpga_driver = {
	.driver	= {
		.name    = "fpga",
		.owner	 = THIS_MODULE,
		//.pm	 = &dm9000_drv_pm_ops,
	},
	.probe   = fpga_probe,
	.remove  = __devexit_p(fpga_drv_remove),
};

static int __init
fpga_init(void)
{
    printk(KERN_INFO "%s driver, v%s\n", MODULE_NAME, DRV_VERSION);
    return platform_driver_register(&fpga_driver);
}

static void __exit
fpga_cleanup(void)
{
    printk(KERN_INFO "cleanup %s driver\n", MODULE_NAME);
    platform_driver_unregister(&fpga_driver);
}

MODULE_AUTHOR("C nauman <cnauman at diagraph dot com>");
MODULE_DESCRIPTION("fpga /dev entries driver");
MODULE_LICENSE("GPL");
module_init(fpga_init);
module_exit(fpga_cleanup);

