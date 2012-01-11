/*
 * fram.c
 *
 * $Id: fram.c,v 1.1 2006/02/13 09:52:32 andrzej Exp $
 *
 * MTD driver for 24xx series I2C fram. 
 * Derived from sample found on net from Ekiert
 * Derived from the MTD test driver (mtdram.c).
 *
 *
 * Copyright (C) 2005 Ekiert sp z o.o.
 * Copyright (C) 2012 C Nauman <cnauman@diagraph.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version. 
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/vmalloc.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#define DBG(a,...)
//#define DBG printk
#ifdef MODULE
static unsigned long total_size = CONFIG_MTD_FRAM_SIZE;
module_param(total_size, ulong, 0);
MODULE_PARM_DESC(total_size, "Total device size in bytes");
#define FRAM24_TOTAL_SIZE (total_size)
#else
#define FRAM24_TOTAL_SIZE CONFIG_MTD_FRAM_SIZE
#endif //MODULE

#define CHUNK_SZE	(512) /*(128)*/
/* Probe at 0x50 only */
static unsigned short normal_i2c[] = { 0x50, I2C_CLIENT_END };

/*
 * I2C driver data
 */

/*
 * Client data (each client gets its own)
 */
struct fram24_data {
	struct i2c_client client;
	struct list_head list;
	struct mtd_info mtd_info;
	int id;
};

/*
 * Internal variables
 */
static LIST_HEAD(fram24_clients);

/*
 * If we're to write 'count' bytes, to address 'addr', return the number 
 * of bytes that will fit within current page. 
 */
static size_t fram24_bytes_to_eop(size_t addr, size_t count)
{
	size_t to_eop = CHUNK_SZE - (addr % CHUNK_SZE);
	return (count < to_eop) ? count : to_eop;
}

/*
 * Erase implemented as no-op. There is no need to erase before write. 
 */
static int fram24_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	if (instr->addr + instr->len > mtd->size) {
		return -EINVAL;
	}

	/* Do nothing */
	
	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/*
 * Read len bytes from the eeprom to buf. 
 */
static int fram24_read(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	struct i2c_client *client;
	struct i2c_msg msg[2];
        uint8_t addr[2];

	DBG("fram24_read(from:%ld, len:%ld)\n", (long)from, (long)len);

	if (from >= mtd->size) {
		return -EINVAL;
	}
        if (len > (mtd->size - from))
            len = mtd->size - from;

	client = (struct i2c_client *)mtd->priv;
	
	/* msg[0]: write two byte address */
        addr[0] = from >> 8;
        addr[1] = from;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = addr;

	/* msg[1]: read 'len' bytes */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	if (i2c_transfer(client->adapter, &msg[0], 2)!=2) {
		*retlen = 0;
		return -EIO;
	}

	*retlen = len;
	return 0;
}

/*
 * Write len bytes from buf to the eeprom. 
 * 
 * Do not write chunks, that are not changed. We could do more, and check
 * every single byte, but that would severly affect performance (already
 * poor). 
 */
static int fram24_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	u_char writebuf[CHUNK_SZE+2];
	struct i2c_client *client;
	struct i2c_msg msg[2];

        uint16_t addr;
	size_t wrote, remain, chunk;

	DBG("fram24_write(pos:%ld, len:%ld)\n", (long)to, (long)len);
	
	if (to + len > mtd->size) {
		return -EINVAL;
	}

	client = (struct i2c_client *)mtd->priv;
	wrote = 0;
	remain = len;
        addr = to;

	while (remain) {
		chunk = fram24_bytes_to_eop(addr, remain);
		if (1) { //memcmp(readbuf, (char*)&buf[wrote], chunk)!=0) {
			DBG("Doing write %d @ 0x%04x \n", chunk, addr);
			/* msg[0]: write two byte address */
			msg[0].addr = client->addr;
			msg[0].flags = 0;
			msg[0].len = 2 + chunk;
			writebuf[0] = addr >> 8;
	        	writebuf[1] = addr;
			memcpy(writebuf+2, &buf[wrote], chunk);
			msg[0].buf = writebuf;

			if (i2c_transfer(client->adapter, msg, 1) != 1) {
				*retlen = wrote;
				return -EIO;
			}
		} else {
			DBG("Write not neccessary\n");
		}

		remain -= chunk;
		addr += chunk;
		wrote += chunk;

		/* sorry about this, we should poll ACK */
		msleep(5);
	}
	*retlen=wrote;
	return 0;
}

/*
 * Detach a client from the driver. 
 * Deregister an associated MTD device. 
 */
static int fram24_detach_client(struct i2c_client *client)
{
	struct fram24_data *data = i2c_get_clientdata(client);
	struct mtd_info *mtd = &(data->mtd_info);

	mtd_device_unregister(mtd); //del_mtd_device(mtd);
        
	list_del(&data->list);
	kfree(data);
	return 0;
}

static int fram24_point(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, void **virt, resource_size_t *phys)
{
	if (from + len > mtd->size)
		return -EINVAL;

	/* can we return a physical address with this driver? */
	if (phys)
		return -EINVAL;

	*virt = mtd->priv + from;
	*retlen = len;
	return 0;
}

static void fram24_unpoint(struct mtd_info *mtd, loff_t from, size_t len)
{
}

/*
 * We'd init on-chip registers here, if there were any ;-)
 * Register an associated MTD device. 
 */
static int fram24_init_client(struct i2c_client *client)
{
	struct fram24_data *data = i2c_get_clientdata(client);
	struct mtd_info *mtd = &(data->mtd_info);

	memset(mtd, 0, sizeof(data->mtd_info));

	/* Setup the MTD structure */
	mtd->name = "I2C FRAM";
	mtd->type = MTD_RAM;
	mtd->flags = MTD_WRITEABLE;
	mtd->size = FRAM24_TOTAL_SIZE;
	mtd->erasesize = PAGE_SIZE; /*CHUNK_SZE;*/
	mtd->priv = client;

	mtd->owner = THIS_MODULE;
	mtd->erase = fram24_erase;
	mtd->read = fram24_read;
	mtd->write = fram24_write;

        mtd->writesize = 1;
        mtd->point = fram24_point;
        mtd->unpoint = fram24_unpoint;

	if (mtd_device_register(mtd, NULL, 0))
        {
		return -EIO;
	}

	return 0;
}

static int fram24_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

         if (!(adapter->class & I2C_CLASS_SPD) && client->addr != normal_i2c[0])
                 return -ENODEV;
 
         if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)
          && !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
                 return -ENODEV;

	/* We can fill in the remaining client fields */
	strlcpy(info->type, "fram24", I2C_NAME_SIZE);

	return 0;
}
/*
 * Detect if a chip is there, if so register the chip. 
 * There isn't much we can to do to detect an EEPROM. 
 */
static int fram24_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fram24_data *data;
	int err = 0;
        struct i2c_msg msg;
        uint8_t buf[2] = {0, 0};

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;

        pr_info("Probe for i2c fram...");
	if ((err = i2c_transfer(client->adapter, &msg, 1)) != 1) {
		pr_info("Cannot find i2c fram. (%d)\n", err);
		return -ENODEV;
	} 

	pr_info("Found i2c fram.\n");

	if (!(data = kmalloc(sizeof(struct fram24_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct fram24_data));
        i2c_set_clientdata(client, data);

	INIT_LIST_HEAD(&data->list);

	/* Initialize the eeprom specific part */
	if ((err = fram24_init_client(client)) != 0) {
		goto exit_free;
	}

	/* Add client to local list */
	list_add(&data->list, &fram24_clients);

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}


static const struct i2c_device_id fram24_id[] = {
         { "fram24", 0 },
         { }
};

static struct i2c_driver fram24_driver = {
        .class		= I2C_CLASS_SPD,
	.driver = {
		.owner	= THIS_MODULE,
            .name		= "fram24",
        },
        .probe		= fram24_probe,
        .remove		= fram24_detach_client,
        .id_table	= fram24_id,
	.detect		= fram24_detect,
	.address_list	= normal_i2c,
};

/*
 * Initialize the driver. 
 */

static int __init init_fram24(void)
{
	return i2c_add_driver(&fram24_driver);
}

/*
 * Cleanup and unregister the driver. 
 */
static void __exit cleanup_fram24(void)
{
	i2c_del_driver(&fram24_driver);
}

//module_init(init_fram24);
late_initcall(init_fram24);
module_exit(cleanup_fram24);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrzej Ekiert <a.ekiert@ekiert.com> and "
        "C Nauman <cnauman@diagraph.com>");
MODULE_DESCRIPTION("24xx FRAM driver");

