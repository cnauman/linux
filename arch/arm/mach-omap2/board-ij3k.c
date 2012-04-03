/*
 * board-ij3k.c - TimLL Devkit8000
 *
 * Copyright (C) 2009 Kim Botherway
 * Copyright (C) 2010 Thomas Weber
 *
 * Modified from mach-omap2/board-omap3beagle.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <mach/id.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

#include <plat/mcspi.h>
#include <linux/input/matrix_keypad.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/dm9000.h>
#include <linux/interrupt.h>

#include "sdram-micron-mt46h32m32lf-6.h"

#include "mux.h"
#include "hsmmc.h"
#include "common-board-devices.h"

/* Reserved omap irqs 6, 8 35, 49-52, 53*, 64, 67-71, 79, 84-85, 95
 * (53 is shared with iva2.2 controller
 */
#define OMAP_FPGA1_GPIO_IRQ	24
#define OMAP_DM9000_GPIO_IRQ	25
#define OMAP_FPGA2_GPIO_IRQ	26
#define OMAP3_DEVKIT_TS_GPIO	27

#define OMAP_FPGA1_SOFT_IRQ	67
#define OMAP_DM9000_SOFT_IRQ	68
#define OMAP_FPGA2_SOFT_IRQ	69
#define OMAP3_TS_SOFT_IRQ	70

static struct mtd_partition ij3k_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Upgrade storage",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x2a0000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x6a0000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= 29,
	},
	{}	/* Terminator */
};

static int ij3k_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value_cansleep(dssdev->reset_gpio, 0);
	return 0;
}

static void ij3k_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value_cansleep(dssdev->reset_gpio, 1);
}

/*static int ij3k_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value_cansleep(dssdev->reset_gpio, 1);
	return 0;
}

static void ij3k_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value_cansleep(dssdev->reset_gpio, 0);
}*/

static struct regulator_consumer_supply ij3k_vmmc1_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

/* ads7846 on SPI */
static struct regulator_consumer_supply ij3k_vio_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi2.0"),
};

static struct panel_generic_dpi_data lcd_panel = {
	.name			= "generic",
	.platform_enable        = ij3k_panel_enable_lcd,
	.platform_disable       = ij3k_panel_disable_lcd,
};

static struct omap_dss_device ij3k_lcd_device = {
	.name                   = "lcd",
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.driver_name            = "generic_dpi_panel",
	.data			= &lcd_panel,
	.phy.dpi.data_lines     = 24,
};

static struct panel_generic_dpi_data dvi_panel = {
	.name			= "optrex",
	.platform_enable        = ij3k_panel_enable_lcd,
	.platform_disable       = ij3k_panel_disable_lcd,
	//.platform_enable        = ij3k_panel_enable_dvi,
	//.platform_disable       = ij3k_panel_disable_dvi,
};
static struct omap_dss_device ij3k_dvi_device = {
	.name                   = "stn",
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.driver_name            = "generic_dpi_panel",
	.data			= &dvi_panel,
	.phy.dpi.data_lines     = 16,
        /*.panel.config           = OMAP_DSS_LCD_IVS |
				  OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IEO,*/
        /*.panel.timings          = 
        {
            .x_res    = 320,
            .y_res    = 240,
            .pixel_clock = 2500,
        }, */
};

/*static struct omap_dss_device ij3k_tv_device = {
	.name                   = "tv",
	.driver_name            = "venc",
	.type                   = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type          = OMAP_DSS_VENC_TYPE_SVIDEO,
};*/


static struct omap_dss_device *ij3k_dss_devices[] = {
	&ij3k_lcd_device,
	&ij3k_dvi_device,
//	&ij3k_tv_device,
};

static struct omap_dss_board_info ij3k_dss_data = {
	.num_devices = ARRAY_SIZE(ij3k_dss_devices),
	.devices = ij3k_dss_devices,
	.default_device = &ij3k_lcd_device,
};

/*static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_1),
	KEY(1, 0, KEY_2),
	KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4),
	KEY(1, 1, KEY_5),
	KEY(2, 1, KEY_6),
	KEY(3, 1, KEY_F5),
	KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8),
	KEY(2, 2, KEY_9),
	KEY(3, 2, KEY_F6),
	KEY(0, 3, KEY_F7),
	KEY(1, 3, KEY_0),
	KEY(2, 3, KEY_F8),
	PERSISTENT_KEY(4, 5),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(5, 5, KEY_VOLUMEDOWN),
	0
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data ij3k_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 6,
	.cols		= 6,
	.rep		= 1,
};*/

static struct gpio_led gpio_leds[];

#define SYS_LED3 (163)
static void omap_led_init(void) {
    int ret = 0;
	ret = gpio_request_one(SYS_LED3, GPIOF_OUT_INIT_HIGH, "led2");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO for led2\n");
	}
}

static int ij3k_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	int ret;

	omap_mux_init_gpio(29, OMAP_PIN_INPUT);
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[1].gpio = gpio + TWL4030_GPIO_MAX + 0; /* kb_caps */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1; /* kb_pwr */

	/* TWL4030_GPIO_MAX + 0 is "LCD_PWREN" (out, active high) */
/*	ij3k_lcd_device.reset_gpio = gpio + TWL4030_GPIO_MAX + 0;
	ret = gpio_request_one(ij3k_lcd_device.reset_gpio,
			       GPIOF_OUT_INIT_LOW, "KBD_PWREN");
	if (ret < 0) {
		ij3k_lcd_device.reset_gpio = -EINVAL;
		printk(KERN_ERR "Failed to request GPIO for LCD_PWRN\n");
	}*/

	/* gpio + 7 is "DVI_PD" (out, active low) */
	/*ij3k_dvi_device.reset_gpio = gpio + 7;
	ret = gpio_request_one(ij3k_dvi_device.reset_gpio,
			       GPIOF_OUT_INIT_LOW, "DVI PowerDown");
	if (ret < 0) {
		ij3k_dvi_device.reset_gpio = -EINVAL;
		printk(KERN_ERR "Failed to request GPIO for DVI PowerDown\n");
	}*/

	return 0;
}

static struct twl4030_gpio_platform_data ij3k_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pulldowns	= BIT(1) | BIT(2) | BIT(6) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= ij3k_twl_gpio_setup,
};

static struct regulator_consumer_supply ij3k_vpll1_supplies[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data ij3k_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(ij3k_vmmc1_supply),
	.consumer_supplies	= ij3k_vmmc1_supply,
};

/* VPLL1 for digital video outputs */
static struct regulator_init_data ij3k_vpll1 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(ij3k_vpll1_supplies),
	.consumer_supplies	= ij3k_vpll1_supplies,
};

/* VAUX4 for ads7846 and nubs */
static struct regulator_init_data ij3k_vio = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(ij3k_vio_supply),
	.consumer_supplies      = ij3k_vio_supply,
};

static struct twl4030_platform_data ij3k_twldata = {
	/* platform_data for children goes here */
	.gpio		= &ij3k_gpio_data,
	.vmmc1		= &ij3k_vmmc1,
	.vpll1		= &ij3k_vpll1,
	.vio		= &ij3k_vio,
	/*.keypad		= &ij3k_kp_data,*/
};

static struct i2c_board_info __initdata i2c_board_info = {
        I2C_BOARD_INFO("fram24", 0x50),
};
static int __init ij3k_i2c_init(void)
{
	omap3_pmic_get_config(&ij3k_twldata,
			  TWL_COMMON_PDATA_USB | TWL_COMMON_PDATA_AUDIO,
			  TWL_COMMON_REGULATOR_VDAC);
	omap3_pmic_init("tps65930", &ij3k_twldata);
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	//omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(2, 1000, &i2c_board_info, 1);
	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "led1",
		.default_trigger	= "heartbeat",
		.gpio			= 186,
		.active_low		= true,
	},
	{
		.name			= "kb_pwr",
		.default_trigger	= "none",
		.gpio			= -1,
		.active_low		= true,
	},
	{
		.name			= "kb_caps",
		.default_trigger	= "none",
		.gpio			= -1,
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

/*static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 26,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct:w
platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};*/
//////////////////////////////////////////////////////////////
/*
 * ij Keyboard Device
 */
#define IJ_OFF	KEY_F11 //KEY_POWER2
#define IJ_ON	KEY_F12 //KEY_POWER

// KEY(row, col, val)
static const uint32_t ij3k_kbd_keymap[] = {
	KEY(0, 0, KEY_RESERVED),
	KEY(1, 0, KEY_F2),
	KEY(2, 0, KEY_F1),
	KEY(3, 0, KEY_F3),
	KEY(4, 0, KEY_F4),
	KEY(5, 0, KEY_RESERVED),
	KEY(6, 0, IJ_ON),
	KEY(7, 0, IJ_OFF),

	KEY(0, 1, KEY_8),
	KEY(1, 1, KEY_9),
	KEY(2, 1, KEY_RESERVED),
	KEY(3, 1, KEY_0),
	KEY(4, 1, KEY_EQUAL),
	KEY(5, 1, KEY_MINUS),
	KEY(6, 1, KEY_RESERVED),
	KEY(7, 1, KEY_BACKSPACE),

	KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_6),
	KEY(2, 2, KEY_4),
	KEY(3, 2, KEY_5),
	KEY(4, 2, KEY_2),
	KEY(5, 2, KEY_3),
	KEY(6, 2, KEY_1),
	KEY(7, 2, KEY_ESC),

	KEY(0, 3, KEY_I),
	KEY(1, 3, KEY_O),
	KEY(2, 3, KEY_LEFT),
	KEY(3, 3, KEY_SPACE),
	KEY(4, 3, KEY_LEFTBRACE),
	KEY(5, 3, KEY_P),
	KEY(6, 3, KEY_RIGHTBRACE),
	KEY(7, 3, KEY_BACKSLASH),

	KEY(0, 4, KEY_U),
	KEY(1, 4, KEY_Y),
	KEY(2, 4, KEY_R),
	KEY(3, 4, KEY_T),
	KEY(4, 4, KEY_W),
	KEY(7, 4, KEY_TAB),
	KEY(6, 4, KEY_Q),
	KEY(5, 4, KEY_E),

	KEY(0, 5, KEY_K),
	KEY(1, 5, KEY_L),
	KEY(2, 5, KEY_SEMICOLON),
	KEY(3, 5, KEY_RESERVED),
	KEY(4, 5, KEY_RESERVED),
	KEY(5, 5, KEY_APOSTROPHE),
	KEY(6, 5, KEY_RESERVED),
	KEY(7, 5, KEY_ENTER),

	KEY(0, 6, KEY_J),
	KEY(1, 6, KEY_H),
	KEY(2, 6, KEY_F),
	KEY(3, 6, KEY_G),
	KEY(4, 6, KEY_S),
	KEY(5, 6, KEY_D),
	KEY(6, 6, KEY_A),
	KEY(7, 6, KEY_CAPSLOCK),

	KEY(0, 7, KEY_M),
	KEY(1, 7, KEY_COMMA),
	KEY(2, 7, KEY_SLASH),
	KEY(3, 7, KEY_DOT),
	KEY(4, 7, KEY_DOWN),
	KEY(5, 7, KEY_UP),
	KEY(6, 7, KEY_RIGHT),
	KEY(7, 7, KEY_RESERVED),

	KEY(0, 8, KEY_N),
	KEY(1, 8, KEY_B),
	KEY(2, 8, KEY_C),
	KEY(3, 8, KEY_V),
	KEY(4, 8, KEY_Z),
	KEY(5, 8, KEY_X),
	KEY(6, 8, KEY_RESERVED),
	KEY(7, 8, KEY_RESERVED),

	KEY(0, 9, KEY_RESERVED),
	KEY(1, 9, KEY_RESERVED),
	KEY(2, 9, KEY_LEFTALT),
	KEY(3, 9, KEY_DELETE),
	KEY(4, 9, KEY_LEFTCTRL),
	KEY(5, 9, KEY_LEFTMETA),
	KEY(6, 9, KEY_LEFTSHIFT),
	KEY(7, 9, KEY_RIGHTSHIFT),
};

static struct matrix_keymap_data ij3k_kbd_keymap_data = {
	.keymap		= ij3k_kbd_keymap,
	.keymap_size	= ARRAY_SIZE(ij3k_kbd_keymap),
};

static const int omap_kbd_row_gpios[] = { 
                     99, // row 0
                    100, // row 1
                    101, // row 2
                    102, // row 3
                    103, // row 4
                    104, // row 5
                    105, // row 6
                    106, // row 7
                };

static const int omap_kbd_col_gpios[] = { 
                     96, // col 0
                    111, // col 1
                     97, // col 2
                     95, // col 3
                     94, // col 4
                     98, // col 5
                    167, // col 6
                    126, // col 7
                    109, // col 8
                    110, // col 9
                };

static struct matrix_keypad_platform_data ij3k_kbd_pdata = {
	.keymap_data		= &ij3k_kbd_keymap_data,
	.row_gpios		= omap_kbd_row_gpios,
	.col_gpios		= omap_kbd_col_gpios,
	.num_row_gpios		= ARRAY_SIZE(omap_kbd_row_gpios),
	.num_col_gpios		= ARRAY_SIZE(omap_kbd_col_gpios),
	.col_scan_delay_us	= 10,
	.debounce_ms		= 10,
        .active_low             = 1,
//	.wakeup			= 1,
};

static struct platform_device omap_kbd_dev = {
	.name		= "matrix-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &ij3k_kbd_pdata,
	},
};
#if 0
static int __init lcd_set_displaytype(char *str)
{
    printk("Parsing: %s\n", str);
	int stn_flag = simple_strtol(str, NULL, 0);
#if 0
	switch (disp_type) {
	case MTYPE_STN320x240:
		cmx2xx_display = &generic_stn_320x240;
		break;
	case MTYPE_TFT640x480:
		cmx2xx_display = &generic_tft_640x480;
		break;
	case MTYPE_CRT640x480:
		cmx2xx_display = &generic_crt_640x480;
		break;
	case MTYPE_CRT800x600:
		cmx2xx_display = &generic_crt_800x600;
		break;
	case MTYPE_TFT320x240:
		cmx2xx_display = &generic_tft_320x240;
		break;
	case MTYPE_STN640x480:
		cmx2xx_display = &generic_stn_640x480;
		break;
	default: /* fallback to CRT 640x480 */
		cmx2xx_display = &generic_crt_640x480;
		break;
	}
#endif
        if (stn_flag) {
            printk("Changing to 8-bit cstn display\n");
	    ij3k_lcd_device.phy.dpi.data_lines     = 8;
            ij3k_lcd_device.panel.config &= ~OMAP_DSS_LCD_TFT;
        }
	return 1;
}

__setup("stn_lcd=", lcd_set_displaytype);
#endif
static void __init ij3k_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
}

static void __init ij3k_init_irq(void)
{
	omap3_init_irq();
}

#define PC104_BASE	        0x2c000000
#define OMAP_DM9000_BASE	((PC104_BASE) + 0x300)

static struct resource omap_dm9000_resources[] = {
	[0] = {
		.start		= OMAP_DM9000_BASE,
		.end		= (OMAP_DM9000_BASE + 0x4 - 1),
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= (OMAP_DM9000_BASE + 0x4),
		.end		= (OMAP_DM9000_BASE + 0x4 + 0x4 - 1),
		.flags		= IORESOURCE_MEM,
	},
	[2] = {
		.start		= OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),
		.flags		= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};

static struct dm9000_plat_data omap_dm9000_platdata = {
	.flags = DM9000_PLATF_16BITONLY,
};

static struct platform_device omap_dm9000_dev = {
	.name = "dm9000",
	.id = -1,
	.num_resources	= ARRAY_SIZE(omap_dm9000_resources),
	.resource	= omap_dm9000_resources,
	.dev = {
		.platform_data = &omap_dm9000_platdata,
	},
};

static void __init omap_dm9000_init(void)
{
	/*unsigned char *eth_addr = omap_dm9000_platdata.dev_addr;*/
	struct omap_die_id odi;
	int ret;

	ret = gpio_request_one(OMAP_DM9000_GPIO_IRQ, GPIOF_IN, "dm9000 irq");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for dm9000 IRQ\n",
			OMAP_DM9000_GPIO_IRQ);
		return;
	}
#if 0
	/* init the mac address using DIE id */
	omap_get_die_id(&odi);

	eth_addr[0] = 0x00; /* locally administered */
	eth_addr[1] = 0x06; //odi.id_1 & 0xff;
	eth_addr[2] = 0xb3; //(odi.id_0 & 0xff000000) >> 24;
	eth_addr[3] = (odi.id_0 & 0x00ff0000) >> 16;
	eth_addr[4] = (odi.id_0 & 0x0000ff00) >> 8;
	eth_addr[5] = (odi.id_0 & 0x000000ff);
#endif
}

static struct resource omap_fpga_resources[] = {
	[0] = {
		.start		= (PC104_BASE + 0x200),
		.end		= (PC104_BASE + 0x200 + 0x0f),
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= (PC104_BASE  + 0x220),
		.end		= (PC104_BASE + 0x220 + 0x0f),
		.flags		= IORESOURCE_MEM,
	},
	[2] = {
		.start		= OMAP_FPGA1_GPIO_IRQ,
//		.end		= OMAP_FPGA1_SOFT_IRQ,
		.flags		= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
	[3] = {
		.start		= OMAP_FPGA2_GPIO_IRQ,
//		.end		= OMAP_FPGA2_SOFT_IRQ,
		.flags		= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
        // deferred irq's
/*	[4] = {
		.start		= OMAP_DM9000_GPIO_IRQ,
		.end		= OMAP_DM9000_SOFT_IRQ,
	},*/
};

static struct platform_device omap_fpga_dev = {
	.name = "fpga",
	.id = -1,
	.num_resources	= ARRAY_SIZE(omap_fpga_resources),
	.resource	= omap_fpga_resources,
};

static void __init fpga_init(void)
{
	int /*ret,*/ i/*, irqs[] = {
            OMAP_FPGA1_GPIO_IRQ, OMAP_FPGA2_GPIO_IRQ,
        }*/;

        /*for (i=0; i < ARRAY_SIZE(irqs); i++) {
            printk(KERN_INFO "Initialize fpga gpio: %d\n", irqs[i]);
	    ret = gpio_request_one(irqs[i], GPIOF_IN, "fpga irq");
	    if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for fpga IRQ\n",
			irqs[i]);
		return;
            }
	}*/

        i = 159;
        printk(KERN_INFO "Init test point %d\n", i);
	if (0 < gpio_request_one(i, GPIOF_OUT_INIT_HIGH, "test point"))
            printk(KERN_ERR "Failed to req gpio%d for test pt\n", i);

}

static struct platform_device *ij3k_devices[] __initdata = {
	&leds_gpio,
	&omap_dm9000_dev,
        &omap_kbd_dev,
        &omap_fpga_dev,
};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* nCS and IRQ for Devkit8000 ethernet */
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE0),
	OMAP3_MUX(ETK_D11, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),

	/* McSPI 2*/
	OMAP3_MUX(MCSPI2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI2_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCSPI2_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI2_CS0, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCSPI2_CS1, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* PENDOWN GPIO */
	OMAP3_MUX(ETK_D13, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* mUSB */
	OMAP3_MUX(HSUSB0_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_STP, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(HSUSB0_DIR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_NXT, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* USB 1 */
	OMAP3_MUX(ETK_CTL, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(ETK_D8, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D9, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP3_MUX(ETK_D7, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* MMC 1 */
	OMAP3_MUX(SDMMC1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* McBSP 2 */
	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* I2C 1 */
	OMAP3_MUX(I2C1_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(I2C1_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* I2C 2 */
	OMAP3_MUX(I2C2_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(I2C2_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* I2C 3 */
	OMAP3_MUX(I2C3_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(I2C3_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* I2C 4 */
	OMAP3_MUX(I2C4_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(I2C4_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* serial ports */
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART1_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* DSS */
	OMAP3_MUX(DSS_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_HSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_VSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_ACBIAS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA0, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA1, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA2, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA3, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA4, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA5, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA8, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA9, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA10, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA11, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA12, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA13, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA14, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA15, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA16, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA17, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA20, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* expansion port */
	/* McSPI 1 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_CS0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCSPI1_CS3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),

	/* HDQ */
	OMAP3_MUX(HDQ_SIO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* McSPI4 */
	OMAP3_MUX(MCBSP1_CLKR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_DX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
//	OMAP3_MUX(MCBSP1_DR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),

	/* MMC 2 */
	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SDMMC2_DAT5, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),

	/* I2C3 */
	OMAP3_MUX(I2C3_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(I2C3_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	OMAP3_MUX(MCBSP1_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCBSP1_FSR, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NCS3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* TPS IRQ */
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_WAKEUP_EN | \
			OMAP_PIN_INPUT_PULLUP),

        /* kbd cols */
        OMAP3_MUX(CAM_XCLKA, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_XCLKB, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_PCLK, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_VS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_HS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_FLD, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_WEN, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_STROBE, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

        /* kbd rows */
        OMAP3_MUX(CAM_D0, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(CAM_D1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(CAM_D2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(CAM_D3, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(CAM_D4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(CAM_D5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(CAM_D6, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(CAM_D7, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP),

        /* kbd cols */
        OMAP3_MUX(CAM_D10, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D11, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

        /* for scope i/o */
        OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(ETK_D10, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D12, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(UART3_RTS_SD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 500,
	.extvbus                = 1,
};

static struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
#if !defined(CONFIG_MACH_DEVKIT8000) && !defined(CONFIG_MACH_OMAP3_IJ3K)
	.pressure_max		= 255,
#endif
	.debounce_max		= 10,
#if defined(CONFIG_MACH_OMAP3_IJ3K)
        .settle_delay_usecs     = 200,
//        .swap_xy		= 1,
	.debounce_tol		= 12, //6,
        .penirq_recheck_delay_usecs = 10,
#else
	.debounce_tol		= 3,
#endif
	.debounce_rep		= 1,
	.gpio_pendown		= -EINVAL,
	.keep_vref_on		= 1,
};

static void __init ij3k_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);
	omap_serial_init();

        omap_led_init();
	omap_dm9000_init();
        fpga_init();

	ij3k_i2c_init();
	platform_add_devices(ij3k_devices,
			ARRAY_SIZE(ij3k_devices));

	omap_display_init(&ij3k_dss_data);

	omap_ads7846_init(2, OMAP3_DEVKIT_TS_GPIO, 0, NULL); //&ads7846_config);

	usb_musb_init(&musb_board_data);
	usbhs_init(&usbhs_bdata);
	omap_nand_flash_init(NAND_BUSWIDTH_16, ij3k_nand_partitions,
			     ARRAY_SIZE(ij3k_nand_partitions));

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
}

MACHINE_START(OMAP3_IJ3K, "OMAP3 IJ3K")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= ij3k_init_early,
	.init_irq	= ij3k_init_irq,
	.init_machine	= ij3k_init,
	.timer		= &omap3_secure_timer,
MACHINE_END
