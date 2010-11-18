/* linux/arch/arm/mach-s3c2440/mach-ij3k.c
 *
 * Copyright (c) 2010 C Nauman <cnauman@diagraph.com>
 *      Based on mach-mini2440.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/dm9000.h>
#include <linux/i2c/at24.h>
#include <linux/platform_device.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/mmc/host.h>
#include <linux/input/matrix_keypad.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/fb.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/leds-gpio.h>
#include <mach/regs-mem.h>
#include <mach/regs-lcd.h>
#include <mach/irqs.h>
#include <plat/nand.h>
#include <plat/iic.h>
#include <plat/mci.h>
#include <plat/udc.h>
#include <plat/ts.h>
#include <mach/regs-gpioj.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <plat/gpio-cfg.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>

#include <sound/s3c24xx_uda134x.h>

#define MACH_IJ3K_DM9K_BASE (S3C2410_CS4 + 0x300)

static struct map_desc ij3k_iodesc[] __initdata = {
	/* nothing to declare, move along */
};

#define UCON S3C2410_UCON_DEFAULT
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE


static struct s3c2410_uartcfg ij3k_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
};

/* USB device UDC support */

static void ij3k_udc_pullup(enum s3c2410_udc_cmd_e cmd)
{
	pr_debug("udc: pullup(%d)\n", cmd);

	switch (cmd) {
		case S3C2410_UDC_P_ENABLE :
			gpio_set_value(S3C2410_GPC(5), 1);
			break;
		case S3C2410_UDC_P_DISABLE :
			gpio_set_value(S3C2410_GPC(5), 0);
			break;
		case S3C2410_UDC_P_RESET :
			break;
		default:
			break;
	}
}

static struct s3c2410_udc_mach_info ij3k_udc_cfg __initdata = {
	.udc_command		= ij3k_udc_pullup,
};


/* LCD timing and setup */

/*
 * This macro simplifies the table bellow
 */
#define _LCD_DECLARE(_clock,_xres,margin_left,margin_right,hsync, \
			_yres,margin_top,margin_bottom,vsync, refresh) \
	.width = _xres, \
	.xres = _xres, \
	.height = _yres, \
	.yres = _yres, \
	.left_margin	= margin_left,	\
	.right_margin	= margin_right,	\
	.upper_margin	= margin_top,	\
	.lower_margin	= margin_bottom,	\
	.hsync_len	= hsync,	\
	.vsync_len	= vsync,	\
	.pixclock	= ((_clock*100000000000LL) /	\
			   ((refresh) * \
			   (hsync + margin_left + _xres + margin_right) * \
			   (vsync + margin_top + _yres + margin_bottom))), \
	.bpp		= 16,\
	.type		= (S3C2410_LCDCON1_TFT16BPP |\
			   S3C2410_LCDCON1_TFT)

static struct s3c2410fb_display ij3k_lcd_cfg[] __initdata = {
	[0] = {	/* mini2440 + 3.5" TFT + touchscreen */
		_LCD_DECLARE(
			7,			/* The 3.5 is quite fast */
			240, 21, 38, 6, 	/* x timing */
			320, 4, 4, 2,		/* y timing */
			60),			/* refresh rate */
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_INVVLINE |
				   S3C2410_LCDCON5_INVVFRAME |
				   S3C2410_LCDCON5_INVVDEN |
				   S3C2410_LCDCON5_PWREN),
	},
	[1] = { /* mini2440 + 7" TFT + touchscreen */
		_LCD_DECLARE(
			10,			/* the 7" runs slower */
			800, 40, 40, 48, 	/* x timing */
			480, 29, 3, 3,		/* y timing */
			50),			/* refresh rate */
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_INVVLINE |
				   S3C2410_LCDCON5_INVVFRAME |
				   S3C2410_LCDCON5_PWREN),
	},
	/* The VGA shield can outout at several resolutions. All share 
	 * the same timings, however, anything smaller than 1024x768
	 * will only be displayed in the top left corner of a 1024x768
	 * XGA output unless you add optional dip switches to the shield.
	 * Therefore timings for other resolutions have been ommited here.
	 */
	[2] = {
		_LCD_DECLARE(
			10,
			1024, 1, 2, 2,		/* y timing */
			768, 200, 16, 16, 	/* x timing */
			24),	/* refresh rate, maximum stable,
				 tested with the FPGA shield */
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_HWSWP),
	},
};

/* todo - put into gpio header */

#define S3C2410_GPCCON_MASK(x)	(3 << ((x) * 2))
#define S3C2410_GPDCON_MASK(x)	(3 << ((x) * 2))

static struct s3c2410fb_mach_info ij3k_fb_info __initdata = {
	.displays	 = &ij3k_lcd_cfg[0], /* not constant! see init */
	.num_displays	 = 1,
	.default_display = 0,

	/* Enable VD[2..7], VD[10..15], VD[18..23] and VCLK, syncs, VDEN
	 * and disable the pull down resistors on pins we are using for LCD
	 * data. */

	.gpcup		= (0xf << 1) | (0x3f << 10),

	.gpccon		= (S3C2410_GPC1_VCLK   | S3C2410_GPC2_VLINE |
			   S3C2410_GPC3_VFRAME | S3C2410_GPC4_VM |
			   S3C2410_GPC10_VD2   | S3C2410_GPC11_VD3 |
			   S3C2410_GPC12_VD4   | S3C2410_GPC13_VD5 |
			   S3C2410_GPC14_VD6   | S3C2410_GPC15_VD7),

	.gpccon_mask	= (S3C2410_GPCCON_MASK(1)  | S3C2410_GPCCON_MASK(2)  |
			   S3C2410_GPCCON_MASK(3)  | S3C2410_GPCCON_MASK(4)  |
			   S3C2410_GPCCON_MASK(10) | S3C2410_GPCCON_MASK(11) |
			   S3C2410_GPCCON_MASK(12) | S3C2410_GPCCON_MASK(13) |
			   S3C2410_GPCCON_MASK(14) | S3C2410_GPCCON_MASK(15)),

	.gpdup		= (0x3f << 2) | (0x3f << 10),

	.gpdcon		= (S3C2410_GPD2_VD10  | S3C2410_GPD3_VD11 |
			   S3C2410_GPD4_VD12  | S3C2410_GPD5_VD13 |
			   S3C2410_GPD6_VD14  | S3C2410_GPD7_VD15 |
			   S3C2410_GPD10_VD18 | S3C2410_GPD11_VD19 |
			   S3C2410_GPD12_VD20 | S3C2410_GPD13_VD21 |
			   S3C2410_GPD14_VD22 | S3C2410_GPD15_VD23),

	.gpdcon_mask	= (S3C2410_GPDCON_MASK(2)  | S3C2410_GPDCON_MASK(3) |
			   S3C2410_GPDCON_MASK(4)  | S3C2410_GPDCON_MASK(5) |
			   S3C2410_GPDCON_MASK(6)  | S3C2410_GPDCON_MASK(7) |
			   S3C2410_GPDCON_MASK(10) | S3C2410_GPDCON_MASK(11)|
			   S3C2410_GPDCON_MASK(12) | S3C2410_GPDCON_MASK(13)|
			   S3C2410_GPDCON_MASK(14) | S3C2410_GPDCON_MASK(15)),
};

/* MMC/SD  */

static struct s3c24xx_mci_pdata ij3k_mmc_cfg __initdata = {
   .gpio_detect   = S3C2410_GPG(8),
   .gpio_wprotect = S3C2410_GPH(8),
   .set_power     = NULL,
   .ocr_avail     = MMC_VDD_32_33|MMC_VDD_33_34,
};

/* NAND Flash on IJ3K board */

static struct mtd_partition ij3k_default_nand_part[] __initdata = {
	[0] = {
		.name	= "u-boot",
		.size	= SZ_256K,
		.offset	= 0,
	},
	[1] = {
		.name	= "u-boot-env",
		.size	= SZ_128K,
		.offset	= SZ_256K,
	},
	[2] = {
		.name	= "kernel",
		/* 5 megabytes, for a kernel with no modules
		 * or a uImage with a ramdisk attached */
		.size	= 0x00500000,
		.offset	= SZ_256K + SZ_128K,
	},
	[3] = {
		.name	= "root",
		.offset	= SZ_256K + SZ_128K + 0x00500000,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct s3c2410_nand_set ij3k_nand_sets[] __initdata = {
	[0] = {
		.name		= "nand",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(ij3k_default_nand_part),
		.partitions	= ij3k_default_nand_part,
		.flash_bbt 	= 1, /* we use u-boot to create a BBT */
	},
};

static struct s3c2410_platform_nand ij3k_nand_info __initdata = {
	.tacls		= 0,
	.twrph0		= 25,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(ij3k_nand_sets),
	.sets		= ij3k_nand_sets,
	.ignore_unset_ecc = 1,
};

/* DM9000AEP 10/100 ethernet controller */

static struct resource ij3k_dm9k_resource[] = {
	[0] = {
		.start = MACH_IJ3K_DM9K_BASE,
		.end   = MACH_IJ3K_DM9K_BASE + 3,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = MACH_IJ3K_DM9K_BASE + 4,
		.end   = MACH_IJ3K_DM9K_BASE + 7,
		.flags = IORESOURCE_MEM
	},
	[2] = {
		.start = IRQ_EINT7,
		.end   = IRQ_EINT7,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

/*
 * The DM9000 has no eeprom, and it's MAC address is set by
 * the bootloader before starting the kernel.
 */
static struct dm9000_plat_data ij3k_dm9k_pdata = {
	.flags		= (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
};

static struct platform_device ij3k_device_eth = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ij3k_dm9k_resource),
	.resource	= ij3k_dm9k_resource,
	.dev		= {
		.platform_data	= &ij3k_dm9k_pdata,
	},
};

/* LEDS */

static struct s3c24xx_led_platdata ij3k_led1_pdata = {
	.name		= "led1",
	.gpio		= S3C2410_GPB(5),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "heartbeat",
};

static struct s3c24xx_led_platdata ij3k_led2_pdata = {
	.name		= "led2",
	.gpio		= S3C2410_GPB(6),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "nand-disk",
};

static struct s3c24xx_led_platdata ij3k_led3_pdata = {
	.name		= "led3",
	.gpio		= S3C2410_GPB(7),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "mmc0",
};

static struct s3c24xx_led_platdata ij3k_led4_pdata = {
	.name		= "led4",
	.gpio		= S3C2410_GPB(8),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "",
};

static struct s3c24xx_led_platdata ij3k_led_backlight_pdata = {
	.name		= "backlight",
	.gpio		= S3C2410_GPG(4),
	.def_trigger	= "backlight",
};

static struct platform_device ij3k_led1 = {
	.name		= "s3c24xx_led",
	.id		= 1,
	.dev		= {
		.platform_data	= &ij3k_led1_pdata,
	},
};

static struct platform_device ij3k_led2 = {
	.name		= "s3c24xx_led",
	.id		= 2,
	.dev		= {
		.platform_data	= &ij3k_led2_pdata,
	},
};

static struct platform_device ij3k_led3 = {
	.name		= "s3c24xx_led",
	.id		= 3,
	.dev		= {
		.platform_data	= &ij3k_led3_pdata,
	},
};

static struct platform_device ij3k_led4 = {
	.name		= "s3c24xx_led",
	.id		= 4,
	.dev		= {
		.platform_data	= &ij3k_led4_pdata,
	},
};

static struct platform_device ij3k_led_backlight = {
	.name		= "s3c24xx_led",
	.id		= 5,
	.dev		= {
		.platform_data	= &ij3k_led_backlight_pdata,
	},
};

/* AUDIO */

/*static struct s3c24xx_uda134x_platform_data mini2440_audio_pins = {
	.l3_clk = S3C2410_GPB(4),
	.l3_mode = S3C2410_GPB(2),
	.l3_data = S3C2410_GPB(3),
	.model = UDA134X_UDA1341
};

static struct platform_device mini2440_audio = {
	.name		= "s3c24xx_uda134x",
	.id		= 0,
	.dev		= {
		.platform_data	= &mini2440_audio_pins,
	},
};*/


//////////////////////////////////////////////////////////////
/*
 * ij Keyboard Device
 */
#define IJ_OFF	KEY_POWER2
#define IJ_ON	KEY_POWER

// KEY(row, col, val)
static const uint32_t ij3k_kbd_keymap[] = {
	KEY(0, 0, KEY_BACKSLASH),
	KEY(1, 0, KEY_RIGHTBRACE),
	KEY(2, 0, KEY_P),
	KEY(3, 0, KEY_LEFTBRACE),
	KEY(4, 0, KEY_SPACE),
	KEY(5, 0, KEY_LEFT),
	KEY(6, 0, KEY_O),
	KEY(7, 0, KEY_I),

	KEY(0, 1, KEY_ESC),
	KEY(1, 1, KEY_1),
	KEY(2, 1, KEY_3),
	KEY(3, 1, KEY_2),
	KEY(4, 1, KEY_5),
	KEY(5, 1, KEY_4),
	KEY(6, 1, KEY_6),
	KEY(7, 1, KEY_7),

	KEY(0, 2, KEY_BACKSPACE),
	KEY(1, 2, KEY_RESERVED),
	KEY(2, 2, KEY_MINUS),
	KEY(3, 2, KEY_EQUAL),
	KEY(4, 2, KEY_0),
	KEY(5, 2, KEY_RESERVED),
	KEY(6, 2, KEY_9),
	KEY(7, 2, KEY_8),

	KEY(0, 3, IJ_OFF),
	KEY(1, 3, IJ_ON),
	KEY(2, 3, KEY_RESERVED),
	KEY(3, 3, KEY_F4),
	KEY(4, 3, KEY_F3),
	KEY(5, 3, KEY_F1),
	KEY(6, 3, KEY_F2),
	KEY(7, 3, KEY_RESERVED),

	KEY(0, 4, KEY_TAB),
	KEY(1, 4, KEY_Q),
	KEY(2, 4, KEY_E),
	KEY(3, 4, KEY_W),
	KEY(4, 4, KEY_T),
	KEY(5, 4, KEY_R),
	KEY(6, 4, KEY_Y),
	KEY(7, 4, KEY_U),

	KEY(0, 5, KEY_ENTER),
	KEY(1, 5, KEY_RESERVED),
	KEY(2, 5, KEY_APOSTROPHE),
	KEY(3, 5, KEY_RESERVED),
	KEY(4, 5, KEY_RESERVED),
	KEY(5, 5, KEY_SEMICOLON),
	KEY(6, 5, KEY_L),
	KEY(7, 5, KEY_K),

	KEY(0, 6, KEY_CAPSLOCK),
	KEY(1, 6, KEY_A),
	KEY(2, 6, KEY_D),
	KEY(3, 6, KEY_S),
	KEY(4, 6, KEY_G),
	KEY(5, 6, KEY_F),
	KEY(6, 6, KEY_H),
	KEY(7, 6, KEY_J),

	KEY(0, 7, KEY_RESERVED),
	KEY(1, 7, KEY_RIGHT),
	KEY(2, 7, KEY_UP),
	KEY(3, 7, KEY_DOWN),
	KEY(4, 7, KEY_DOT),
	KEY(5, 7, KEY_SLASH),
	KEY(6, 7, KEY_COMMA),
	KEY(7, 7, KEY_M),

	KEY(0, 8, KEY_RESERVED),
	KEY(1, 8, KEY_RESERVED),
	KEY(2, 8, KEY_X),
	KEY(3, 8, KEY_Z),
	KEY(4, 8, KEY_V),
	KEY(5, 8, KEY_C),
	KEY(6, 8, KEY_B),
	KEY(7, 8, KEY_N),

	KEY(0, 9, KEY_RIGHTSHIFT),
	KEY(1, 9, KEY_LEFTSHIFT),
	KEY(2, 9, KEY_LEFTMETA),
	KEY(3, 9, KEY_LEFTCTRL),
	KEY(4, 9, KEY_DELETE),
	KEY(5, 9, KEY_LEFTALT),
	KEY(6, 9, KEY_RESERVED),
	KEY(7, 9, KEY_RESERVED),
};

static struct matrix_keymap_data ij3k_kbd_keymap_data = {
	.keymap		= ij3k_kbd_keymap,
	.keymap_size	= ARRAY_SIZE(ij3k_kbd_keymap),
};

static const int ij3k_kbd_row_gpios[] = { 
                    S3C2410_GPG(0),
                    S3C2410_GPF(6),
                    S3C2410_GPF(5),
                    S3C2410_GPF(4),
                    S3C2410_GPF(3),
                    S3C2410_GPF(2),
                    S3C2410_GPF(1),
                    S3C2410_GPF(0),
                };

static const int ij3k_kbd_col_gpios[] = { 
                    S3C2410_GPJ(3),
                    S3C2410_GPJ(2),
                    S3C2410_GPJ(1),
                    S3C2410_GPJ(0),
                    S3C2410_GPJ(4),
                    S3C2410_GPJ(5),
                    S3C2410_GPJ(6),
                    S3C2410_GPJ(7),
                    S3C2410_GPJ(8),
                    S3C2410_GPJ(9),
                };

static struct matrix_keypad_platform_data ij3k_kbd_pdata = {
	.keymap_data		= &ij3k_kbd_keymap_data,
	.row_gpios		= ij3k_kbd_row_gpios,
	.col_gpios		= ij3k_kbd_col_gpios,
	.num_row_gpios		= ARRAY_SIZE(ij3k_kbd_row_gpios),
	.num_col_gpios		= ARRAY_SIZE(ij3k_kbd_col_gpios),
	.col_scan_delay_us	= 10,
	.debounce_ms		= 10,
        .active_low             = 1,
//	.wakeup			= 1,
};

static struct platform_device ij3k_kbd_device = {
	.name		= "matrix-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &ij3k_kbd_pdata,
	},
};

static struct s3c2410_ts_mach_info ij3k_ts_cfg __initdata = {
	.delay = 10000,
	.presc = 49,
	.oversampling_shift = 3,
};

//////////////////////////////////////////////////////////////

/*
 * I2C devices
 */
static struct at24_platform_data at24c08 = {
	.byte_len	= SZ_8K / 8,
	.page_size	= 16,
};

static struct i2c_board_info ij3k_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("24c08", 0x50),
		.platform_data = &at24c08,
	},
};

static struct platform_device *ij3k_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_rtc,
        &ij3k_kbd_device,
	&ij3k_device_eth,
	&ij3k_led1,
	&ij3k_led2,
	&ij3k_led3,
	&ij3k_led4,
        &s3c_device_adc,
//        &s3c_device_ts,
	&s3c_device_nand,
	&s3c_device_sdi,
	&s3c_device_iis,
//	&mini2440_audio,
};

static void __init ij3k_map_io(void)
{
	s3c24xx_init_io(ij3k_iodesc, ARRAY_SIZE(ij3k_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(ij3k_uartcfgs, ARRAY_SIZE(ij3k_uartcfgs));
}

/*
 * ij3k_features string
 *
 * t = Touchscreen present
 * b = backlight control
 * c = camera [TODO]
 * 0-9 LCD configuration
 *
 */
static char ij3k_features_str[12] __initdata = "0tb";

static int __init ij3k_features_setup(char *str)
{
	if (str)
		strlcpy(ij3k_features_str, str, sizeof(ij3k_features_str));
	return 1;
}

__setup("ij3k=", ij3k_features_setup);

#define FEATURE_SCREEN (1 << 0)
#define FEATURE_BACKLIGHT (1 << 1)
#define FEATURE_TOUCH (1 << 2)
#define FEATURE_CAMERA (1 << 3)

struct ij3k_features_t {
	int count;
	int done;
	int lcd_index;
	struct platform_device *optional[8];
};

static void ij3k_parse_features(
		struct ij3k_features_t * features,
		const char * features_str )
{
	const char * fp = features_str;

	features->count = 0;
	features->done = 0;
	features->lcd_index = -1;

	while (*fp) {
		char f = *fp++;

		switch (f) {
		case '0'...'9':	/* tft screen */
			if (features->done & FEATURE_SCREEN) {
				printk(KERN_INFO "IJ3K: '%c' ignored, "
					"screen type already set\n", f);
			} else {
				int li = f - '0';
				if (li >= ARRAY_SIZE(ij3k_lcd_cfg))
					printk(KERN_INFO "IJ3K: "
						"'%c' out of range LCD mode\n", f);
				else {
					features->optional[features->count++] =
							&s3c_device_lcd;
					features->lcd_index = li;
				}
			}
			features->done |= FEATURE_SCREEN;
			break;
		case 'b':
			if (features->done & FEATURE_BACKLIGHT)
				printk(KERN_INFO "IJ3K: '%c' ignored, "
					"backlight already set\n", f);
			else {
				features->optional[features->count++] =
						&ij3k_led_backlight;
			}
			features->done |= FEATURE_BACKLIGHT;
			break;
		case 't':
                        if (features->done & FEATURE_TOUCH)
			    printk(KERN_INFO "IJ3K: '%c' ignored, "
				"touchscreen already set\n", f);
                        else {
                            features->optional[features->count++] = 
                                &s3c_device_ts;
                        }
                        features->done |= FEATURE_TOUCH;
			break;
/*		case 'c':
			if (features->done & FEATURE_CAMERA)
				printk(KERN_INFO "IJ3K: '%c' ignored, "
					"camera already registered\n", f);
			else
				features->optional[features->count++] =
					&s3c_device_camif;
			features->done |= FEATURE_CAMERA;
			break;*/
		}
	}
}

static void __init ij3k_init(void)
{
	struct ij3k_features_t features = { 0 };
        int i, ret;

	printk(KERN_INFO "IJ3K: Option string ij3k=%s\n",
			ij3k_features_str);

	/* Parse the feature string */
	ij3k_parse_features(&features, ij3k_features_str);

	/* turn LCD on */
	s3c_gpio_cfgpin(S3C2410_GPC(0), S3C2410_GPC0_LEND);

	/* Turn the backlight early on */
	WARN_ON(gpio_request(S3C2410_GPG(4), "backlight"));
	gpio_direction_output(S3C2410_GPG(4), 1);

	/* remove pullup on optional PWM backlight -- unused on 3.5 and 7"s */
	s3c_gpio_setpull(S3C2410_GPB(1), S3C_GPIO_PULL_UP);
	s3c2410_gpio_setpin(S3C2410_GPB(1), 0);
	s3c_gpio_cfgpin(S3C2410_GPB(1), S3C2410_GPIO_INPUT);

        /* configure keyboard for pullups */
        for (i=0; i < ARRAY_SIZE(ij3k_kbd_row_gpios); i++) {
	    ret = s3c_gpio_setpull(ij3k_kbd_row_gpios[i], S3C_GPIO_PULL_UP);
        }

	/* Make sure the D+ pullup pin is output */
	WARN_ON(gpio_request(S3C2410_GPC(5), "udc pup"));
	gpio_direction_output(S3C2410_GPC(5), 0);

	if (features.lcd_index != -1) {
		int li;

		ij3k_fb_info.displays =
			&ij3k_lcd_cfg[features.lcd_index];

		printk(KERN_INFO "IJ3K: LCD");
		for (li = 0; li < ARRAY_SIZE(ij3k_lcd_cfg); li++)
			if (li == features.lcd_index)
				printk(" [%d:%dx%d]", li,
					ij3k_lcd_cfg[li].width,
					ij3k_lcd_cfg[li].height);
			else
				printk(" %d:%dx%d", li,
					ij3k_lcd_cfg[li].width,
					ij3k_lcd_cfg[li].height);
		printk("\n");
		s3c24xx_fb_set_platdata(&ij3k_fb_info);
	}

	s3c24xx_udc_set_platdata(&ij3k_udc_cfg);
	s3c24xx_mci_set_platdata(&ij3k_mmc_cfg);
	s3c24xx_ts_set_platdata(&ij3k_ts_cfg);

	s3c_nand_set_platdata(&ij3k_nand_info);
	s3c_i2c0_set_platdata(NULL);

	i2c_register_board_info(0, ij3k_i2c_devs,
				ARRAY_SIZE(ij3k_i2c_devs));

	platform_add_devices(ij3k_devices, ARRAY_SIZE(ij3k_devices));

	if (features.count)	/* the optional features */
		platform_add_devices(features.optional, features.count);

}


MACHINE_START(MINI2440, "MINI2440")
	/* Maintainer: Michel Pollet <buserror@gmail.com> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= ij3k_map_io,
	.init_machine	= ij3k_init,
	.init_irq	= s3c24xx_init_irq,
	.timer		= &s3c24xx_timer,
MACHINE_END
