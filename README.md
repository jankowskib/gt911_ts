gt911_ts
============
GT911 Touch Screen Driver For Rockchip

How to configure in your device:
1. Clone into drivers/input/touchscreen/gt9xx
2. Add in drivers/input/touchscreen/Kconfig
```
config TOUCHSCREEN_GT9XX_IIC
	tristate "GT9xx_IIC based touchscreens"
	depends on I2C2_RK30
	
source "drivers/input/touchscreen/gt9xx/Kconfig"
```
3. Add in drivers/input/touchscreen/Makefile
```
obj-$(CONFIG_TOUCHSCREEN_GT9XX_IIC)      +=  gt9xx/
```
4. Add platform structure to your arch/arm/plat-rk/include/plat/board.h file
```
struct goodix_9xx_platform_data 
{
    int model;
    unsigned gpio_shutdown;
    unsigned gpio_irq;
    unsigned gpio_reset;
    bool irq_edge; /* 0:rising edge, 1:falling edge */
    int xmax;
    int ymax;
    int config_info_len;
    u8 *config_info;
    int (*init_platform_hw)(void);
    int (*exit_platform_hw)(void);
    bool send_config;	// Sends config_info, instead ROM's confing
    bool custom_config; // Sends xmax, ymax, irq edge instead stuff in config
    bool mirrorxy;
    bool swap_xy;
};
```
5. Setup touchpanel config in your board file. Make sure you replace goodix_config data with yours!
```
#if defined(CONFIG_TOUCHSCREEN_GT9XX_IIC)

#define TOUCH_RESET_PIN  RK30_PIN4_PD0
#define TOUCH_INT_PIN    RK30_PIN4_PC2
#define TOUCH_PWR_PIN    INVALID_GPIO

	int goodix_init_platform_hw(void)
	{
		int ret;
		
		if (TOUCH_PWR_PIN != INVALID_GPIO) {
			ret = gpio_request(TOUCH_PWR_PIN, "goodix power pin");
			if (ret != 0) {
				gpio_free(TOUCH_PWR_PIN);
				printk("goodix power error\n");
				return -EIO;
			}
			gpio_direction_output(TOUCH_PWR_PIN, 0);
			gpio_set_value(TOUCH_PWR_PIN, GPIO_LOW);
			msleep(100);
		}
		
		if (TOUCH_INT_PIN != INVALID_GPIO) {
			ret = gpio_request(TOUCH_INT_PIN, "goodix irq pin");
			if (ret != 0) {
				gpio_free(TOUCH_INT_PIN);
				printk("goodix irq error\n");
				return -EIO;
			}
		}	

		if (TOUCH_RESET_PIN != INVALID_GPIO) {
			ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
			if (ret != 0) {
				gpio_free(TOUCH_RESET_PIN);
				printk("goodix gpio_request error\n");
				return -EIO;
			}
			
		   gpio_direction_output(TOUCH_RESET_PIN, 0);
		   gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);  
		   msleep(50);
		   gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
		   msleep(50);
		   gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);
		   msleep(50);

		}
		return 0;
	}

void goodix_exit_platform_hw(void)
{
    gpio_free(TOUCH_RESET_PIN);
    gpio_free(TOUCH_INT_PIN);
}

u8 goodix_config[] = {
	0x50,0x00,0x04,0x00,0x03,0x05,0x35,0x00,0x03,0x0D,0x14,0x0F,0x5F,
	0x55,0x03,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x1A,0x1E,
	0x14,0x8C,0x27,0x0E,0x1D,0x23,0x04,0x0D,0x00,0x00,0x00,0x9B,0x02,
	0x35,0x00,0x00,0x00,0x00,0x00,0x03,0x64,0x32,0x00,0x00,0x00,0x32,
	0xFA,0x01,0x00,0x00,0x08,0x00,0x00,0x7B,0x15,0x15,0x7B,0x15,0x1D,
	0x4B,0x1D,0x23,0x4B,0x1D,0x29,0x33,0x21,0x39,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,
	0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
	0x02,0x04,0x06,0x08,0x0A,0x0C,0x0F,0x10,0x12,0x13,0x14,0x16,0x18,
	0x1C,0x1D,0x1E,0x1F,0x20,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0x7E,0x01
};

struct goodix_9xx_platform_data goodix_info = 
{
	.model = 9110,
    .irq_edge = 1, /* 0:rising edge, 1:falling edge */
	.gpio_shutdown = INVALID_GPIO,
    .gpio_irq = TOUCH_INT_PIN,
    .gpio_reset = TOUCH_RESET_PIN,
	.swap_xy = 1,
	.mirrorxy = 0,
	.xmax = 1024,
	.ymax = 768,
	.send_config = 1,
	.custom_config = 1,
	.config_info_len = ARRAY_SIZE(goodix_config),
    .config_info = goodix_config,
	.init_platform_hw = goodix_init_platform_hw,
	.exit_platform_hw = goodix_exit_platform_hw,
};
#endif

//...
#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_TOUCHSCREEN_GT9XX_IIC)
	{
		.type          = "Goodix-TS",
		.addr          = 0x5d,
		.flags         = 0,
		.irq           = TOUCH_INT_PIN,
		.platform_data = &goodix_info,
	},
#endif
```