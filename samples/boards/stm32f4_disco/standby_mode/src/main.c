#include <zephyr.h>
#include <misc/printk.h>
#include <stm32f4xx_ll_pwr.h>
#include <stm32f4xx_ll_cortex.h>
#include <device.h>
#include <gpio.h>

static struct device *gpioa;
static struct gpio_callback gpiocb;

static struct k_timer timer;

static int standby_mode_cnt = 0;

void sleep_mode_button(struct device *gpioc, struct gpio_callback *cb,
                    uint32_t pins)

{
	k_timer_start(&timer, 100, 0);
}



void configure_gpio(void)
{
	gpioa = device_get_binding("GPIOA");
	if (!gpioa) {
		printk("Error\n");
		return;
	}

	gpio_pin_configure(gpioa, 0, GPIO_DIR_IN
		                        | GPIO_INT | GPIO_INT_EDGE
		                        | GPIO_INT_ACTIVE_HIGH);
	gpio_init_callback(&gpiocb, sleep_mode_button, BIT(0));
	gpio_add_callback(gpioa, &gpiocb);
	gpio_pin_enable_callback(gpioa, 0);
}

/*
 * Press and hold the user button(LL_PWR_WAKEUP_PIN1: PA0) for 3 seconds to enter standby mode.
 */
static void sleep_mode_timer(struct k_timer *work)
{
	int value;

	gpio_pin_read(gpioa, 0, &value);
	if (value == 1) {
		standby_mode_cnt++;
		if (standby_mode_cnt == 30) {
			/*
			 * In stm32f4_disco, LL_PWR_WAKEUP_PIN1 is PA0.
			 * PA0 is also built in as a User Button.
			 */
			LL_PWR_DisableWakeUpPin(LL_PWR_WAKEUP_PIN1);
			LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN1);
			LL_PWR_ClearFlag_WU();
			if (LL_PWR_IsActiveFlag_WU()) {
				LL_PWR_ClearFlag_WU();
			}
			LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

			printk("Now call LL_LPM_EnableDeepSleep().\n");
			LL_LPM_EnableDeepSleep();
		}
	
		k_timer_start(&timer, 100, 0);
	} else {
		standby_mode_cnt = 0;
	}

}

void main(void)
{
	printk("standby mode test: %s\n", CONFIG_BOARD);

	configure_gpio();

	k_timer_init(&timer, sleep_mode_timer, NULL);

	while (1) {
		k_sleep(1000);
		printk("You can not see this message in standby mode.\n");
	}
}
