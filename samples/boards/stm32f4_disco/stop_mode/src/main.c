#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <stm32f4xx_ll_pwr.h>
#include <stm32f4xx_ll_cortex.h>
#include <clock_control/stm32_clock_control.h>

static struct device *gpioa;
static struct gpio_callback gpiocb;

static struct device *gpiod;
#define LED_NUMBER 14

static int message_cnt = 1;

void wake_up(struct device *gpioc, struct gpio_callback *cb,
                    uint32_t pins)
{
	return;
}

void configure_gpio(void)
{
	// User Button(PA0)
	gpioa = device_get_binding("GPIOA");
	if (!gpioa) {
		printk("Error\n");
		return;
	}
	gpio_pin_configure(gpioa, 0, GPIO_DIR_IN
		                        | GPIO_INT | GPIO_INT_EDGE
		                        | GPIO_INT_ACTIVE_HIGH);
	gpio_init_callback(&gpiocb, wake_up, BIT(0));
	gpio_add_callback(gpioa, &gpiocb);
	gpio_pin_enable_callback(gpioa, 0);


	// RED LED(PD14)
	gpiod = device_get_binding("GPIOD");
	if (!gpiod) {
		printk("Error\n");
		return;
	}
	gpio_pin_configure(gpiod, LED_NUMBER, GPIO_DIR_OUT);
}

/*
 * I saw an article on Zerphy's mailing list about initializing the clock in STOP Mode.
 * In this article I initialized STM32_CLOCK_CONTROL_NAME and initialized the clock.
 *
 * Reference mailing list: https://lists.zephyrproject.org/g/users/message/1154?p=,,,20,0,0,0::Created,,STM32f0+Stop+Mode,20,2,0,27217100
 */
void clock_init(void)
{
	struct device *clk;
	struct device_config *cfg;
	int clk_status;

	clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
	if (!clk) {
		printk("Error\n");
		return;
	}

	cfg = clk->config;
	clk_status = cfg->init(clk);
}


void stop_mode(void)
{
	LL_PWR_SetPowerMode(LL_PWR_MODE_STOP_MAINREGU);
	printk("Enter stop mode\n");
	LL_LPM_EnableDeepSleep();

	/*
	 * When the user button interrupts, it wakes up in STOP mode.
	 * However, STOP Mode is automatically released after about 1 second even if the User Button does not generate an interrupt.
	 * I thought there was a hardware interrupt that I do not know.
	 * So I disabled SYSTICK by referring to https://community.st.com/s/question/0D50X00009XkXphSAF/unwanted-stop-mode-wakeup-stm32l052k8.
	 * I do not know the exact reason yet.
	 */
	LL_SYSTICK_DisableIT();

	__WFI();


	LL_SYSTICK_EnableIT();
	clock_init();
	LL_LPM_EnableSleep();
}

void main(void)
{
	printk("stop mode test: %s\n", CONFIG_BOARD);
	printk("After 3 seconds, enter stop mode.\n");

	configure_gpio();

	while (1) {
		printk("%dst output message.\n", message_cnt++);
		if (message_cnt == 31) {
			stop_mode();
		}

		gpio_pin_write(gpiod, LED_NUMBER, message_cnt % 2);
		k_sleep(100);
	}
}
