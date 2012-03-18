#define MS_PER_SEC		1000
#define DEBOUNCE_DELAY		40
//* STM32 includes */
#include <stm32f10x.h>
#include <stm32f10x_conf.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* Includes */
#include "common.h"
#include "serial.h"
#include "dev_misc.h"

#define LED_PORT GPIOC
#define LED_1 GPIO_Pin_8
#define LED_2 GPIO_Pin_9

int __errno; 

/* Function Prototypes */
void setup_rcc(void);
void setup_gpio(void);
void setup_exti(void);
void setup_nvic(void);

void button_task(void *pvParameters) NORETURN;

void toggle_blue(void);
void toggle_green(void);

static xSemaphoreHandle debounce_sem;

enum button_state {
	BUTTON_STATE_UP,
	BUTTON_STATE_DOWN
};

static enum button_state button_state;
static uint8_t led_blue = 1;
static uint8_t led_green = 1;


void dev_enable_ints()
{
//	sei();
}

void dev_disable_ints()
{
//	cli();
}

void toggle_led()
{
	toggle_green();
}

void delay_ms(double time_ms) 
{

}

void delay_us(double time_us)
{
//	_delay_us(time_us); 
}

void sleep_mode()
{

}

extern int grbl_main();
void grbl_task(void * args)
{
	grbl_main();	
	while(1);
}

void grbl_task( void *pvParameters );
void blink_task( void *pvParameters );

void blink_task(void * param)
{	
	const portTickType xDelay = 500/portTICK_RATE_MS;
	
	for(;;){
		toggle_blue();
		toggle_green();
		vTaskDelay( xDelay );
	}
	while(1);	
}

void startup_task(void *pvParameters);
inline void setup();

inline void main_noreturn(void)
{
	xTaskHandle task;

	xTaskCreate(startup_task, (signed portCHAR *)"startup",
			configMINIMAL_STACK_SIZE * 1, NULL, tskIDLE_PRIORITY + 1,
			&task);
	assert_param(task);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();
	assert_param(NULL);
	while (1) ;
}

void startup_task(void *pvParameters)
{
	vSemaphoreCreateBinary(debounce_sem);

	setup();
//	serial_init();
	
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//enable receiving.

	xTaskHandle task;
/*
	xTaskCreate(serial_task, (signed portCHAR *)"serial",
			configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2,
			&task);
	assert_param(task);
*/
	xTaskCreate(blink_task, (signed portCHAR *)"blink",
			configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,
			&task);
	assert_param(task);

	vTaskDelete(NULL);
	for (;;);
}

/**
 * Main function
 */
int main(void)
{
	main_noreturn();
}

inline void setup()
{
	setup_rcc();
	setup_gpio();
	setup_exti();
	//setup_usart();
	setup_nvic();
}

/**
 * @brief  Configures the peripheral clocks
 * @param  None
 * @retval None
 */
void setup_rcc(void)
{
	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR |
			RCC_APB1Periph_BKP | RCC_APB1Periph_TIM2,
			ENABLE);

	/* Enable GPIOA and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
			RCC_APB2Periph_GPIOC |
			RCC_APB2Periph_USART1 |
			RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports
 * @param  None
 * @retval None
 */
void setup_gpio(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Configure UART tx pin */
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	/* Configure UART rx pin */
	gpio_init.GPIO_Pin = GPIO_Pin_10;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	//config LED pins and servo
	gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;	
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &gpio_init);

	/* Configure button input floating */
	gpio_init.GPIO_Pin = GPIO_Pin_0;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_init);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
}

/**
 * @brief  Configures EXTI Lines
 * @param  None
 * @retval None
 */
void setup_exti(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}


/**
 * @brief  Configure the nested vectored interrupt controller
 * @param  None
 * @retval None
 */
void setup_nvic(void)
{
	NVIC_InitTypeDef nvic_init;

	/* Set the Vector Table base address as specified in .ld file */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* 4 bits for Interupt priorities so no sub priorities */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	/* Configure USART interrupt */
	nvic_init.NVIC_IRQChannel = USART1_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xf;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure EXTI interrupt */
	nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xc;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure Servo interrupt 
	nvic_init.NVIC_IRQChannel = TIM2_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0x2;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);*/
}

/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 */
void exti0_isr(void)
{
	static signed portBASE_TYPE task_woken;

	// Clear pending bit 
	EXTI_ClearITPendingBit(EXTI_Line0);

	xSemaphoreGiveFromISR(debounce_sem, &task_woken);
	portEND_SWITCHING_ISR(task_woken);
}

void button_task(void *pvParameters)
{
	portTickType delay = portMAX_DELAY;
	uint8_t debounce = 0;

	toggle_blue();

	for (;;) {
		if (xSemaphoreTake(debounce_sem, delay) == pdTRUE) {
			if (!debounce) {
				debounce = 1;
				delay = DEBOUNCE_DELAY;
			}
		} else {
			volatile uint8_t button =
				GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

			if (button_state == BUTTON_STATE_UP && button) {
				button_state = BUTTON_STATE_DOWN;
				toggle_blue();
				//debug_msg("bprssd");
			} else if (button_state == BUTTON_STATE_DOWN && !button) {
				button_state = BUTTON_STATE_UP;
				//debug_msg("brlsd");
			}

			debounce = 0;
			delay = portMAX_DELAY;
		}
	}
}

void toggle_blue()
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_blue);
	led_blue ^= 1;
}

void toggle_green()
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_green);
	led_green ^= 1;
}

