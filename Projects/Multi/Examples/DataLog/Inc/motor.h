#ifndef __MOTOR_H
#define	__MOTOR_H

#include "stm32f4xx_hal.h"

// Morpho Connector | Arduino Connector	| Nucleo					| X-Nucleo-IKS01A1
// CN10-1  					| N/A				 				| PC9							| N/A
// CN10-3  					| CN5-10 D15 				| PB8							| I2C_SDA
// CN10-5  					| CN5-9  D14 				| PB9							| I2C_SCL
// CN10-7  					| CN5-8  AVDD 			| AVDD						| X
// CN10-9  					| CN5-7  GND 				|	GND							| GND
// CN10-11 					| CN5-6  D13 				| PA5	(Blue BTN)	| X
// CN10-13 					| CN5-5  D12 				| PA6							| X
// CN10-15 					| CN5-4  D11 				| PA7							| X
// CN10-17 					| CN5-3  D10 				| PB6							| X
// CN10-19 					| CN5-2  D9 				| PC7							| X
// CN10-21 					| CN5-1  D8 				| PA9							| X
// CN10-23 					| CN9-8  D7 				| PA8							| X
// CN10-25 					| CN9-7  D6 				| PB10/PE8				| HTS221_DRDY
// CN10-27 					| CN9-6  D5 				| PB4							| LPS25H_INT1
// CN10-29 					| CN9-5  D4 				| PB5							| LSM6DS0_INT1
// CN10-31 					| CN9-4  D3 				| SWO (PB3)				| X
// CN10-33 					| CN9-3  D2 				| PA10						| USER_INT
// CN10-35 					| CN9-2  D1					| USART_TX (PA2)	| X
// CN10-37 					| CN9-1  D0 				| USART_RX (PA3)	| X

// CN7-2						| N/A								| PC11						| N/A
// CN7-4						| N/A								| PD2							| N/A
// CN7-6						| N/A								| E5V							| N/A
// CN7-8						| N/A								| GND							| N/A
// CN7-10						| CN6-1							| X								| X
// CN7-12						| CN6-2							| +3V3						| +3V3
// CN7-14						| CN6-3							| NRST						| NRST
// CN7-16						| CN6-4							| +3V3						| +3V3
// CN7-18						| CN6-5							| +5V							| +5V
// CN7-20						| CN6-6							| GND							| GND
// CN7-22						| CN6-7							| GND							| GND
// CN7-24						| CN6-8							| VIN 						| X
// CN7-26						| N/A								| X								| X
// CN7-28  					| CN8-1  A0 				| PA0							| X
// CN7-30  					| CN8-2  A1 				| PA1							| X
// CN7-32  					| CN8-3  A2 				| PA4							| M_INT1
// CN7-34  					| CN8-4  A3 				| PB0							| M_INT2
// CN7-36  					| CN8-5  A4 				| PC1							| LIS3MDL_INT1
// CN7-38  					| CN8-6  A5 				| PC0							| LIS3MDL_DRDY

// CN10-2  					| N/A				 				| PC8							| N/A
// CN10-4  					| N/A				 				| PC6							| N/A
// CN10-6  					| N/A				 				| PC5							| N/A
// CN10-8  					| N/A				 				| U5V							| N/A
// CN10-10  				| N/A				 				| PD8							| N/A
// CN10-12  				| N/A				 				| PA12						| N/A
// CN10-14  				| N/A				 				| PA11						| N/A
// CN10-16  				| N/A				 				| PB12						| N/A
// CN10-18  				| N/A				 				| PB11/PE9				| N/A
// CN10-20  				| N/A				 				| GND							| N/A
// CN10-22  				| N/A				 				| PB2							| N/A
// CN10-24  				| N/A				 				| PB1							| N/A
// CN10-26  				| N/A				 				| PA7							| N/A
// CN10-28  				| N/A				 				| PA6							| N/A
// CN10-30  				| N/A				 				| PA5							| N/A
// CN10-32  				| N/A				 				| AGND						| N/A
// CN10-34  				| N/A				 				| PC4							| N/A
// CN10-36  				| N/A				 				| PF5							| N/A
// CN10-38  				| N/A				 				| PF4							| N/A

// CN7-1  					| N/A				 				| PC10						| N/A
// CN7-3  					| N/A				 				| PC12						| N/A
// CN7-5  					| N/A				 				| VDD							| N/A
// CN7-7  					| N/A				 				| BOOT0						| N/A
// CN7-9  					| N/A				 				| PF6							| N/A
// CN7-11  					| N/A				 				| PF7							| N/A
// CN7-13  					| N/A				 				| PA13						| N/A
// CN7-15  					| N/A				 				| PB14						| N/A
// CN7-17  					| N/A				 				| PB15						| N/A
// CN7-19  					| N/A				 				| GND							| N/A
// CN7-21  					| N/A				 				| PB7							| N/A
// CN7-23  					| N/A				 				| PC13						| N/A
// CN7-25  					| N/A				 				| X								| N/A
// CN7-27  					| N/A				 				| X								| N/A
// CN7-39  					| N/A				 				| PF0/PH0/PD0			| N/A
// CN7-31  					| N/A				 				| PF1/PH1/PD1			| N/A
// CN7-33  					| N/A				 				| VBAT/VLCD/VDD		| N/A
// CN7-35  					| N/A				 				| PC2							| N/A
// CN7-37  					| N/A				 				| PC3							| N/A

#define	ARRAY_SIZE(a)										( sizeof(a) / sizeof(a[0]) )
#define	DIR_CW													1
#define	DIR_NA													0
#define	DIR_CCW													-1

#define HORI_PLS_PIN										GPIO_PIN_9					// Nucleo-F401RE CN5-1 D8
#define HORI_PLS_GPIO_PORT							GPIOA
#define HORI_PLS_GPIO_CLK_ENABLE()			__GPIOA_CLK_ENABLE()
#define HORI_PLS_GPIO_CLK_DISABLE()			__GPIOA_CLK_DISABLE()

#define HORI_DIR_PIN										GPIO_PIN_7					// Nucleo-F401RE CN5-2 D9
#define HORI_DIR_GPIO_PORT							GPIOC
#define HORI_DIR_GPIO_CLK_ENABLE()			__GPIOC_CLK_ENABLE()
#define HORI_DIR_GPIO_CLK_DISABLE()			__GPIOC_CLK_DISABLE()

#define HORI_SENSOR0_GPIO_PIN						GPIO_PIN_0					// Nucleo-F401RE CN8-1
#define	HORI_SENSOR0_GPIO_PORT					GPIOA
#define HORI_SENSOR0_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#define HORI_SENSOR0_GPIO_CLK_DISABLE()	__GPIOA_CLK_DISABLE()
#define HORI_SENSOR0_GPIO_IRQ						EXTI0_IRQn

#define HORI_SENSOR1_GPIO_PIN						GPIO_PIN_1					// Nucleo-F401RE CN8-2
#define	HORI_SENSOR1_GPIO_PORT					GPIOA
#define HORI_SENSOR1_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#define HORI_SENSOR1_GPIO_CLK_DISABLE()	__GPIOA_CLK_DISABLE()
#define HORI_SENSOR1_GPIO_IRQ						EXTI1_IRQn

#define HORI_SENSOR2_GPIO_PIN						GPIO_PIN_3					// Nucleo-F401RE CN9-4 D3
#define	HORI_SENSOR2_GPIO_PORT					GPIOB
#define HORI_SENSOR2_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#define HORI_SENSOR2_GPIO_CLK_DISABLE()	__GPIOB_CLK_DISABLE()
#define HORI_SENSOR2_GPIO_IRQ						EXTI3_IRQn

#define HORI_SENSOR3_GPIO_PIN						GPIO_PIN_8					// Nucleo-F401RE CN9-8 D7
#define	HORI_SENSOR3_GPIO_PORT					GPIOA
#define HORI_SENSOR3_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#define HORI_SENSOR3_GPIO_CLK_DISABLE()	__GPIOA_CLK_DISABLE()
#define HORI_SENSOR3_GPIO_IRQ						EXTI9_5_IRQn

#define VERT_PLS_PIN										GPIO_PIN_6					// Nucleo-F401RE CN5-5 D12
#define VERT_PLS_GPIO_PORT							GPIOA
#define VERT_PLS_GPIO_CLK_ENABLE()			__GPIOA_CLK_ENABLE()
#define VERT_PLS_GPIO_CLK_DISABLE()			__GPIOA_CLK_DISABLE()

#define VERT_DIR_PIN										GPIO_PIN_12					// Nucleo-F401RE CN10-12
#define VERT_DIR_GPIO_PORT							GPIOA
#define VERT_DIR_GPIO_CLK_ENABLE()			__GPIOA_CLK_ENABLE()
#define VERT_DIR_GPIO_CLK_DISABLE()			__GPIOA_CLK_DISABLE()

#define VERT_SENSOR0_GPIO_PIN						GPIO_PIN_6					// Nucleo-F401RE CN5-3 D10
#define	VERT_SENSOR0_GPIO_PORT					GPIOB
#define VERT_SENSOR0_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#define VERT_SENSOR0_GPIO_CLK_DISABLE()	__GPIOB_CLK_DISABLE()
#define VERT_SENSOR0_GPIO_IRQ						EXTI9_5_IRQn

#define VERT_SENSOR1_GPIO_PIN						GPIO_PIN_7					// Nucleo-F401RE CN5-4 D11
#define	VERT_SENSOR1_GPIO_PORT					GPIOA
#define VERT_SENSOR1_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#define VERT_SENSOR1_GPIO_CLK_DISABLE()	__GPIOA_CLK_DISABLE()
#define VERT_SENSOR1_GPIO_IRQ						EXTI9_5_IRQn

typedef struct {
	GPIO_TypeDef* GPIOx;			// gpio group address
	uint16_t GPIO_Pin;				// gpin pin
} gpio_t;

typedef struct {
	GPIO_TypeDef* GPIOx;			// gpio group address
	uint16_t GPIO_Pin;				// gpin pin
	IRQn_Type			irq;
	uint8_t				triggered;
} intr_t;

typedef struct {
	uint8_t						id;
	int8_t						direction;
	gpio_t						gpio_pls, gpio_dir;
	TIM_TypeDef 			*TimX;
	TIM_HandleTypeDef	TimXHandle;
	uint16_t					Period;
	uint16_t					Period_delta;
	uint32_t					steps, current_step;
	uint32_t					real_steps;
	uint16_t					accel_steps;
	uint8_t						stop_message;
	uint8_t						stop_force;
	uint16_t					stop_delta, stop_period;
	int								nSensors;
	intr_t						*sensors;
	uint16_t					sensor_steps;
} motor_t;

extern motor_t motors[2];
extern const int nMotors;
extern intr_t motor0_sensors[4];
extern intr_t motor1_sensors[2];

extern void motor_init( motor_t *otor );
int motor_drive( uint8_t id, int8_t direction, uint16_t steps, uint16_t period_start, uint16_t period_end, uint16_t duration_accel );
void motor_stop( uint8_t id, uint16_t period_stop, uint16_t period_accel );
void motor_test( uint8_t id, uint8_t pls, uint8_t dir, uint8_t awo, uint8_t cs);

#endif
