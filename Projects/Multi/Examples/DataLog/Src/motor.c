#include "motor.h"

#define	PERIOD_DEFAULT	1000			// 1000 us

intr_t motor0_sensors[] = {
		{HORI_SENSOR0_GPIO_PORT, HORI_SENSOR0_GPIO_PIN, HORI_SENSOR0_GPIO_IRQ },
		{HORI_SENSOR1_GPIO_PORT, HORI_SENSOR1_GPIO_PIN, HORI_SENSOR1_GPIO_IRQ },
		{HORI_SENSOR2_GPIO_PORT, HORI_SENSOR2_GPIO_PIN, HORI_SENSOR2_GPIO_IRQ },
		{HORI_SENSOR3_GPIO_PORT, HORI_SENSOR3_GPIO_PIN, HORI_SENSOR3_GPIO_IRQ },
};

intr_t motor1_sensors[] = {
		{VERT_SENSOR1_GPIO_PORT, VERT_SENSOR1_GPIO_PIN, VERT_SENSOR1_GPIO_IRQ },
		{VERT_SENSOR0_GPIO_PORT, VERT_SENSOR0_GPIO_PIN, VERT_SENSOR0_GPIO_IRQ },
};

const int nMotors = ARRAY_SIZE(motors);
motor_t motors[] = {
	{ 
		.Period = PERIOD_DEFAULT,
		.TimX = TIM3,
		.gpio_pls = {HORI_PLS_GPIO_PORT, HORI_PLS_PIN },
		.gpio_dir = {HORI_DIR_GPIO_PORT, HORI_DIR_PIN },
		.sensors = motor0_sensors,
		.nSensors = ARRAY_SIZE(motor0_sensors),
	},
	{
		.Period = PERIOD_DEFAULT,
		.TimX = TIM4,
		.gpio_pls = {VERT_PLS_GPIO_PORT, VERT_PLS_PIN },
		.gpio_dir = {VERT_DIR_GPIO_PORT, VERT_DIR_PIN },
		.sensors = motor1_sensors,
		.nSensors = ARRAY_SIZE(motor1_sensors),
		},
};

void gpio_set( gpio_t *gpio, GPIO_PinState PinState ) {
	HAL_GPIO_WritePin( gpio->GPIOx, gpio->GPIO_Pin, PinState);
}

void gpio_init( gpio_t *gpio ) {
	GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = gpio->GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init( gpio->GPIOx, &GPIO_InitStruct );
}

void intr_init( intr_t *intr ) {
	GPIO_InitTypeDef  GPIO_InitStruct;
	
  GPIO_InitStruct.Pin = intr->GPIO_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; 
  HAL_GPIO_Init( intr->GPIOx, &GPIO_InitStruct);
    
  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority( intr->irq, 0x0F, 0x00 );
  HAL_NVIC_EnableIRQ( intr->irq );
}
	
void motor_tick_init( motor_t *motor ) {
	uint32_t	uwPrescalerValue = 0;

	uwPrescalerValue = (uint32_t) ((SystemCoreClock / 1000000) - 1);	// 1000KHz (1us)
	motor->TimXHandle.Instance = motor->TimX;
	motor->TimXHandle.Init.Period = motor->Period;
  motor->TimXHandle.Init.Prescaler = uwPrescalerValue;
  motor->TimXHandle.Init.ClockDivision = 0;
  motor->TimXHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&motor->TimXHandle);
}

void motor_tick_start( motor_t *motor ) {
	HAL_TIM_Base_Start_IT( &motor->TimXHandle );
}

void motor_init( motor_t *motor ) {
	int	i;

	gpio_init( &motor->gpio_pls );
	gpio_set( &motor->gpio_pls, GPIO_PIN_RESET );
	gpio_init( &motor->gpio_dir );
	gpio_set( &motor->gpio_dir, GPIO_PIN_RESET );
	
	for( i=0; i<motor->nSensors; i++ )
		intr_init( &motor->sensors[i] );

	motor_tick_init( motor );
	motor_tick_start( motor );
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	motor_t	*motor;
	int	i;
	
	for( i=0; i<nMotors; i++ ) {
		if( htim==&motors[i].TimXHandle ) {
			motor = &motors[i];
			break;
		}
	}
	if( i==nMotors )
		return;
	
	if( motor->stop_force ) {
		motor->stop_force = 0;
		if( motor->stop_period >= motor->Period ) {
			motor->accel_steps = (motor->stop_period-motor->Period)/motor->stop_delta * 2;
			motor->steps = motor->accel_steps * 2;
			motor->current_step = motor->accel_steps;
		} else {
			motor->current_step = 0;
			motor->stop_message = 1;
		}
	}
	if( motor->current_step ) {
		if( motor->current_step&1 ) {
			if( motor->current_step&2 ) {
				if( motor->current_step > motor->steps-motor->accel_steps )
					motor->Period -= motor->Period_delta;
				else if( motor->current_step < motor->accel_steps )
					motor->Period += motor->Period_delta;
			}
			motor->TimXHandle.Instance->ARR = motor->Period;
			gpio_set( &motor->gpio_pls, GPIO_PIN_RESET );
		} else {
			motor->real_steps++;
			gpio_set( &motor->gpio_pls, GPIO_PIN_SET );
		}
		motor->current_step--;
		if( motor->current_step==0 )
				motor->stop_message = 1;
	} else {
		motor->Period = 1000;
		motor->TimXHandle.Instance->ARR = motor->Period;
	}
}

int motor_drive( uint8_t id, int8_t direction, uint16_t steps, uint16_t period_start, uint16_t period_end, uint16_t period_accel ) {
	motor_t	*motor;
	
	if( id>=nMotors )
		return 0;
	motor = &motors[id];
	
	if( !motor->current_step ) {
		if( direction ) {
			gpio_set( &motor->gpio_dir, direction>0 ? GPIO_PIN_SET : GPIO_PIN_RESET );
			motor->direction = direction > 0 ? DIR_CW : DIR_CCW;
		}
		motor->steps = steps * 2;
		motor->TimXHandle.Instance->ARR = motor->Period = period_start / 2;
		motor->Period_delta = period_accel;
		motor->accel_steps = (period_start - period_end) / period_accel * 2;
		if( motor->accel_steps > steps )
				motor->accel_steps = steps;
		motor->stop_message = 0;
		motor->stop_force = 0;
		motor->real_steps = 0;
		motor->current_step = motor->steps;
		return 1;
	}
	return 0;
}

void motor_stop( uint8_t id, uint16_t period_stop, uint16_t period_accel ) {
	motor_t	*motor;
	
	if( id>=nMotors )
		return;
	motor = &motors[id];
	
	if( motor->current_step ) {
		motor->stop_force = 1;
		motor->stop_period = period_stop / 2;
		motor->stop_delta = period_accel;
	}
	return;
}

void motor_test( uint8_t id, uint8_t pls, uint8_t dir, uint8_t awo, uint8_t cs) {
	motor_t	*motor;
	
	if( id>=nMotors )
		return;
	motor = &motors[id];
	
	gpio_set( &motor->gpio_pls, pls ? GPIO_PIN_SET : GPIO_PIN_RESET );
	gpio_set( &motor->gpio_dir, dir ? GPIO_PIN_SET : GPIO_PIN_RESET );
}
