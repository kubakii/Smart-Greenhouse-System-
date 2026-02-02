#include "data.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"


uint8_t Read_Light(void)
{
	return HAL_GPIO_ReadPin(LED_GPIO_Port,LED_Pin);
}

uint8_t Read_Motor(void)
{
	return HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
}

uint8_t Read_Temp(void)
{
	return 0;
}

uint16_t* Read_Sensor(void)
{
	static uint16_t sensor_value[4];
	if(HAL_ADC_Start_DMA(&hadc1,(uint32_t*)sensor_value,4)!=HAL_OK)
	{
		sensor_value[0]=0;
		sensor_value[1]=0;
		sensor_value[2]=0;
		sensor_value[3]=0;
	}
	while(hdma_adc1.State!=HAL_DMA_STATE_READY);
	HAL_ADC_Stop_DMA(&hadc1);
	
	return sensor_value;
}

void Control_Light(uint16_t flag)
{
	switch(flag)
	{
		case 0x00:
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
			break;
		case 0x01:
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
			break;
		default:
			break;
	}
	
}

void Control_Motor(uint16_t speed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);
}

bool ParseCommand(uint8_t *buffer, ControlCmd *cmd) {
    if(strncmp((char*)buffer, "MOTOR", 5) == 0) {
        cmd->flag = MOTORC;
        cmd->data = atoi((char*)buffer + 6);
        return true;
    }
    else if(strncmp((char*)buffer, "LED", 3) == 0) {
        cmd->flag = LEDC;
        cmd->data = atoi((char*)buffer + 4);
        return true;
    }
    return false;
}




