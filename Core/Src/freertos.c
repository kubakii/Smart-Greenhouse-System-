/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "data.h"
#include "queue.h"
#include "usart.h"
#include <stdio.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rx_byte;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ReceiveTask */
osThreadId_t ReceiveTaskHandle;
const osThreadAttr_t ReceiveTask_attributes = {
  .name = "ReceiveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for SendTask */
osThreadId_t SendTaskHandle;
const osThreadAttr_t SendTask_attributes = {
  .name = "SendTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for AcquisitionTask */
osThreadId_t AcquisitionTaskHandle;
const osThreadAttr_t AcquisitionTask_attributes = {
  .name = "AcquisitionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for CmdQueue */
osMessageQueueId_t CmdQueueHandle;
const osMessageQueueAttr_t CmdQueue_attributes = {
  .name = "CmdQueue"
};
/* Definitions for DataQueue */
osMessageQueueId_t DataQueueHandle;
const osMessageQueueAttr_t DataQueue_attributes = {
  .name = "DataQueue"
};
/* Definitions for UartQueue */
osMessageQueueId_t UartQueueHandle;
const osMessageQueueAttr_t UartQueue_attributes = {
  .name = "UartQueue"
};
/* Definitions for UartMutex */
osMutexId_t UartMutexHandle;
const osMutexAttr_t UartMutex_attributes = {
  .name = "UartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TASK_PRIORITY_RECEIVE(void *argument);
void TASK_PRIORITY_send(void *argument);
void TASK_PRIORITY_CONTROL(void *argument);
void TASK_PRIORITY_ACQUISITION(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of UartMutex */
  UartMutexHandle = osMutexNew(&UartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CmdQueue */
  CmdQueueHandle = osMessageQueueNew (16, sizeof(SensorData), &CmdQueue_attributes);

  /* creation of DataQueue */
  DataQueueHandle = osMessageQueueNew (16, sizeof(SensorData), &DataQueue_attributes);

  /* creation of UartQueue */
  UartQueueHandle = osMessageQueueNew (20, sizeof(uint8_t), &UartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ReceiveTask */
  ReceiveTaskHandle = osThreadNew(TASK_PRIORITY_RECEIVE, NULL, &ReceiveTask_attributes);

  /* creation of SendTask */
  SendTaskHandle = osThreadNew(TASK_PRIORITY_send, NULL, &SendTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(TASK_PRIORITY_CONTROL, NULL, &ControlTask_attributes);

  /* creation of AcquisitionTask */
  AcquisitionTaskHandle = osThreadNew(TASK_PRIORITY_ACQUISITION, NULL, &AcquisitionTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1); 
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_TASK_PRIORITY_RECEIVE */
/**
  * @brief  Function implementing the ReceiveTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TASK_PRIORITY_RECEIVE */
void TASK_PRIORITY_RECEIVE(void *argument)
{
  /* USER CODE BEGIN TASK_PRIORITY_RECEIVE */
		uint8_t rxByte;
    uint8_t buffer[64];
    uint8_t index = 0;
  /* Infinite loop */
  for(;;)
  {
		if(xQueueReceive(UartQueueHandle, &rxByte, portMAX_DELAY) == pdPASS) {
            buffer[index++] = rxByte;
            
            if(rxByte == '\n' || index >= sizeof(buffer)-1) {
                buffer[index] = '\0';
                
                ControlCmd cmd;
                if(ParseCommand(buffer, &cmd)) {
                    xQueueSend(CmdQueueHandle, &cmd, 0);
                }
                index = 0;
            }
        }

  }
  /* USER CODE END TASK_PRIORITY_RECEIVE */
}

/* USER CODE BEGIN Header_TASK_PRIORITY_send */
/**
* @brief Function implementing the SendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK_PRIORITY_send */
void TASK_PRIORITY_send(void *argument)
{
  /* USER CODE BEGIN TASK_PRIORITY_send */
	SensorData data;
	static char txbuffer[128];
  /* Infinite loop */
  for(;;)
  {
		 if(xQueueReceive(DataQueueHandle, &data, portMAX_DELAY) == pdPASS)
        {
            int len = snprintf(txbuffer, sizeof(txbuffer), 
                "AA,%02X,%02X,%02X,%02X,%04X\n", 
                data.led, data.motor, data.temp, 
                data.humi, data.lightsensor);
            
            
            osMutexAcquire(UartMutexHandle, osWaitForever);
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)txbuffer, len);
						//HAL_UART_Transmit(&huart1, (uint8_t*)txbuffer, len,1000);
            osMutexRelease(UartMutexHandle);
        }
    osDelay(20);
  }
  /* USER CODE END TASK_PRIORITY_send */
}

/* USER CODE BEGIN Header_TASK_PRIORITY_CONTROL */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK_PRIORITY_CONTROL */
void TASK_PRIORITY_CONTROL(void *argument)
{
  /* USER CODE BEGIN TASK_PRIORITY_CONTROL */
	ControlCmd cmd;
    
  /* Infinite loop */
  for(;;)
  {
		 if(xQueueReceive(CmdQueueHandle, &cmd, portMAX_DELAY) == pdPASS) {
            switch(cmd.flag) {
                case MOTORC:
                    Control_Motor(cmd.data);
                    break;
                    
                case LEDC:
                     Control_Light(cmd.data);
                    break;
            }
        }

  }
  /* USER CODE END TASK_PRIORITY_CONTROL */
}

/* USER CODE BEGIN Header_TASK_PRIORITY_ACQUISITION */
/**
* @brief Function implementing the AcquisitionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK_PRIORITY_ACQUISITION */
void TASK_PRIORITY_ACQUISITION(void *argument)
{
  /* USER CODE BEGIN TASK_PRIORITY_ACQUISITION */
	SensorData sensorData;
	uint16_t* sensor;
  /* Infinite loop */
  for(;;)
  {
        
        
				sensor = Read_Sensor();
        sensorData.led = Read_Light();
				sensorData.motor = Read_Motor();
				sensorData.temp = 0;
				sensorData.humi = 0;
				sensorData.lightsensor = sensor[0];
        
        
        xQueueSend(DataQueueHandle, &sensorData, 0);
    osDelay(50);
  }
  /* USER CODE END TASK_PRIORITY_ACQUISITION */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// 将接收到的字节发送到队列
        xQueueSendFromISR(UartQueueHandle, &rx_byte, NULL);
        
        // 重新启动接收（中断方式必须重新启动！）
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}
/* USER CODE END Application */

