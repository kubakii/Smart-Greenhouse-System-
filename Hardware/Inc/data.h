#include <stdint.h>
#include <stdbool.h> 
#include <string.h>  
#include <stdlib.h> 


typedef enum {
				LEDC 	    =0x01,				
				MOTORC,					
} cflagType_t;



typedef struct {
	uint8_t led;
	uint8_t motor;
	uint8_t temp;
	uint8_t humi;
	uint8_t lightsensor;
}SensorData;

typedef struct {
	cflagType_t  flag;
	uint16_t    data;
}ControlCmd;


uint8_t Read_Light(void);
uint8_t Read_Motor(void);
uint8_t Read_Temp(void);
uint16_t* Read_Sensor(void);
void Control_Light(uint16_t flag);
void Control_Motor(uint16_t speed);
bool ParseCommand(uint8_t *buffer, ControlCmd *cmd);




