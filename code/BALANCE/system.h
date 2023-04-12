#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//����������Ҫ�õ���ͷ�ļ�
#include "FreeRTOSConfig.h"
//FreeRTOS���ͷ�ļ� 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//��������ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "show.h"								   
#include "key.h"
#include "lidar.h"
#include "pstwo.h"
#include "robot_select_init.h"

// Enumeration of car types
//С���ͺŵ�ö�ٶ���
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car
} CarMode;

//Motor speed control related parameters of the structure
//����ٶȿ�����ز����ṹ��
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //��������ֵ����ȡ���ʵʱ�ٶ�
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //���PWM��ֵ�����Ƶ��ʵʱ�ٶ�
	float Target;      //Control the target speed of the motor //���Ŀ���ٶ�ֵ�����Ƶ��Ŀ���ٶ�
	float Velocity_KP; //Speed control PID parameters //�ٶȿ���PID����
	float	Velocity_KI; //Speed control PID parameters //�ٶȿ���PID����
}Motor_parameter;

//Smoothed the speed of the three axes
//ƽ��������������ٶ�
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** �ⲿ�������壬������c�ļ�����system.hʱ��Ҳ����ʹ��system.c����ı��� ******/
extern uint8_t Flag_Stop;
extern int Divisor_Mode;
extern uint8_t Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern uint8_t CCD_ON_Flag, APP_ON_Flag, ELE_ON_Flag, Avoid_ON_Flag, Follow_ON_Flag, Along_wall, PS2_ON_Flag;
extern uint8_t Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern uint8_t PID_Send; 
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern float CCD_KP,CCD_KI;
extern float ELE_KP,ELE_KI;
extern float Omni_Car_CCD_KP,Omni_Car_CCD_KI;
extern float Omni_Car_ELE_KP,Omni_Car_ELE_KI; 
extern float Tank_Car_CCD_KP,Tank_Car_CCD_KI;
extern float Tank_Car_ELE_KP,Tank_Car_ELE_KI; 

void systemInit(void);

/***Macros define***/ /***�궨��***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//����(1000/100hz=10)������������С�������˶�
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//�������ͺ�����������Divisor_Mode��ֵ��Ŀǰ��6��С������
#define CAR_NUMBER    6      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***�궨��***/

//C library function related header file
//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 