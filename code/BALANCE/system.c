/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-9-26

Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0               
Update：2022-6-20

All rights reserved
***********************************************/

#include "system.h"

//Robot software fails to flag bits
//机器人软件失能标志位
u8 Flag_Stop=0;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADC值分段变量，取决于小车型号数量，目前有6种小车型号
int Divisor_Mode;

// Robot type variable
//机器人型号变量
//0=Mec_Car，1=Omni_Car，2=Akm_Car，3=Diff_Car，4=FourWheel_Car，5=Tank_Car
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
int Servo;  

//Default speed of remote control car, unit: mm/s
//遥控小车的默认速度，单位：mm/s
float RC_Velocity=500; 

//The Tank_Car CCD related variables
//全向小车CCD相关变量
float Tank_Car_CCD_KP=35,Tank_Car_CCD_KI=140;

//The Tank_Car ELE related variables
//全向小车ELE相关变量
float Tank_Car_ELE_KP=25,Tank_Car_ELE_KI=200;

//The Omni_Car CCD related variables
//全向小车CCD相关变量
float Omni_Car_CCD_KP=10,Omni_Car_CCD_KI=110;

//The Omni_Car ELE related variables
//全向小车ELE相关变量
float Omni_Car_ELE_KP=9,Omni_Car_ELE_KI=100;

//The other CCD related variables
//其他小车CCD相关变量
float CCD_KP=37,CCD_KI=140;

//The other ELE related variables
//其他小车ELE相关变量
float ELE_KP=30,ELE_KI=200;

//The PS2 gamepad controls related variables
//PS2手柄控制相关变量
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//速度控制PID参数
float Velocity_KP=700,Velocity_KI=700; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;  

//The parameter structure of the motor
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus; 
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1 communication control flag bit.
//These 5 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2手柄、蓝牙APP、航模手柄、CAN通信、串口1通信控制标志位。这5个标志位默认都为0，代表串口3控制模式
u8 CCD_ON_Flag=0, APP_ON_Flag=0, ELE_ON_Flag=0, Avoid_ON_Flag=0, Follow_ON_Flag=0, Along_wall=0, PS2_ON_Flag=0; 

//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send; 

void systemInit(void)
{       
	//Interrupt priority group setting
	//中断优先级分组设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//Delay function initialization
	//延时函数初始化
	delay_init(168);			                                            
	
	//Initialize the hardware interface connected to the LED lamp
	//初始化与LED灯连接的硬件接口
	LED_Init();                     
	    
  //Initialize the hardware interface connected to the buzzer	
  //初始化与蜂鸣器连接的硬件接口
	Buzzer_Init();  
	
	//Initialize the hardware interface connected to the enable switch
	//初始化与使能开关连接的硬件接口
	Enable_Pin();

  //Initialize the hardware interface connected to the OLED display
  //初始化与OLED显示屏连接的硬件接口	
	OLED_Init();     
	
	//Initialize the hardware interface connected to the user's key
	//初始化与用户按键连接的硬件接口
	KEY_Init();	
	
	//Serial port 1 initialization, communication baud rate 115200, 
	//串口1初始化，通信波特率115200
	uart1_init(115200);	  
	
	//Serial port 2 initialization, communication baud rate 9600, 
	//used to communicate with Bluetooth APP terminal
	//串口2初始化，通信波特率9600，用于与蓝牙APP端通信
	uart2_init(9600);  
	
	//ADC pin initialization, used to read the battery voltage and potentiometer gear, 
	//potentiometer gear determines the car after the boot of the car model
	//ADC引脚初始化，用于读取电池电压与电位器档位，电位器档位决定小车开机后的小车适配型号
 	Adc_Init();  
  //According to the tap position of the potentiometer, determine which type of car needs to be matched, 
  //and then initialize the corresponding parameters	
  //根据电位器的档位判断需要适配的是哪一种型号的小车，然后进行对应的参数初始化	
	Robot_Select();                 
	
	LIDAR_USART_Init();
         
	//Encoder A is initialized to read the real time speed of motor A
	//编码器A初始化，用于读取电机A的实时速度
	Encoder_Init_TIM2(); 
	//Encoder B is initialized to read the real time speed of motor B
  //编码器B初始化，用于读取电机B的实时速度	
	Encoder_Init_TIM3(); 
  //Encoder C is initialized to read the real time speed of motor C  
  //编码器C初始化，用于读取电机C的实时速度	
	Encoder_Init_TIM4();
	//Encoder D is initialized to read the real time speed of motor D
  //编码器D初始化，用于读取电机D的实时速度	
	Encoder_Init_TIM5();  
	
	
//	if(Car_Mode==Akm_Car) 
	// initialize the steering gear control pin of Ackerman car,
	//初始化阿克曼小车的舵机控制引脚
	Servo_PWM_Init(9999,167);

  //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
  //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
	MiniBalance_PWM_Init(16799,0);  //高级定时器TIM10的频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k 
	
	MiniBalance_PWM11_Init(16799,0);  //高级定时器TIM11的频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k
	
	MiniBalance_PWM9_Init(16799,0);  //高级定时器TIM9的频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k 
	
	MiniBalance_PWM1_Init(16799,0);  //高级定时器TIM1的频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k
	
	//Initialize the hardware interface to the PS2 controller
	//初始化与PS2手柄连接的硬件接口
	PS2_Init();
	
	//PS2 gamepad configuration is initialized and configured in analog mode
  //PS2手柄配置初始化,配置为模拟量模式	
	PS2_SetInit();	
	 	
}
