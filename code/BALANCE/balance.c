#include "balance.h"

int Time_count=0; //Time variable //计时变量 

u8 Lidar_Detect = Lidar_Detect_ON;			//电磁巡线模式雷达检测障碍物，默认开启

u8 Mode;
float RC_Velocity_CCD=350,RC_Velocity_ELE=350; 
float PS2_Velocity,PS2_Turn_Velocity;			//遥控控制的速度
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //全向移动小车才开启速度平滑处理
	   
	
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
      //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //麦克纳姆轮小车
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//全向轮小车
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //没有使用到
		}
		
		//Ackermann structure car
		//阿克曼小车
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //运动学逆解
//			if(AngleR!=0)
//			{
//				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
//				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
//			}
//			else 
//			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			//}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度
			//Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.755f;
			Servo=SERVO_INIT + (Angle_Servo - 1.755f)*Ratio;

			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //舵机PWM值限幅
			}
		
		//Differential car
		//差速小车
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			
		}
		
		//FourWheel car
		//四驱车
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 		
		}
		
		//Tank Car
		//履带车
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  static u8 Count_CCD = 0;								//调节CCD控制频率
	  static u8 last_mode = 0;
	  u32 lastWakeTime = getSysTickCnt();
	  
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以100Hz的频率运行（10ms控制一次）
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			//Time count is no longer needed after 30 seconds
			//时间计数，30秒后不再需要
			if(Time_count<3000)Time_count++;
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//获取编码器数据，即车轮实时速度，并转换位国际单位
			Get_Velocity_Form_Encoder();   

        switch(click_N_Double(50))
				{
					case 1:    //单击用来切换模式
						Mode+=1;
						if(Mode == ELE_Line_Patrol_Mode)			//选择电磁巡线控制模式
						{
								ele_Init();							//初始化ELE
						}
						else if(Mode == CCD_Line_Patrol_Mode)			//选择电磁巡线控制模式
						{
								ccd_Init();							//初始化ELE
						}
					else if(Mode>6)
							 Mode = 0;
						 break;
					case 2:    //电磁巡线状态时，双击可以打开/关闭雷达检测障碍物，默认打开
						Lidar_Detect = !Lidar_Detect;
						if(Lidar_Detect == Lidar_Detect_OFF)
							memset(Dataprocess,0, sizeof(PointDataProcessDef)*225);		//用于雷达检测障碍物的数组清零
						break;				 
			  }
				
				if(last_mode != Mode)  //????????????
				{
					last_mode++;
					OLED_Clear();
					if(last_mode>6)
					{
						//OLED_Clear();
						last_mode = 0;
					}
				} 
      			
				if(Mode != ELE_Line_Patrol_Mode)
					Buzzer_Alarm(0);
			  if(Mode == APP_Control_Mode)          Get_RC();             //Handle the APP remote commands //处理APP遥控命令
			  else if(Mode == PS2_Control_Mode)     PS2_Control();        //Handle PS2 controller commands //处理PS2手柄控制命令
			  else if(Mode == Lidar_Avoid_Mode)     Lidar_Avoid();        //Avoid Mode //避障模式
			  else if(Mode == Lidar_Follow_Mode)    Lidar_Follow();       //Follow Mode //跟随模式
				else if(Mode == Lidar_Along_Mode)     Lidar_along_wall();   //Along Mode //走直线模式
				else if(Mode == ELE_Line_Patrol_Mode) 
				{	
				  Get_RC_ELE();         //ELE模�
				}
				else														//CCD模式
					{
						if(++Count_CCD == 4)									//调节控制频率，4*5 = 20ms控制一次
						{
							Count_CCD = 0;
							Get_RC_CCD();											
						}
						else if(Count_CCD>4)
							Count_CCD = 0;
					}					
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
				//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
				if(Turn_Off(Voltage)==0) 
				 { 			
           //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
					 Limit_Pwm(16500) ;
					 //Set different PWM control polarity according to different car models
					 //根据不同小车型号设置不同的PWM控制极性
					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  -MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
							case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //全向轮小车
							case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //阿克曼小车
							case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //两轮差速小车
							case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //四驱车 
							case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //履带车
					 }
				 }
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
				 else	Set_Pwm(0,0,0,0,0); 
			 	
		 }  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	if(motor_a<0)			PWMA2=16800,PWMA1=16800+motor_a;
	else 	            PWMA1=16800,PWMA2=16800-motor_a;
			
	if(motor_b<0)			PWMB1=16800,PWMB2=16800+motor_b;
	else 	            PWMB2=16800,PWMB1=16800-motor_b;
			
	if(motor_c<0)			PWMC1=16800,PWMC2=16800+motor_c;
	else 	            PWMC2=16800,PWMC1=16800-motor_c;
	
	if(motor_d<0)			PWMD2=16800,PWMD1=16800+motor_d;
	else 	            PWMD1=16800,PWMD2=16800-motor_d;
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1; 
					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:     Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


/**************************************************************************
Function: PS2_Control
Input   : none
Output  : none
函数功能：PS2手柄控制
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void PS2_Control(void)
{
	int LY,RX,LX;									//手柄ADC的值
	int Threshold=20; 							//阈值，忽略摇杆小幅度动作
	static float Key1_Count = 0,Key2_Count = 0;	//用于控制读取摇杆的速度
	//转化为128到-128的数值
	LY=-(PS2_LY-128);//左边Y轴控制前进后退
	RX=-(PS2_RX-128);//右边X轴控制转向
	LX=-(PS2_LX-128);//左边X轴控制转向,麦轮和全向小车专用
	
	if(LY>-Threshold&&LY<Threshold)	LY=0;
	if(RX>-Threshold&&RX<Threshold)	RX=0;		//忽略摇杆小幅度动作
	if(LX>-Threshold&&LX<Threshold)	LX=0;
	
	if(Strat) //按下start键才可以控制小车
	{
		if (PS2_KEY == PSB_L1) 					 	//按下左1键加速（按键在顶上）
		{	
			if((++Key1_Count) == 20)				//调节按键反应速度
			{
				PS2_KEY = 0;
			  Key1_Count = 0;
				if((PS2_Velocity += X_Step)>MAX_RC_Velocity)				//前进最大速度1230
					PS2_Velocity = MAX_RC_Velocity;
				if(Car_Mode != Akm_Car)								//非阿克曼车可调节转向速度
				{
					if((PS2_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)	//转向最大速度325
						PS2_Turn_Velocity = MAX_RC_Turn_Bias;
				}
			}
		}
		else if(PS2_KEY == PSB_R1) 					//按下右1键减速
		{
			if((++Key2_Count) == 15)
			{
				PS2_KEY = 0;
				Key2_Count = 0;
				if((PS2_Velocity -= X_Step)<MINI_RC_Velocity)			//前后最小速度110
					PS2_Velocity = MINI_RC_Velocity;
				
				if(Car_Mode != Akm_Car)								//非阿克曼车可调节转向速度
				{
					if((PS2_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)//转向最小速度45
					PS2_Turn_Velocity = MINI_RC_Turn_Velocity;
				}
			}
		}
		else
			Key2_Count = 0,Key2_Count = 0;			//读取到其他按键重新计数
		Move_X = (PS2_Velocity/128)*LY;				//速度控制，力度表示速度大小
		if(Car_Mode == Mec_Car || Car_Mode == Omni_Car)
		{
			Move_Y = LX*PS2_Velocity/128;
		}
		else
		{
			Move_Y = 0;
		}
		if(Car_Mode == Akm_Car)						//阿克曼车转向控制，力度表示转向角度
			Move_Z = (PS2_Turn_Velocity/128)*RX;	
		else										//其他车型转向控制
		{
			//if(Move_X>=0)
				Move_Z = (PS2_Turn_Velocity/128)*RX;	//转向控制，力度表示转向速度
//			else
//				Move_Z = -(PS2_Turn_Velocity/128)*RX;
		}
  }
	else
	{
		Move_X = 0;
		Move_Y = 0;
		Move_Z = 0;
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


/**************************************************************************
函数功能：CCD巡线，采集3个电感的数据并提取中线 
入口参数：无
返回  值：无
**************************************************************************/
void  Get_RC_CCD(void)
{
	static float Bias,Last_Bias;
	float move_z=0;
									
			Move_X=RC_Velocity_CCD;													//CCD巡线模式线速度
			Bias=CCD_Median-64;  //提取偏差，64为巡线的中心点
	    if(Car_Mode == Omni_Car)
			  move_z=-Bias*Omni_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Omni_Car_CCD_KI*0.1f; //PD控制，原理就是使得小车保持靠近巡线的中心点
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Tank_Car_CCD_KI*0.1f;
			else
				move_z=-Bias*CCD_KP*0.1f-(Bias-Last_Bias)*CCD_KI*0.1f;
			Last_Bias=Bias;   //保存上一次的偏差
			if(Car_Mode==Mec_Car)															
			{
				Move_Z=move_z*RC_Velocity_CCD/50000;							//差速控制原理需要经过此处处理
			}

			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_CCD/21000;							//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Akm_Car)												
			{
				Move_Z=move_z/450;																//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Diff_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/67000;					//差速控制原理需要经过此处处理	
			}
			else if(Car_Mode==Tank_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/50000;					//差速控制原理需要经过此处处理	
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/20100;					//差速控制原理需要经过此处处理
			}			
		
			//Z-axis data conversion //Z轴数据转化	
			//Unit conversion, mm/s -> m/s
			//单位转换，mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
			//Control target value is obtained and kinematics analysis is performed
			//得到控制目标值，进行运动学分析
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
函数功能：电磁巡线，采集3个电感的数据并提取中线 
入口参数：无
返回  值：无
**************************************************************************/
void  Get_RC_ELE(void)
{
	static float Bias,Last_Bias;
	float move_z=0;
	
	if(Detect_Barrier() == No_Barrier)
	{
			Move_X=RC_Velocity_ELE;				//电磁巡线模式的速度
			Bias=100-Sensor;  //提取偏差	
      if(Car_Mode == Omni_Car)		
			  move_z=-Bias* Omni_Car_ELE_KP*0.08f-(Bias-Last_Bias)* Omni_Car_ELE_KI*0.05f; 
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_ELE_KP*0.1f-(Bias-Last_Bias)*Tank_Car_ELE_KI*0.1f;
			else
				move_z=-Bias* ELE_KP*0.08f-(Bias-Last_Bias)* ELE_KI*0.05f; 
			Last_Bias=Bias; 
		  Buzzer_Alarm(0);

			if(Car_Mode==Mec_Car)															
			{		
				Move_Z=move_z*RC_Velocity_ELE/50000;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_ELE/10800;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Diff_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/45000;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Tank_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/28000;					//差速控制原理需要经过此处处理
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;
				Move_Z=move_z*RC_Velocity_ELE/20100;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Akm_Car)											
			{
				Move_Z=move_z/450;														//差速控制原理需要经过此处处理
			}
		}
	
	else									//有障碍物
	{
		Buzzer_Alarm(100);				//当电机使能的时候，有障碍物则蜂鸣器报警
		Move_X = 0;
		Move_Z = 0;
	}

			//Z-axis data conversion //Z轴数据转化	
			//Unit conversion, mm/s -> m/s
			//单位转换，mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
	
			//Control target value is obtained and kinematics analysis is performed
			//得到控制目标值，进行运动学分析
	    Move_Y=0;
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
函数功能：检测前方是否有障碍物
入口参数：无
返回  值：无
**************************************************************************/
u8 Detect_Barrier(void)
{
	u8 i;
	u8 point_count = 0;
	
	if(Lidar_Detect == Lidar_Detect_ON)
	{
		for(i=0;i<225;i++)	//检测是否有障碍物
		{
			if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))
			{
				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<700)//700mm内是否有障碍物
					point_count++;
		  }
	}
		if(point_count > 0)//有障碍物
			return Barrier_Detected;
		else
			return No_Barrier;
	}
	else
		return No_Barrier;
}

/**************************************************************************
函数功能：小车避障模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Avoid(void)
{
	u8 i = 0; 
	u8 calculation_angle_cnt = 0;	//用于判断225个点中需要做避障的点
	float angle_sum = 0;			//粗略计算障碍物位于左或者右
	u8 distance_count = 0;			//距离小于某值的计数
	for(i=0;i<225;i++)				//遍历120度范围内的距离数据，共120个点左右的数据
	{
		if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))  //避障角度在300-60之间
		{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Avoid_Distance))	//距离小于450mm需要避障,只需要120度范围内点
			{
				calculation_angle_cnt++;						 			//计算距离小于避障距离的点个数
				if(Dataprocess[i].angle<60)		
					angle_sum += Dataprocess[i].angle;
				else if(Dataprocess[i].angle>300)
					angle_sum += (Dataprocess[i].angle-360);	//300度到60度转化为-60度到60度
				if(Dataprocess[i].distance<Avoid_Min_Distance)				//记录小于200mm的点的计数
					distance_count++;
			}
	  }
	}
	Move_X = forward_velocity;
  if(calculation_angle_cnt == 0)//不需要避障
	 {
		Move_Z = 0;
	 }
	else                          //当距离小于200mm，小车往后退
	{
		if(distance_count>8)
		{
			Move_X = -forward_velocity;
			Move_Z = 0;
		}
		else
		{
			Move_X = 0;
			if(angle_sum > 0)//障碍物偏右
			{
				if(Car_Mode == Mec_Car)  //麦轮转弯需要把前进速度降低
					Move_X = 0;
				else                     //其他车型保持原有车速
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = PI/4;
				else if(Car_Mode == Omni_Car)
					Move_Z=corner_velocity;
				else
				  Move_Z=other_corner_velocity;//左转
			}
			else //偏左
			{
				if(Car_Mode == Mec_Car)
					Move_X = 0;
				else
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = -PI/4;
				else if(Car_Mode == Omni_Car)
				  Move_Z=-corner_velocity;//右转
				else
					Move_Z=-other_corner_velocity;
			}
	  }
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


/**************************************************************************
函数功能：小车跟随模式 
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Follow(void)
{
	static u16 cnt = 0;
	int i;
	int calculation_angle_cnt = 0;
	static float angle = 0;				//避障的角度
	static float last_angle = 0;		//
	u16 mini_distance = 65535;
	static u8 data_count = 0;			//用于滤除一写噪点的计数变量
	//需要找出跟随的那个点的角度
	for(i = 0; i < 225; i++)
	{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Follow_Distance))
			{
				calculation_angle_cnt++;
				if(Dataprocess[i].distance<mini_distance)
				{
					mini_distance = Dataprocess[i].distance;
					angle = Dataprocess[i].angle;
				}
			}
	}
	if(angle > 180)  //0--360度转换成0--180；-180--0（顺时针）
		angle -= 360;
	if((angle-last_angle > 10)||(angle-last_angle < -10))   //做一定消抖，波动大于10度的需要做判断
	{
		if(++data_count > 30)   //连续30次采集到的值(300ms后)和上次的比大于10度，此时才是认为是有效值
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else    //波动小于10度的可以直接认为是有效值
	{
		if(++data_count > 10)   //连续10次采集到的值(100ms后)，此时才是认为是有效值
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	if(calculation_angle_cnt < 8)  //跟随距离小于8且当cnt>40的时候，认为在1600内没有跟随目标
	{
		if(cnt < 40)
			cnt++;
		if(cnt >= 40)
		{
			Move_X = 0;
			Move_Z = 0;
		}
	}
	else
	{
		cnt = 0;
		if(Move_X > 0.06f || Move_X < -0.06f)  //当Move_X有速度时，转向PID开始调整
		{
			if(mini_distance < 700 && (last_angle > 60 || last_angle < -60))
			{
				Move_Z = -0.0098f*last_angle;  //当距离偏小且角度差距过大直接快速转向
			}
			else
			{
				  Move_Z = -Follow_Turn_PID(last_angle,0);		//转向PID，车头永远对着跟随物品
			}
		}
		else
		{
			Move_Z = 0;
		}
		if(angle>150 || angle<-150)  //如果小车在后方60°需要反方向运动以及快速转弯
		{
			Move_X = -Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);
			Move_Z = -0.0098f*last_angle;
		}
		else
		{
		  Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //保持距离保持在500mm
		}
		Move_X = target_limit_float(Move_X,-amplitude_limiting,amplitude_limiting);   //对前进速度限幅
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
函数功能：小车走直线模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_along_wall(void)
{
	static u32 target_distance=0;
	static int i,j;

	u32 distance;
	u8 data_count = 0;			//用于滤除一写噪点的计数变量
	
	Move_X = forward_velocity;  //初始速度
	
	for(j=0;j<225;j++)
	  {
			if(Dataprocess[j].angle>268 && Dataprocess[j].angle<272)   //取雷达的4度的点
			{
				if(i==0)
				{
					target_distance=Dataprocess[j].distance;  //雷达捕获第一个距离
					i++;
				}
				 if(Dataprocess[j].distance<(target_distance+limit_distance))//限制一下雷达的探测距离
				 {
					 data_count++;
					 distance=Dataprocess[j].distance;//实时距离
				 }
		  }
	  }
		if(Car_Mode == Mec_Car || Car_Mode == Omni_Car)  //只有麦轮和全向可以用Move_Y
		{
			Move_Y=Along_Adjust_PID(distance,target_distance);
			Move_X = forward_velocity;
			Move_Z = 0;
		}
		else   //其他车型使用Move_Z保持走直线状态
		{
			Move_Z=Along_Adjust_PID(distance,target_distance);
			Move_X = forward_velocity;
			Move_Y = 0;
		}
		if(data_count == 0)  //当data_count等于0，只有前进速度
			{
				Move_Y = 0;
				Move_Z = 0;
			}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


