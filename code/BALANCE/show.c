#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int Time_count;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
�������ܣ���ȡ��ص�ѹ�������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));//This task runs at 10Hz //��������10Hz��Ƶ������
		
		//����ʱ���������ݷ�������������
		//The buzzer will beep briefly when the machine is switched on
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //����������
		else if(Buzzer_count==5)Buzzer=0;
		
		//Read the battery voltage //��ȡ��ص�ѹ
		for(i=0;i<10;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/10;
		Voltage_All=0;
		 
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5V���͵���ʱ��������һ�α���
		else if(Voltage<10 && LowVoltage_1<2)LowVoltage_2++; //10V, when the car is not allowed to control, the buzzer will alarm the second time //10V��С����ֹ����ʱ�������ڶ��α���
					
		APP_Show();	 //Send data to the APP //��APP��������
	  oled_show(); //Tasks are displayed on the screen //��ʾ����ʾ����
   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{  
   static int count=0;	 
	 int Car_Mode_Show;
	
	 //Collect the tap information of the potentiometer, 
	 //and display the car model to be fitted when the car starts up in real time
	 //�ɼ���λ����λ��Ϣ��ʵʱ��ʾС������ʱҪ�����С���ͺ�
	 Divisor_Mode=2048/CAR_NUMBER+5;
	 Car_Mode_Show=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode);	
	 if(Car_Mode_Show>5)Car_Mode_Show=5;
	 Voltage_Show=Voltage*100; 
	 count++;
	
	//CCD line patrol mode display screen content display 
	//CCDѲ��ģʽ��ʾ��������ʾ
	if(Mode == CCD_Line_Patrol_Mode)											
		{
			//OLED_Clear();
				//The first line of the display shows the content // 
				//��ʾ����1����ʾ����//
				OLED_Show_CCD(); 																																			//��̬��ʾCCDͼ��
				
				//The second line of the display shows the content //
				//��ʾ����2����ʾ����//
				OLED_ShowString(00,10,"Median :");
				OLED_ShowNumber(40,10, CCD_Median ,5,12);  																						//CCD���ߵ���ֵ��Ѳ��������λ��Ϊ64
			
				switch(Car_Mode_Show)				
				{
					case Mec_Car:       OLED_ShowString(86,10,"Mec "); break; 
					case Omni_Car:      OLED_ShowString(86,10,"Omni"); break; 
					case Akm_Car:       OLED_ShowString(86,10,"Akm "); break; 
					case Diff_Car:      OLED_ShowString(86,10,"Diff"); break; 
					case FourWheel_Car: OLED_ShowString(86,10,"4WD "); break; 
					case Tank_Car:      OLED_ShowString(86,10,"Tank"); break; 
				} 
			
				//The third line of the display shows the content // 
				//��ʾ����3����ʾ����//
				OLED_ShowString(00,20,"Threshold :");						
				OLED_ShowNumber(90,20, CCD_Threshold,5,12);     //��ʾCCD��ֵ
			
				//The fourth line of the display shows the content // 
				//��ʾ����4����ʾ����//
				//��ʾ���г��͵��A��B��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
				OLED_ShowString(00,30,"A:");	
				if(MOTOR_A.Target<0)	 	 OLED_ShowString(15,30,"-"),
																 OLED_ShowNumber(25,30,-MOTOR_A.Target*1000,5,12);   				 //����ת����ʾ
				else                 	   OLED_ShowString(15,30,"+"),
																 OLED_ShowNumber(25,30, MOTOR_A.Target*1000,5,12);    			//���Ŀ��ת��
				OLED_ShowString(60,30,"B:");
				if(MOTOR_B.Target<0)	   OLED_ShowString(75,30,"-"),
																 OLED_ShowNumber(85,30,-MOTOR_B.Target*1000,5,12);   			  //����ת����ʾ
				else                     OLED_ShowString(75,30,"+"),
																 OLED_ShowNumber(85,30, MOTOR_B.Target*1000,5,12);    			//���Ŀ��ת��	
				
				//The 5th line of the display shows the content // 
				//��ʾ����5����ʾ����//
				if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
				{
					//���ֳ�����������ʾ���C��D��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
					OLED_ShowString(00,40,"C:");
					if(MOTOR_C.Target<0)	   OLED_ShowString(15,40,"-"),
																	 OLED_ShowNumber(25,40,-MOTOR_C.Target*1000,5,12);    		//����ת����ʾ
					else                     OLED_ShowString(15,40,"+"),
																   OLED_ShowNumber(25,40, MOTOR_C.Target*1000,5,12);    		//���Ŀ��ת��
						
					OLED_ShowString(60,40,"D:");
					if(MOTOR_D.Target<0)	   OLED_ShowString(75,40,"-"),
																	 OLED_ShowNumber(85,40,-MOTOR_D.Target*1000,5,12);    		//����ת����ʾ
					else                   OLED_ShowString(75,40,"+"),
																	 OLED_ShowNumber(85,40, MOTOR_D.Target*1000,5,12);    		//���Ŀ��ת��
					}
			
				else if(Car_Mode==Akm_Car)
				{
					//������С����ʾ�����PWM����ֵ//
					OLED_ShowString(00,40,"SERVO:");
					if( Servo<0)		      OLED_ShowString(60,40,"-"),
																OLED_ShowNumber(80,40,-Servo,4,12);
					else               	OLED_ShowString(60,40,"+"),
																OLED_ShowNumber(80,40, Servo,4,12); 		
					}	
				//ȫ���ֳ���ʾ���C��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
				else if(Car_Mode==Omni_Car)
				{
					OLED_ShowString(00,40,"C:");
					if(MOTOR_C.Target<0)	   OLED_ShowString(15,40,"-"),	
																	 OLED_ShowNumber(25,40,-MOTOR_C.Target*1000,5,12);   	  //����ת����ʾ
					else                     OLED_ShowString(15,40,"+"),
																   OLED_ShowNumber(25,40, MOTOR_C.Target*1000,5,12);    	//���Ŀ��ת��
				}
				
				else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
				{
					//����С�����Ĵ�����ʾ���ҵ����PWM����ֵ
																	 OLED_ShowString(00,40,"MA");
					 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
																	 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
					 else                 	 OLED_ShowString(20,40,"+"),
																	 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
																	 OLED_ShowString(60,40,"MB");
					 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
																	 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
					 else                 	 OLED_ShowString(80,40,"+"),
																	 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
				 }
			 
	}
		//Display content on the display in electromagnetic line patrol mode 
		//���Ѳ��ģʽ��ʾ��������ʾ
		else if(Mode == ELE_Line_Patrol_Mode)							
		{	
			//OLED_Clear();
						//The first line of the display shows the content // 
					 //��ʾ����1����ʾ����//
					 switch(Car_Mode_Show)
					 {
						case Mec_Car:       OLED_ShowString(00,00,"Mec "); break; 
						case Omni_Car:      OLED_ShowString(00,00,"Omni"); break; 
						case Akm_Car:       OLED_ShowString(00,00,"Akm "); break; 
						case Diff_Car:      OLED_ShowString(00,00,"Diff"); break; 
						case FourWheel_Car: OLED_ShowString(00,00,"4WD "); break; 
						case Tank_Car:      OLED_ShowString(00,00,"Tank"); break; 
					 }				
					 OLED_ShowString(65,00,"L:");
					 OLED_ShowNumber(80,00,Sensor_Left,5,12);	 																	 //��ߵ�е�����
					  
		//else if(Mode==ELE_Line_Patrol_Mode)		OLED_ShowString(50,0,"ELE    ");
					 //The second line of the display shows the content //
					  //��ʾ����2����ʾ����//
					 OLED_ShowString(00,10,"M:");
					 OLED_ShowNumber(20,10,Sensor_Middle,5,12); 																//�м��е�����
					 OLED_ShowString(60,10,"R:");
					 OLED_ShowNumber(80,10,Sensor_Right,5,12);  																//�ұߵ�е�����
					  
					 //The third line of the display shows the content //
					  //��ʾ����3����ʾ����//
					 OLED_ShowString(00,20,"Devia:");
					 OLED_ShowNumber(40,20,Sensor,5,12);		    																//ƫ��ֵ 
			
			//The fourth line of the display shows the content // 					 
			//��ʾ����4����ʾ����//
			if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car||Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==Omni_Car)
			{	
				//��ʾ���г��͵��A��B��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
				OLED_ShowString(00,30,"A:");	
				if(MOTOR_A.Target<0)	 	 OLED_ShowString(15,30,"-"),
																 OLED_ShowNumber(25,30,-MOTOR_A.Target*1000,5,12);    //����ת����ʾ
				else                 	   OLED_ShowString(15,30,"+"),
																 OLED_ShowNumber(25,30, MOTOR_A.Target*1000,5,12);    //���Ŀ��ת��
				OLED_ShowString(60,30,"B:");
				if(MOTOR_B.Target<0)	   OLED_ShowString(75,30,"-"),
																 OLED_ShowNumber(85,30,-MOTOR_B.Target*1000,5,12);    //����ת����ʾ
				else                     OLED_ShowString(75,30,"+"),
																 OLED_ShowNumber(85,30, MOTOR_B.Target*1000,5,12);    //���Ŀ��ת��
			}
			
			//The 5th line of the display shows the content // 
			//��ʾ����5����ʾ����//
			if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
			{
				//����С������������ʾ���C��D��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
				OLED_ShowString(00,40,"C:");
				if(MOTOR_C.Target<0)	   OLED_ShowString(15,40,"-"),
																 OLED_ShowNumber(25,40,-MOTOR_C.Target*1000,5,12);    //����ת����ʾ
				else                     OLED_ShowString(15,40,"+"),
																 OLED_ShowNumber(25,40, MOTOR_C.Target*1000,5,12);    //���Ŀ��ת��
				
				OLED_ShowString(60,40,"D:");
				if(MOTOR_D.Target<0)	   OLED_ShowString(75,40,"-"),
																 OLED_ShowNumber(85,40,-MOTOR_D.Target*1000,5,12);    //����ת����ʾ
				else                     OLED_ShowString(75,40,"+"),
																 OLED_ShowNumber(85,40, MOTOR_D.Target*1000,5,12);    //���Ŀ��ת��	
			}
			
			else if(Car_Mode==Akm_Car)
			{
				//������С����ʾ�����PWM����ֵ//
				OLED_ShowString(00,40,"SERVO:");
				if( Servo<0)		     		 OLED_ShowString(60,40,"-"),
																 OLED_ShowNumber(80,40,-Servo,4,12);
				else                 		 OLED_ShowString(60,40,"+"),
																 OLED_ShowNumber(80,40, Servo,4,12); 		
			}	
			else if(Car_Mode==Omni_Car)
			{
				OLED_ShowString(00,40,"C:");
				if(MOTOR_C.Target<0)	   OLED_ShowString(15,40,"-"),
																 OLED_ShowNumber(25,40,-MOTOR_C.Target*1000,5,12);    //����ת����ʾ
				else                     OLED_ShowString(15,40,"+"),
																 OLED_ShowNumber(25,40, MOTOR_C.Target*1000,5,12);    //���Ŀ��ת��
			}
			
			else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //����С�����Ĵ�����ʾ���ҵ����PWM����ֵ//
															 OLED_ShowString(00,40,"MA");
			 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
															 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(20,40,"+"),
															 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
															 OLED_ShowString(60,40,"MB");
			 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
															 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(80,40,"+"),
															 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
		 }
		}	

	//APP Bluetooth mode display content display// 
	//APP����ģʽ��ʾ��������ʾ//
	else											
		{
			//OLED_Clear();
			//The first line of the display shows the content // 
		 //��ʾ����1����ʾ����//
		 switch(Car_Mode_Show)
		 {
			case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; 
			case Omni_Car:      OLED_ShowString(0,0,"Omni"); break; 
			case Akm_Car:       OLED_ShowString(0,0,"Akm "); break; 
			case Diff_Car:      OLED_ShowString(0,0,"Diff"); break; 
			case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; 
			case Tank_Car:      OLED_ShowString(0,0,"Tank"); break; 
		 }
		 
//		OLED_ShowString(40,0, "Mode:");	//��ʾģʽ
//		if(Mode==Normal_Mode)					OLED_ShowString(80,0,"Normal ");
//		else if(Mode==Lidar_Avoid_Mode)			OLED_ShowString(80,0,"Avoid  ");
//		else if(Mode==Lidar_Follow_Mode)		OLED_ShowString(80,0,"Follow ");
		//else if(Mode==ELE_Line_Patrol_Mode)		OLED_ShowString(50,0,"ELE    ");
		 //The second line of the display shows the content // 
		 //��ʾ����2����ʾ����//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//���֡�ȫ���֡���������ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
			OLED_ShowString(0,10,"A");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
														OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,10,"+"),
														OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
			
			if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
														OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,10,"+"),
														OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
		 }
		 
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //�����������١��Ĵ�����ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
			 OLED_ShowString(0,10,"L:");
			 if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
															OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			 else                 	OLED_ShowString(15,10,"+"),
															OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12);  
			 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,10,"-"),
															OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,10,"+"),
															OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
		 }
		 
				//The third line of the display shows the content // 
			 //��ʾ����3����ʾ����//
			 //�����������١��Ĵ�����ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
			 OLED_ShowString(0,20,"R:");
			 if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			 else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20,  MOTOR_B.Target*1000,5,12);  
				
			 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
		 

			if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
			{
			//���֡�ȫ���֡���������ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
			OLED_ShowString(0,20,"B");		
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
			
			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
			}
		 
		  //The fourth line of the display shows the content // 
			//��ʾ����4����ʾ����//
			//���֡���������ȫ������ʾ���C��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
			 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
			 {
														OLED_ShowString(0,30,"C");
			if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
														OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
														OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
				
			if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
														OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
														OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
				}
		  if(Car_Mode==Akm_Car)
		 {
				//������С����ʾ�����PWM����ֵ//
				OLED_ShowString(00,30,"SERVO:");
				if( Servo<0)		      OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(80,30,-Servo,4,12);
				else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(80,30, Servo,4,12); 
		 }
		 	 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //����С�����Ĵ�����ʾ������PWM����ֵ//
															 OLED_ShowString(00,30,"MA");
			 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(40,30,"-"),
															 OLED_ShowNumber(50,30,-MOTOR_A.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(40,30,"+"),
															 OLED_ShowNumber(50,30, MOTOR_A.Motor_Pwm,4,12); 
		 }	
		 
		 //The 5th line of the display shows the content // 
		 //��ʾ����5����ʾ����//
		 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
		 {
				//����С����ʾ���D��Ŀ���ٶȺ͵�ǰʵ���ٶ�//
				OLED_ShowString(0,40,"D");
				if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
															OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
				else                 	OLED_ShowString(15,40,"+"),
															OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
				if( MOTOR_D.Encoder<0)OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
		 }

		 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //����С�����Ĵ�����ʾ�ҵ����PWM����ֵ//
															 OLED_ShowString(00,40,"MB");
			 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(40,40,"-"),
															 OLED_ShowNumber(50,40,-MOTOR_B.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(40,40,"+"),
															 OLED_ShowNumber(50,40, MOTOR_B.Motor_Pwm,4,12);
		 }			 
	}

	
			//The 6th line of the display shows the content // 
			//��ʾ����6����ʾ����// 
			//Display the current control mode 
		 //��ʾ��ǰ����ģʽ//
		 if(Mode==CCD_Line_Patrol_Mode)         OLED_ShowString(0,50,"CCD  ");
		 else if (Mode==ELE_Line_Patrol_Mode)   OLED_ShowString(0,50,"ELE  ");
	   else if(Mode==Lidar_Avoid_Mode)     OLED_ShowString(0,50,"AVO");
	   else if(Mode==Lidar_Follow_Mode) OLED_ShowString(0,50,"Fol");
	   else if(Mode==Lidar_Along_Mode) OLED_ShowString(0,50,"Alo");
	   else if(Mode==PS2_Control_Mode) OLED_ShowString(0,50,"PS2");
		 else if(Mode==APP_Control_Mode)OLED_ShowString(0,50,"APP");
	
			
		 //��ʾ��ǰС���Ƿ��������//
		 if(EN==1&&Flag_Stop==0)   	OLED_ShowString(45,50,"O N");  
		 else                      	OLED_ShowString(45,50,"OFF"); 
			
																OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
			                          OLED_ShowString(88,50,".");
																OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
			                          OLED_ShowString(110,50,"V");
		 if(Voltage_Show%100<10) 		OLED_ShowNumber(92,50,0,2,12);
		
    //OLED_Clear();		
		OLED_Refresh_Gram();
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //�Ե�ص�ѹ����ɰٷֱ���ʽ
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //�����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //���ڽ����ӡAPP���ݺ���ʾ����
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {	 
		 if(Mode == ELE_Line_Patrol_Mode)			//���Ѳ�ߵ���
		 {
				//Send parameters to the APP, the APP is displayed in the debug screen
				//���Ͳ�����APP��APP�ڵ��Խ�����ʾ
				printf("{C%d:%d:%d}$",(int)RC_Velocity_ELE,(int)ELE_KP,(int)ELE_KI);
		 }
		 else if(Mode == CCD_Line_Patrol_Mode)		//CCDѲ�ߵ���
		 {
				//Send parameters to the APP, the APP is displayed in the debug screen
				//���Ͳ�����APP��APP�ڵ��Խ�����ʾ
				printf("{C%d:%d:%d}$",(int)RC_Velocity_CCD,(int)CCD_KP,(int)CCD_KI);
		 }
		 else if(Mode == Lidar_Along_Mode)  //��ֱ��ģʽ��APP����PID����
		 {
			 if(Car_Mode == Akm_Car)
			    printf("{C%d:%d:%d}$",(int)Akm_Along_Distance_KP,(int)Akm_Along_Distance_KD,(int)Akm_Along_Distance_KI);
			 else if(Car_Mode == Diff_Car)
				 printf("{C%d:%d:%d}$",(int)Diff_Along_Distance_KP,(int)Diff_Along_Distance_KD,(int)Diff_Along_Distance_KI);
			 else if(Car_Mode == FourWheel_Car)
				 printf("{C%d:%d:%d}$",(int)FourWheel_Along_Distance_KP,(int)FourWheel_Along_Distance_KD,(int)FourWheel_Along_Distance_KI);
			 else
				 printf("{C%d:%d:%d}$",(int)Along_Distance_KP,(int)Along_Distance_KD,(int)Along_Distance_KI);
		 }
		  else if(Mode == Lidar_Follow_Mode)   //����ģʽ��APP����PID�������
		 {
			 printf("{C%d:%d:%d:%d:%d:%d}$",(int)Distance_KP,(int)Distance_KD,(int)Distance_KI,(int)Follow_KP,(int)Follow_KD,(int)Follow_KI);
		 }
		 
		 else
		 {
				//Send parameters to the APP, the APP is displayed in the debug screen
				//���Ͳ�����APP��APP�ڵ��Խ�����ʾ
				//printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);
				printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);
			 // printf("{B%d}$",(int)PointDataProcess[i].distance);
		 }
		 
		 PID_Send=0;	
		  
	 }

    else if(flag_show==0)
		 {
			 //Send parameters to the APP and the APP will be displayed on the front page
			 //���Ͳ�����APP��APP����ҳ��ʾ
		   printf("{A%d:%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(u8)Left_Figure,(u8)Right_Figure);
		 }
		 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //���Ͳ�����APP��APP�ڲ��ν�����ʾ������Ҫ��ʾ�Ĳ��������Ӧ��λ�ü��ɣ���������ʾ5������
	   printf("{B%d:%d:%d}$",(int)RC_Velocity,(u8)Left_Figure,(u8)Right_Figure);
	
	 }
 		 
}

//OLED��ʾCCD����
void OLED_Show_CCD(void)
{ 
	 u8 i,t;
	 for(i = 0;i<128; i++)
  {
		if(ADV[i]<CCD_Threshold) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
  }
}

//OLED��ʾCCD���㺯��
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}

//������ʾһ�ε�����
//void oled_show_once(void)
//{
//   OLED_ShowString(0,00,"Turn Right Wheel");  //ת������
//   OLED_ShowString(0,10,"TO Select Mode"); //ѡ��ģʽ
//	 OLED_ShowString(0,20,"Current Mode Is");//��ǰ��ģʽ�ǣ�
//	if(ELE_ON_Flag==1)				  OLED_ShowString(50,30,"ELE");//���Ѳ��ģʽ
//	if(CCD_ON_Flag==1)				  OLED_ShowString(50,30,"CCD");//CCDģʽ
//	if(APP_ON_Flag==1)				  OLED_ShowString(50,30,"APP");//����ģʽ����APP��PS2��Remote
//	if(Along_wall==1)				    OLED_ShowString(50,30,"WAL");
//	if(Follow_ON_Flag==1)       OLED_ShowString(50,30,"FOL");
//	if(Avoid_ON_Flag==1)        OLED_ShowString(50,30,"Avo");
//	if(PS2_ON_Flag==1)        OLED_ShowString(50,30,"Ps2");
//	
//	OLED_ShowString(0,40,"Press User Key");// ��һ���û�����
//  OLED_ShowString(0,50,"TO End Selection");//����ѡ��
//	OLED_Refresh_Gram();	//OLEDˢ��
//}

