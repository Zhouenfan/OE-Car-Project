/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-9-26

Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update���2022-6-20

All rights reserved
***********************************************/
#include "system.h"

//Task priority    //�������ȼ�
#define START_TASK_PRIO	1

//Task stack size //�����ջ��С	
#define START_STK_SIZE 	256  

//Task handle     //������
TaskHandle_t StartTask_Handler;

//Task function   //������
void start_task(void *pvParameters);

//Main function //������
int main(void)
{ 
  systemInit(); //Hardware initialization //Ӳ����ʼ��
	
	//Create the start task //������ʼ����
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //������
							(const char*    )"start_task",          //Task name       //��������
							(uint16_t       )START_STK_SIZE,        //Task stack size //�����ջ��С
							(void*          )NULL,                  //Arguments passed to the task function //���ݸ��������Ĳ���
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //�������ȼ�
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //������    					
	vTaskStartScheduler();  //Enables task scheduling //�����������	
}
 
//Start task task function //��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //�����ٽ���
	
    //Create the task //��������
	  xTaskCreate(pstwo_task,  "pstwo_task",  PS2_STK_SIZE,  NULL, PS2_TASK_PRIO,  NULL);
	  xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//Vehicle motion control task //С���˶���������
    xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //The OLED display displays tasks //OLED��ʾ����ʾ����
    xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	//LED light flashing task //LED����˸����
	  xTaskCreate(adc_task,       "ADC_task",     ADC_STK_SIZE,     NULL, ADC_TASK_PRIO,     NULL);//ADC acquisition task 	//ADC�ɼ�����
    vTaskDelete(StartTask_Handler); //Delete the start task //ɾ����ʼ����

    taskEXIT_CRITICAL();            //Exit the critical section//�˳��ٽ���
}






