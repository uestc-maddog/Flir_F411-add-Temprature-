/*-----------------------------------------------------------------
 * Name:    key.c   
 * Purpose: key driver  
 *-----------------------------------------------------------------
 * 
 * Copyright (c) *reserve
 
||                       _      _               ||
||    /\  /\  __  _  __ | | __ | | ____  ___ _  ||
||   /  \/  \/ _ ` |/ _ ` |/ _ ` |/ _  \/ _ ` | ||
||  / /\  /\  (_|    (_|    (_|    (_)   (_)  | ||
|| /_/  \/  \_.__, |\__, _|\__, _|\____/\___. | ||
|| =====================================|____/  ||
||                                              ||

 -----------------------------------------------------------------*/
 
/********************************************************************************************************
 *                                               INCLUDES
 ********************************************************************************************************/
#include "stm32f4xx_hal.h"
#include "key.h"
#include "electricity.h"
#include "flir_lcd.h"
/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               GLOBAL VARIABLES
 ********************************************************************************************************/
uint8_t  Key_Up = 0;
uint8_t  Key_Down = 0;
uint32_t Time_1ms = 0;                // ����ʱ�䲶��
bool return_mark = false;
volatile uint8_t Time_Sleep = 0;      // Sleep Time counter

/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern uint8_t Charge_Flag;
 
/********************************************************************************************************
 *                                               EXTERNAL FUNCTIONS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               LOCAL VARIABLES
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               PUBLIC FUNCTIONS
 ********************************************************************************************************/
 
/*********************************************************************
 * @fn        Key_Scan()
 *
 * @brief     ����ɨ������ 
 *
 * @param     None
 *
 * @return    Key_None��������Ч   Key_Short�������̰�   Key_Long����������
 */
KeyStatus Key_Scan(void)
{
	if(Key_Down == 1)
	{
		if(Time_1ms > Long_Thre && return_mark == false)       
		{
			return_mark = true;
			//Time_1ms = 0;          // ������ʱ��ǰ������
			return Key_Long;         // ��������ʱ�����2s      ����
		}
	}
	if(Key_Up == 1)        // �а�������
	{
		Key_Up = 0;
		Key_Down = 0;
//		if(Time_1ms > Long_Thre)       
//		{
//			//Time_1ms = 0;          // ������ʱ��ǰ������
//			return Key_Long;         // ��������ʱ�����2s      ����
//		}
//		else 
		if(Time_1ms > Short_Thre && return_mark == false) 
		{
			//Time_1ms = 0;
			return Key_Short;        // ��������ʱ��С��2s      �̰� 
		}		
		else if(Time_1ms > Long_Thre && return_mark == false) 
		{
			//Time_1ms = 0;
			return Key_Long;        // ��������ʱ��da��2s        ���� 
		}		
		else                           
		{
			return_mark = false;
			//Time_1ms = 0;
			return Key_None;         // ������������
		}
	}
	return Key_None;
}
/*********************************************************************
 * @fn        HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)()
 *
 * @brief     TIM2 detection callbacks.
 *
 * @param     htim�����ö�ʱ���ṹ��   
 *
 * @return    None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	if(htim->Instance == TIM2) Time_1ms++; 
	if(htim->Instance == TIM3) 
	{
		Time_Sleep++;
		//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	}
}
/*********************************************************************
 * @fn        HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)()
 *
 * @brief     EXTI line detection callbacks.
 *
 * @param     GPIO_Pin: Specifies the pins connected EXTI line  
 *
 * @return    None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint8_t CAPTURE_STA = 0;	   // ����״̬
	static uint8_t Temp = 0;	       
	uint32_t timer = 36000000;
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
		/* NOTE: This function Should not be modified, when the callback is needed,
						 the HAL_GPIO_EXTI_Callback could be implemented in the user file
		 */
	if(GPIO_Pin == GPIO_PIN_0)           // PBSTAT�ж�
	{
//		HAL_Delay(500);HAL_Delay(500);HAL_Delay(500);
		if(!(GPIOB->IDR&0x0001))           // PB0�½���    
		{
			//Temp++;
			if(flir_conf.file_sys_LowPower == Not_LowPower) 
			{
				HAL_Delay(500);HAL_Delay(500);HAL_Delay(500);
				if(!(GPIOB->IDR&0x0001)) {
				flir_conf.file_sys_LowPower = Is_LowPower;        // ״̬�л�
				setSandby();                     // ����͹���ģʽ
				}
			}
			else
			{
//				HAL_Delay(500);HAL_Delay(500);
//				while(--timer)   ;
//				if(!(GPIOB->IDR&0x0001))           // PB0�½��� 			
				flir_conf.file_sys_LowPower = Not_LowPower;       // ״̬�л�
				flir_conf.file_sys_PBWakeup = PBWakeup_Down;      // ���PBSTA�������Ѽ�����
				Save_Parameter();                                 // ����9��ϵͳ������FLASH
				/*  SoftReset  */
				HAL_PWREx_DisableFlashPowerDown();
				__set_FAULTMASK(1);                               // �ر������ж�
				NVIC_SystemReset();                               // �����λ
			}
		}
	}
	if(GPIO_Pin == GPIO_PIN_12)          // �����ж�
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		
		if(CAPTURE_STA == 1)               // �Ѿ����񵽰�������
		{
			GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			
			HAL_TIM_Base_Stop_IT(&htim2);    // �������£��رն�ʱ��TIM2
			Key_Up = 1;
			CAPTURE_STA = 0;
		}
		else                               // ��δ���񵽰�������
		{
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;   
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			
			Key_Down = 1;
			Time_1ms = 0;
			HAL_TIM_Base_Start_IT(&htim2);   // �������£�������ʱ��TIM2
			CAPTURE_STA = 1;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_15)     // ����ж�
	{
		if(!(GPIOA->IDR&0x8000))           // �½��أ�������
		{
			if(flir_conf.file_sys_LowPower == Not_LowPower)  // ��Stopģʽ��������ģʽ
			{
				flir_conf.file_sys_chargingMode = charging;
			}
			else
			{
				flir_conf.file_sys_LowPower = Not_LowPower;       // ״̬�л�

				/*  SoftReset  */
				HAL_PWREx_DisableFlashPowerDown();
				__set_FAULTMASK(1);                               // �ر������ж�
				NVIC_SystemReset();                               // �����λ
			}
		}
		else                                // �Ͻ��أ��˳����
		{
			flir_conf.file_sys_chargingMode = normal; 
			Charge_Flag = 0;
		}
	}
}
 /********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

/*********************************************************************
 * @fn      
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 */
 
 


/*********************************************************************
 */
