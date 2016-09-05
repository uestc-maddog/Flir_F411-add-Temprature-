#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"                    // Device header
#include "stm32f4xx.h"                    // Device header

// FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes 
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes 

// ϵͳ�����洢
#define PARA_NUMS 9                               // ��Ҫ����Ĳ������� 
#define PARA_SAVE_ADDR  ADDR_FLASH_SECTOR_7	      // ��Ҫ����Ĳ�������ʼ��ַ
//����FLASH �����ַ(����Ϊ4�ı���������������,Ҫ���ڱ�������ռ�õ�������.
//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

// FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	// STM32 FLASH����ʼ��ַ
#define FLASH_WAITETIME  60000        // FLASH�ȴ���ʱʱ��

uint8_t STMFLASH_GetFlashSector(uint32_t addr);     //��ȡĳ����ַ���ڵ�flash����
uint32_t STMFLASH_ReadWord(uint32_t faddr);		  	                             // ������  
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);// ��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   // ��ָ����ַ��ʼ����ָ�����ȵ�����
//����д��
void Test_Write(uint32_t WriteAddr,uint32_t WriteData);	
#endif
