 /**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ���� 8M����flash���ԣ�����������Ϣͨ������1�ڵ��Եĳ����ն��д�ӡ����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-MINI STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "./usart/app_usart.h"
#include "./led/bsp_led.h"
#include "./nrf24l01/app_spi_nrf24l01.h"


typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* ��ȡ�������ĳ��� */
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)
#define countof(a)      (sizeof(a) / sizeof(*(a)))
#define  BufferSize (countof(Tx_Buffer)-1)

#define  FLASH_WriteAddress     0x00000
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress

#define CLOCK 72/8 //ʱ��=72M

     

/* ���ͻ�������ʼ�� */
uint8_t Tx_Buffer[] = "��л��ѡ��Ұ��stm32������\r\n";
uint8_t Rx_Buffer[BufferSize];

__IO uint32_t DeviceID = 0;
__IO uint32_t FlashID = 0;
__IO TestStatus TransferStatus1 = FAILED;

// ����ԭ������
void Delay(__IO uint32_t nCount);
TestStatus Buffercmp(uint8_t* pBuffer1,uint8_t* pBuffer2, uint16_t BufferLength);


 
 /*------------------------------------------------------------
                         us��ʱ���� 
------------------------------------------------------------*/
void delay_us(unsigned int us)
{
	u8 n;		    
	while(us--)for(n=0;n<CLOCK;n++); 	 
}
 
 /*------------------------------------------------------------
                         ms��ʱ����
------------------------------------------------------------*/
void delay_ms(unsigned int ms)
{
	while(ms--)delay_us(1000);	 
}

/*
 * ��������main
 * ����  ��������
 * ����  ����
 * ���  ����
 */
int main(void)
{ 
	char flag = 1;
	
	LED_GPIO_Config();
	
	/* ���ô���Ϊ��115200 8-N-1 */
	USART_Config();
//	printf("\r\n ����һ��8Mbyte����flash(W25Q64)ʵ�� \r\n");
	
	/* 8M����flash W25Q64��ʼ�� */
//	SPI_FLASH_Init();
	
	/* ��ȡ Flash Device ID */
//	DeviceID = SPI_FLASH_ReadDeviceID();	
//	Delay( 200 );
	
	/* ��ȡ SPI Flash ID */
//	FlashID = SPI_FLASH_ReadID();	
//	printf("\r\n FlashID is 0x%X,\
//	Manufacturer Device ID is 0x%X\r\n", FlashID, DeviceID);
//	
//	/* ���� SPI Flash ID */
//	if (FlashID == sFLASH_ID)
//	{	
//		printf("\r\n ��⵽����flash W25Q64 !\r\n");
//		
//		/* ������Ҫд��� SPI FLASH ������FLASHд��ǰҪ�Ȳ��� */
//		// �������4K����һ����������������С��λ������
//		SPI_FLASH_SectorErase(FLASH_SectorToErase);	 	 
//		
//		/* �����ͻ�����������д��flash�� */
//		// ����дһҳ��һҳ�Ĵ�СΪ256���ֽ�
//		SPI_FLASH_BufferWrite(Tx_Buffer, FLASH_WriteAddress, BufferSize);		
//		printf("\r\n д�������Ϊ��%s \r\t", Tx_Buffer);
//		
//		/* ���ո�д������ݶ������ŵ����ջ������� */
//		SPI_FLASH_BufferRead(Rx_Buffer, FLASH_ReadAddress, BufferSize);
//		printf("\r\n ����������Ϊ��%s \r\n", Rx_Buffer);
//		
//		/* ���д�������������������Ƿ���� */
//		TransferStatus1 = Buffercmp(Tx_Buffer, Rx_Buffer, BufferSize);
//		
//		if( PASSED == TransferStatus1 )
//		{ 
//			LED2_ON;
//			printf("\r\n 8M����flash(W25Q64)���Գɹ�!\n\r");
//		}
//		else
//		{        
//			LED1_ON;
//			printf("\r\n 8M����flash(W25Q64)����ʧ��!\n\r");
//		}
//	}// if (FlashID == sFLASH_ID)
//	else// if (FlashID == sFLASH_ID)
//	{ 
//		LED1_ON;
//		printf("\r\n ��ȡ���� W25Q64 ID!\n\r");
//	}
	
	LED1(ON);
	
	while(1) {
		printf("\r\n ����һ��8Mbyte����flash(W25Q64)ʵ�� \r\n");  
//		UsartSend(str);
		delay_ms(1000);
		if(flag) {
			LED1(OFF);
			flag = 0;
		}else {
			LED1(ON);
			flag = 1;
		}
	}
}
/*
 * ��������Buffercmp
 * ����  ���Ƚ������������е������Ƿ����
 * ����  ��-pBuffer1     src������ָ��
 *         -pBuffer2     dst������ָ��
 *         -BufferLength ����������
 * ���  ����
 * ����  ��-PASSED pBuffer1 ����   pBuffer2
 *         -FAILED pBuffer1 ��ͬ�� pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return PASSED;
}

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
