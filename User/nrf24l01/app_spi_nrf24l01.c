 /**
  ******************************************************************************
  * @file    app_spi_nrf24l01.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    20200601
  * @brief   NRF24L01 �ײ�����
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
  
#include "./nrf24l01/app_spi_nrf24l01.h"

static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;    
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���յ�ַ

/**
  * @brief  SPI_FLASH��ʼ��
  * @param  ��
  * @retval ��
  */
void SPI_NRF24L01_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	/* ʹ��SPIʱ�� */
	NRF24L01_SPI_APBxClock_FUN ( NRF24L01_SPI_CLK, ENABLE );
	
	/* ʹ��SPI������ص�ʱ�� */
 	NRF24L01_SPI_SCK_APBxClock_FUN ( NRF24L01_SPI_SCK_CLK|NRF24L01_SPI_MOSI_CLK|NRF24L01_SPI_MISO_CLK, ENABLE );

  /* ʹ��CS���ŵ�ʱ�� */
  NRF24L01_SPI_CS_APBxClock_FUN ( NRF24L01_SPI_CS_CLK, ENABLE );

  /* ʹ��CE���ŵ�ʱ�� */
  NRF24L01_CE_APBxClock_FUN ( NRF24L01_CE_CLK, ENABLE );

  /* ʹ��IRQ���ŵ�ʱ�� */
  NRF24L01_IRQ_APBxClock_FUN ( NRF24L01_IRQ_CLK, ENABLE );

  /* ����SPI�� CS���ţ���ͨIO���� */
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(NRF24L01_SPI_CS_PORT, &GPIO_InitStructure);

  /* ����CE���ţ���ͨIO���� */
  GPIO_InitStructure.GPIO_Pin = NRF24L01_CE_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);

  /* ����IRQ���ţ���ͨIO���� */
  GPIO_InitStructure.GPIO_Pin = NRF24L01_IRQ_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStructure);

  /* ����SPI�� SCK����*/
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(NRF24L01_SPI_SCK_PORT, &GPIO_InitStructure);

  /* ����SPI�� MISO����*/
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_MISO_PIN;
  GPIO_Init(NRF24L01_SPI_MISO_PORT, &GPIO_InitStructure);

  /* ����SPI�� MOSI����*/
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_MOSI_PIN;
  GPIO_Init(NRF24L01_SPI_MOSI_PORT, &GPIO_InitStructure);

  /* ֹͣ�ź�: CS���Ÿߵ�ƽ*/
  Set_SPI_CS_HIGH();

  /* SPI ģʽ���� */
  // FLASHоƬ ֧��SPIģʽ0��ģʽ3���ݴ�����CPOL CPHA
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 72M/8=9M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(NRF24L01_SPIx , &SPI_InitStructure);

  /* ʹ�� SPI  */
  SPI_Cmd(NRF24L01_SPIx , ENABLE);
	
}

  /**
  * @brief  ʹ��SPI��ȡһ���ֽڵ�����
  * @param  ��
  * @retval ���ؽ��յ�������
  */
uint8_t SPI_ReadByte(void)
{
  return (SPI_SendByte(Dummy_Byte));
}

 /**
  * @brief  ʹ��SPI����һ���ֽڵ�����
  * @param  byte��Ҫ���͵�����
  * @retval ���ؽ��յ�������
  */
uint8_t SPI_SendByte(uint8_t byte)
{
	 SPITimeout = SPIT_FLAG_TIMEOUT;
  /* �ȴ����ͻ�����Ϊ�գ�TXE�¼� */
  while (SPI_I2S_GetFlagStatus(NRF24L01_SPIx , SPI_I2S_FLAG_TXE) == RESET) {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
  }

  /* д�����ݼĴ�������Ҫд�������д�뷢�ͻ����� */
  SPI_I2S_SendData(NRF24L01_SPIx , byte);

	SPITimeout = SPIT_FLAG_TIMEOUT;
  /* �ȴ����ջ������ǿգ�RXNE�¼� */
  while (SPI_I2S_GetFlagStatus(NRF24L01_SPIx , SPI_I2S_FLAG_RXNE) == RESET) {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
  }

  /* ��ȡ���ݼĴ�������ȡ���ջ��������� */
  return SPI_I2S_ReceiveData(NRF24L01_SPIx);
}

 /**
  * @brief  дNRF24L01�Ĵ���
  * @param  reg���Ĵ�����ַ
  * @param  value��д��ֵ
  * @retval ����״ֵ̬
  */
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
  
  Set_SPI_CS_LOW();               //ʹ��SPI����
  	status = SPI_SendByte(reg);   //���ͼĴ����� 
  	SPI_SendByte(value);          //д��Ĵ�����ֵ
  Set_SPI_CS_HIGH();              //��ֹSPI����	   
  return(status);       			    //����״ֵ̬
}

 /**
  * @brief  ��NRF24L01�Ĵ���
  * @param  reg���Ĵ�����ַ
  * @retval ���ؼĴ���ֵ
  */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	
  
  Set_SPI_CS_LOW();               //ʹ��SPI����
  	SPI_SendByte(reg);            //���ͼĴ����� 
  	reg_val = SPI_ReadByte();          //���Ĵ�����ֵ
  Set_SPI_CS_HIGH();              //��ֹSPI����	   
  return(status);       			    //����״ֵ̬
}

 /**
  * @brief  ��ָ��λ��ָ�����ȵ�����
  * @param  reg���Ĵ�����ַ
  * @param  pBuf: ���뻺����ָ��
  * @param  len: ��ȡ�ĳ���
  * @retval ����״ֵ̬
  */
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;

  Set_SPI_CS_LOW();                         //ʹ��SPI����
  	status=SPI_SendByte(reg);               //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
      pBuf[u8_ctr]=SPI_ReadByte();          //��������
  Set_SPI_CS_HIGH();                        //��ֹSPI����
  return status;        //���ض�����״ֵ̬
}

 /**
  * @brief  дָ��λ��ָ�����ȵ�����
  * @param  reg���Ĵ�����ַ
  * @param  pBuf: ���뻺����ָ��
  * @param  len: ��ȡ�ĳ���
  * @retval ����״ֵ̬
  */
uint8_t NRF24L01_Write_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;
  	       
  Set_SPI_CS_LOW();                         //ʹ��SPI����
  	status=SPI_SendByte(reg);               //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
      SPI_WriteByte(*pBuf++);               //д������
  Set_SPI_CS_HIGH();                        //��ֹSPI����
  return status;        //���ض�����״ֵ̬
}

 /**
  * @brief  ����NRF24L01����һ������
  * @param  txbuf: ���뻺����ָ��
  * @retval ����״ֵ̬
  */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
  uint8_t ret;

	Set_CE_LOW();
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	Set_CE_HIGH();//��������

	while(Read_IRQ()!=0);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX) {  //�ﵽ����ط�����
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		ret = MAX_TX; 
	} else if(sta&TX_OK) {   //�������
		ret = TX_OK;
	} else {
    ret = 0xff;
  }
	return ret;
}

 /**
  * @brief  ����NRF24L01����һ������
  * @param  txbuf: ���뻺����ָ��
  * @retval ����״ֵ̬
  */
 uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;	
  uint8_t ret;
 
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	if(sta&RX_OK) { //���յ�����
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		ret = 0; 
	} else {
    ret = 1;
  }
  NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	return ret;
}		

/**
  * @brief  ������ز�������Rxģʽ
  * @param  None.
  * @retval None.
  */
void SetToRxMode(void)
{
	Set_CE_LOW();	  
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  Set_CE_HIGH(); //CEΪ��,�������ģʽ 
}	

/**
  * @brief  ������ز�������Txģʽ
  * @param  None.
  * @retval None.
  */
void SetToTxMode(void)
{
	Set_CE_LOW();	  
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  Set_CE_HIGH(); //CEΪ��,�������ģʽ 
}	

/**
  * @brief  �ȴ���ʱ�ص�����
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* �ȴ���ʱ��Ĵ���,���������Ϣ */
  FLASH_ERROR("SPI �ȴ���ʱ!errorCode = %d",errorCode);
  return 0;
}
   
/*********************************************END OF FILE**********************/
