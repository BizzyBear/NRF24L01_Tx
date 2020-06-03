 /**
  ******************************************************************************
  * @file    app_spi_nrf24l01.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    20200601
  * @brief   NRF24L01 底层驱动
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
  
#include "./nrf24l01/app_spi_nrf24l01.h"

static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;    
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接收地址

/**
  * @brief  SPI_FLASH初始化
  * @param  无
  * @retval 无
  */
void SPI_NRF24L01_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	/* 使能SPI时钟 */
	NRF24L01_SPI_APBxClock_FUN ( NRF24L01_SPI_CLK, ENABLE );
	
	/* 使能SPI引脚相关的时钟 */
 	NRF24L01_SPI_SCK_APBxClock_FUN ( NRF24L01_SPI_SCK_CLK|NRF24L01_SPI_MOSI_CLK|NRF24L01_SPI_MISO_CLK, ENABLE );

  /* 使能CS引脚的时钟 */
  NRF24L01_SPI_CS_APBxClock_FUN ( NRF24L01_SPI_CS_CLK, ENABLE );

  /* 使能CE引脚的时钟 */
  NRF24L01_CE_APBxClock_FUN ( NRF24L01_CE_CLK, ENABLE );

  /* 使能IRQ引脚的时钟 */
  NRF24L01_IRQ_APBxClock_FUN ( NRF24L01_IRQ_CLK, ENABLE );

  /* 配置SPI的 CS引脚，普通IO即可 */
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(NRF24L01_SPI_CS_PORT, &GPIO_InitStructure);

  /* 配置CE引脚，普通IO即可 */
  GPIO_InitStructure.GPIO_Pin = NRF24L01_CE_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);

  /* 配置IRQ引脚，普通IO即可 */
  GPIO_InitStructure.GPIO_Pin = NRF24L01_IRQ_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStructure);

  /* 配置SPI的 SCK引脚*/
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(NRF24L01_SPI_SCK_PORT, &GPIO_InitStructure);

  /* 配置SPI的 MISO引脚*/
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_MISO_PIN;
  GPIO_Init(NRF24L01_SPI_MISO_PORT, &GPIO_InitStructure);

  /* 配置SPI的 MOSI引脚*/
  GPIO_InitStructure.GPIO_Pin = NRF24L01_SPI_MOSI_PIN;
  GPIO_Init(NRF24L01_SPI_MOSI_PORT, &GPIO_InitStructure);

  /* 停止信号: CS引脚高电平*/
  Set_SPI_CS_HIGH();

  /* SPI 模式配置 */
  // FLASH芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
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

  /* 使能 SPI  */
  SPI_Cmd(NRF24L01_SPIx , ENABLE);
	
}

  /**
  * @brief  使用SPI读取一个字节的数据
  * @param  无
  * @retval 返回接收到的数据
  */
uint8_t SPI_ReadByte(void)
{
  return (SPI_SendByte(Dummy_Byte));
}

 /**
  * @brief  使用SPI发送一个字节的数据
  * @param  byte：要发送的数据
  * @retval 返回接收到的数据
  */
uint8_t SPI_SendByte(uint8_t byte)
{
	 SPITimeout = SPIT_FLAG_TIMEOUT;
  /* 等待发送缓冲区为空，TXE事件 */
  while (SPI_I2S_GetFlagStatus(NRF24L01_SPIx , SPI_I2S_FLAG_TXE) == RESET) {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
  }

  /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
  SPI_I2S_SendData(NRF24L01_SPIx , byte);

	SPITimeout = SPIT_FLAG_TIMEOUT;
  /* 等待接收缓冲区非空，RXNE事件 */
  while (SPI_I2S_GetFlagStatus(NRF24L01_SPIx , SPI_I2S_FLAG_RXNE) == RESET) {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
  }

  /* 读取数据寄存器，获取接收缓冲区数据 */
  return SPI_I2S_ReceiveData(NRF24L01_SPIx);
}

 /**
  * @brief  写NRF24L01寄存器
  * @param  reg：寄存器地址
  * @param  value：写入值
  * @retval 返回状态值
  */
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
  
  Set_SPI_CS_LOW();               //使能SPI传输
  	status = SPI_SendByte(reg);   //发送寄存器号 
  	SPI_SendByte(value);          //写入寄存器的值
  Set_SPI_CS_HIGH();              //禁止SPI传输	   
  return(status);       			    //返回状态值
}

 /**
  * @brief  读NRF24L01寄存器
  * @param  reg：寄存器地址
  * @retval 返回寄存器值
  */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	
  
  Set_SPI_CS_LOW();               //使能SPI传输
  	SPI_SendByte(reg);            //发送寄存器号 
  	reg_val = SPI_ReadByte();          //读寄存器的值
  Set_SPI_CS_HIGH();              //禁止SPI传输	   
  return(status);       			    //返回状态值
}

 /**
  * @brief  读指定位置指定长度的数据
  * @param  reg：寄存器地址
  * @param  pBuf: 读入缓冲区指针
  * @param  len: 读取的长度
  * @retval 返回状态值
  */
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;

  Set_SPI_CS_LOW();                         //使能SPI传输
  	status=SPI_SendByte(reg);               //发送寄存器值(位置),并读取状态值   	   
 	  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
      pBuf[u8_ctr]=SPI_ReadByte();          //读出数据
  Set_SPI_CS_HIGH();                        //禁止SPI传输
  return status;        //返回读到的状态值
}

 /**
  * @brief  写指定位置指定长度的数据
  * @param  reg：寄存器地址
  * @param  pBuf: 读入缓冲区指针
  * @param  len: 读取的长度
  * @retval 返回状态值
  */
uint8_t NRF24L01_Write_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;
  	       
  Set_SPI_CS_LOW();                         //使能SPI传输
  	status=SPI_SendByte(reg);               //发送寄存器值(位置),并读取状态值   	   
 	  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
      SPI_WriteByte(*pBuf++);               //写入数据
  Set_SPI_CS_HIGH();                        //禁止SPI传输
  return status;        //返回读到的状态值
}

 /**
  * @brief  启动NRF24L01发送一批数据
  * @param  txbuf: 读入缓冲区指针
  * @retval 返回状态值
  */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
  uint8_t ret;

	Set_CE_LOW();
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	Set_CE_HIGH();//启动发送

	while(Read_IRQ()!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX) {  //达到最大重发次数
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		ret = MAX_TX; 
	} else if(sta&TX_OK) {   //发送完成
		ret = TX_OK;
	} else {
    ret = 0xff;
  }
	return ret;
}

 /**
  * @brief  启动NRF24L01发送一批数据
  * @param  txbuf: 读入缓冲区指针
  * @retval 返回状态值
  */
 uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;	
  uint8_t ret;
 
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	if(sta&RX_OK) { //接收到数据
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		ret = 0; 
	} else {
    ret = 1;
  }
  NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	return ret;
}		

/**
  * @brief  设置相关参数进入Rx模式
  * @param  None.
  * @retval None.
  */
void SetToRxMode(void)
{
	Set_CE_LOW();	  
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  	NRF24L01_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  Set_CE_HIGH(); //CE为高,进入接收模式 
}	

/**
  * @brief  设置相关参数进入Tx模式
  * @param  None.
  * @retval None.
  */
void SetToTxMode(void)
{
	Set_CE_LOW();	  
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);       //设置RF通道为40
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
  Set_CE_HIGH(); //CE为高,进入接收模式 
}	

/**
  * @brief  等待超时回调函数
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* 等待超时后的处理,输出错误信息 */
  FLASH_ERROR("SPI 等待超时!errorCode = %d",errorCode);
  return 0;
}
   
/*********************************************END OF FILE**********************/
