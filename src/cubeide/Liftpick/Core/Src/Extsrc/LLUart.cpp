/*
 * LLUart.cpp
 *
 *  Created on: Jul 6, 2022
 *      Author: studio3s
 */

#include "Extinc/LLUart.h"



uint16_t buf_counter=0;
uint8_t uart_buf[20]={0};
uint8_t receive_condition=0;




int Usart_Write()
{
	uint32_t count=0;
	uint16_t length;
	char data[20];

	memset(data,0x00,sizeof(data));
	length=sprintf(data,"start\r\n");
	Usart_Transmit(USART6,data,length);

	while (1)
	{
	  LL_mDelay(200);
	  count++;
	  memset(data,0x00,sizeof(data));
	  length=sprintf(data,"count:%ld\r\n",count);
	  Usart_Transmit(USART6,data,length);
	}

}






int Usart_Transmit(USART_TypeDef *USARTx, char* data, uint16_t length)
{
	uint16_t i=0;
	for(i=0;i<length;i++){
		LL_USART_TransmitData8(USART6,data[i]);
		while(!LL_USART_IsActiveFlag_TXE(USART6));
	}

}





int Usart_Receive()
{
	/* USER CODE BEGIN 2 */
	LL_USART_EnableIT_RXNE(USART6);
	LL_USART_EnableIT_IDLE(USART6);
	printf("start\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
	  if(receive_condition)
	  {
		  uart_buf[buf_counter]=0x00;
		  receive_condition=0;
		  printf("str%d:%s\n\n\r",buf_counter,uart_buf);

		  buf_counter=0;
	  }
	}
}





