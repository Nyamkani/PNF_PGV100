/*
 * LLUart.cpp
 *
 *  Created on: Jul 6, 2022
 *      Author: studio3s
 */

#include "Extinc/LLUart.h"

//these four-values must be in stm32f7xx_it.h or .c
//extern uint16_t g_pnf_comm_time_5;
//extern uint16_t g_pnf_comm_time_6;

extern std::vector<uint16_t> g_pnf_read_buffer_5;
extern std::vector<uint16_t> g_pnf_read_buffer_6;

extern uint16_t g_pnf_buffer_counter_5;
extern uint16_t g_pnf_buffer_counter_6;




/*
int Usart_Write(USART_TypeDef *USARTx, std::vector<uint16_t> data)
{
	while (1)
	{
	  LL_mDelay(1);
	  uint16_t length= data.size();
	  Usart_Transmit((*USARTx),data,length);
	}

}
*/





void UsartTransmit(USART_TypeDef *USARTx, std::string data)
{
	uint16_t datalength = data.size();
	for(uint16_t i=0;i<datalength;i++){
		LL_USART_TransmitData8(USARTx, data[i]);
		while(!LL_USART_IsActiveFlag_TXE(USARTx));
	}
}





void UsartReceive(USART_TypeDef *USARTx)
{
	/* USER CODE BEGIN 2 */
	LL_USART_EnableIT_RXNE(USARTx);
	LL_USART_EnableIT_IDLE(USARTx);
	/* USER CODE END 2 */
}




//uint16_t buf_counter=0;
//uint8_t uart_buf[20]={0};
//uint8_t receive_condition=0;


