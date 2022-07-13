/*
 * LLUart.h
 *
 *  Created on: Jul 6, 2022
 *      Author: studio3s
 */

#ifndef INC_EXTINC_LLUART_H_
#define INC_EXTINC_LLUART_H_


#include <stdio.h>
#include <string.h>
#include "stm32f746xx.h"
#include "stm32f7xx_it.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_usart.h"
#include <vector>
#include <string>

void Usart_Transmit(USART_TypeDef *USARTx, std::string data);
void Usart_Receive(USART_TypeDef *USARTx);



#endif /* INC_EXTINC_LLUART_H_ */
