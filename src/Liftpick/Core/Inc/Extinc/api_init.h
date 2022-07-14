/*
 * api_init.cpp
 *
 *  Created on: Jul 5, 2022
 *      Author: studio3s
 */

#ifndef INC_API_INIT_H__
#define INC_API_INIT_H_


#ifdef __cplusplus
extern "C" {
#endif



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CAN1_Init(void);
void MX_FMC_Init(void);
void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_UART5_Init(void);
void StartDefaultTask(void *argument);








#ifdef __cplusplus
}
#endif



#endif /* INC_API_INIT_CPP_ */
