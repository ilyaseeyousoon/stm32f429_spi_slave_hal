/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
uint8_t	err;
/* USER CODE BEGIN 0 */
extern uint8_t receiveBuffer[32];
 uint32_t gj;
 extern uint8_t m;
 uint16_t ftp=5;
 uint32_t res[100];
/* USER CODE END 0 */
const uint8_t COMM_WIDTH = 3;
/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/
/**********************************/
/*     MAIN SPI STRUCT            */
/**********************************/
typedef enum
{
  CMD_STATE_READY             = 0x00,    /*!< Command Interpreter ready to get command           */
	CMD_STATE_BUSY             = 0x01,     /*!< Previous command not finished, new one will be neglected    */
  
}CMD_StateTypeDef;


typedef struct
{
	
  uint8_t Byte1;                  	/*!< Specifies additional data for command according to PROTOCOL 
                                           This parameter can be 0 when unused  */
	
	uint8_t Byte2;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */

	uint8_t Byte3;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */

	uint8_t Byte4;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */

	uint8_t Byte5;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */
																				 
	uint8_t Byte6;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */

	uint8_t Byte7;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */
																				 
	uint8_t Byte8;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */

	uint8_t Byte9;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */
																				 
	uint8_t Byte10;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */																					 
																					 
	uint8_t Byte11;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */	
																				 
	uint8_t Byte12;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */	
																				 
 	uint8_t Byte13;                  	/*!< Specifies additional data for command according to PROTOCOL 
																				 This parameter can be 0 when unused  */																					 
	
	CMD_StateTypeDef State;						/*!< Specifies current state of command unterpreter - READY or BUSY   */
	
}CMD_TypeDef;

/* USER CODE END 0 */
CMD_TypeDef cmd;
/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
	__HAL_SPI_DISABLE_IT(&hspi1, SPI_IT_RXNE);

  /* USER CODE BEGIN SPI1_IRQn 0 */
//	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);	
	err = HAL_SPI_Receive(&hspi1, hspi1.pRxBuffPtr, COMM_WIDTH, 100);
//res[m] = SPI1->DR; //Читаем то что пришло
//	for( gj=0;gj<1000;gj++){}
//    SPI1->DR = m; // отправляем обратно то что приняли
  /* USER CODE END SPI1_IRQn 0 */
//  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */
m=m+1;

		cmd.Byte2 = hspi1.pRxBuffPtr[1]; 		
  /* USER CODE END SPI1_IRQn 1 */
			__HAL_SPI_ENABLE_IT(&hspi1, SPI_IT_RXNE);
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
HAL_GPIO_WritePin( GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
	for(uint32_t i=0;i<100000;i++){}
HAL_GPIO_WritePin( GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
//	 HAL_UART_Receive_IT(&huart1, receiveBuffer, 1);
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
