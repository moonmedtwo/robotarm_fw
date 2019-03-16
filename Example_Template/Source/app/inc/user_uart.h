#ifndef USER_UART__
#define USER_UART__

#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stdint.h>
#include <stdlib.h>

#include "user_timing.h"

/*
 * PROTOCOL:
 * HEADER | LEN | DATA                        | CRC
 *     1B |  1B | DATA[0]:CMD, DATA[1]->[n-1] |  1B
 */
 
#define HEADER_STM32    0xCC
#define HEADER_APP      0xBB

#define MAX_DATA_LENGTH 128

#define UART_CHECKCRC 			0x0001
#define UART_BADCRC         0x0002
#define UART_EXCEEDLENGTH	  0x0004
#define UART_NEEDRESTART		0x0008

#define OVERLOAD_SIZE (uint8_t) 3u

#define TRUE 					(uint8_t) 1u
#define FALSE 				(uint8_t) 0u

#define ACK						0xEE

typedef enum usartRcvState
{
  PACKET_HEADER = 0x00,
  PACKET_LEN    = 0x01,
  PACKET_DATA   = 0x02,
  PACKET_CRC    = 0x03,
}	usartRcvState_t;

typedef enum protocolCommand 
{
	// FROM PC -> MCU
  CMD_MOVETOXYZ			= 0x00,
  CMD_GRAB          = 0x01,
  CMD_DROP          = 0x02,
  CMD_STOP          = 0x03,
	
	// FROM MCU -> PC
  CMD_ENDPOINT			= 0x04,
	CMD_OUTOFRANGE		= 0x05,
	
  NUMB_OF_CMD,
  CMD_ERROR         = 0xFF,
}	protocolCmd_t;

typedef struct usartCtrl
{
    USART_TypeDef * hUART;
	
    uint8_t 	storedData[128];	// array to store data from protocol
    uint8_t * storedData_Cnt; 	
    uint8_t 	lenData_Cnt;		
    uint8_t 	lenData;			// length of received data
    uint8_t 	rcvCRC;				// received CRC byte	
    uint32_t 	Status;	
	
    usartRcvState_t State;	// received state sequence: HEADER -> LENGTH -> DATA -> CRC
}	usartCtrl_t;


 
extern usartCtrl_t USART2_Ctrl;
extern uint16_t USART_STATUS;
/*
 * @brief: handler for incoming data
 * @param[in]: d last received byte
 */
void USART_RX_Handler(uint8_t d, usartCtrl_t * pCtrl);

/*
 * @brief: reset params
 * @brief: act as uart timeout, timeout is equal to loop rate
 */
void usart_Ctrl_Reset(usartCtrl_t * pCtrl);

/*
 * @brief: initialize params for uart controller;
 */
void usart_Ctrl_Init(usartCtrl_t *pCtrl, USART_TypeDef * hUART);

/*
 * @brief: handler received data
 * @param[in1]: pCtrl - usart_controller in which data is received 
 * @param[in2]: pBuf - holder for data received 
 * @return: cmd 
 */
protocolCmd_t parseCommand_DataUSART(usartCtrl_t *pCtrl, uint8_t **ppBuf);

/*
 * @brief: handler received data
 */
// protocolCmd_t processDataUart(usartCtrl_t *pCtrl,uint8_t *pBuf);

/*
 * @brief: pack and send packet
 */
void sendDataPackage_USART(uint8_t *pData, uint8_t len, usartCtrl_t *pCtrl);
/*
 * @brief: wrapper for USART_SEND
 */
void sendData_USART(uint8_t *pData, uint8_t len, USART_TypeDef * hUART);
/*
 * @brief: util function
 */
uint8_t user_checkFlag(uint32_t *status,uint32_t flag);

void user_setFlag(uint32_t *status, uint32_t flag);

void user_clearFlag(uint32_t *status, uint32_t flag);

/*
 * @brief: util function
 */

#endif //USER_UART__
