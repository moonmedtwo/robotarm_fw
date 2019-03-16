#include "user_uart.h"
#include "stdlib.h"
#include "string.h"

#define UART_TIMEOUT_MS 				5

usartCtrl_t USART2_Ctrl;
uint16_t USART_STATUS = 0;

extern uint32_t msTicks;

const uint8_t CRC_8_Table[256] =
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};
/*******************************************************
 * Static function
 *******************************************************/
/*
 * @brief: Calculate Checksum
 */
static uint8_t mlsUtilsCRC8Calculate(void * buffIn, uint8_t len)
{
	uint8_t CRC_Val = 0;
	uint8_t * data = (uint8_t *)buffIn;
	while( len-- )
	{
		CRC_Val = CRC_8_Table[CRC_Val ^ * data];
		data += 1;
	}
	return CRC_Val;
}
/*
 * @brief: handler for each uart byte
 */
void USART_RX_Handler(uint8_t d,usartCtrl_t * pCtrl)
{
	switch (pCtrl->State)
	{
		case PACKET_HEADER:
		{
			if(d == HEADER_APP)
			{
				(pCtrl->State)++;
			}
		}
		break;

		case PACKET_LEN:
		{
			if(d > MAX_DATA_LENGTH)
			{
				(pCtrl->State) = PACKET_HEADER;
			}
			else
			{
				(pCtrl->lenData) 	 = d;
				(pCtrl->lenData_Cnt) = (pCtrl->lenData);
				(pCtrl->State)++;
			}
		}
		break;

		case PACKET_DATA:
		{
			*(pCtrl->storedData_Cnt)++ = d;
			if(--(pCtrl->lenData_Cnt))
			{
				// Do nothing.
			}
			else
			{
				(pCtrl->State)++;
			}
		}
		break;

		case PACKET_CRC:
		{
			(pCtrl->rcvCRC) = d;
			// Notify main loop
			(pCtrl->Status) |= UART_CHECKCRC;

			// Reset structure
			(pCtrl->State) 			= PACKET_HEADER;
			(pCtrl->storedData_Cnt) = (pCtrl->storedData);
		}
		break;

		default:
		break;
	}
}
/*
 * @brief: init usart controller params
 */
void usart_Ctrl_Init(usartCtrl_t * pCtrl, USART_TypeDef * hUART)
{
    if(!pCtrl || !hUART) 
	{
		while(1)
		{
			
		};
	}
    pCtrl->hUART = hUART;
	
    pCtrl->State = PACKET_HEADER;
    pCtrl->storedData_Cnt = pCtrl->storedData;
    pCtrl->lenData_Cnt 	= 0;
    pCtrl->lenData		= 0;
		pCtrl->rcvCRC = 0xFF;	
    pCtrl->Status = 0;	
}
/*
 * @brief: reset params
 * @brief: act as uart timeout, timeout is equal to loop rate
 */
void usart_Ctrl_Reset(usartCtrl_t * pCtrl)
{
    pCtrl->State = PACKET_HEADER;
    pCtrl->storedData_Cnt = pCtrl->storedData;
    pCtrl->lenData_Cnt = 0;
    pCtrl->lenData 	   = pCtrl->lenData_Cnt;
    pCtrl->rcvCRC = 0xFF;
    pCtrl->Status = 0x0;
	
}
/*
 * @brief: handler received data
 * @param[in1]: pCtrl - usart_controller in which data is received 
 * @param[in2]: ppBuf - pointer to holder for data received 
 * @return: cmd 
 */
protocolCmd_t parseCommand_DataUSART(usartCtrl_t * pCtrl, uint8_t ** ppBuf)
{
	/* Cant check CRC Flag -> Error. */
    if ( !user_checkFlag(&(pCtrl->Status),UART_CHECKCRC ) )
	{
        return CMD_ERROR;
	}
	
	/* Check CRC Flag. */
    uint8_t calCRC = mlsUtilsCRC8Calculate(pCtrl->storedData,pCtrl->lenData);
	
    if (calCRC == pCtrl->rcvCRC)
    {
        uint8_t * p = malloc(sizeof(uint8_t)* MAX_DATA_LENGTH * 2);
        if (p != NULL)
        {
            *ppBuf = p;
            memcpy(*ppBuf,pCtrl->storedData,pCtrl->lenData);
        }
        usart_Ctrl_Reset(pCtrl);
        return (protocolCmd_t) (pCtrl->storedData[0]);
    }
	
	/* CRC is wrong. */
    user_setFlag(&(pCtrl->Status),UART_BADCRC);
    return CMD_ERROR;
}

/*
 * @brief: pack and send packet
 * @param[in1] pData: pointer to data to send
 * @param[in2] len: only length of the data
 * @param[in3] pCtrl: pointer to usart controller used
 * @noreturn
 */
void sendDataPackage_USART(uint8_t * pData, uint8_t len, usartCtrl_t * pCtrl)
{
    if (len > 0)
    {
        uint8_t * pBuf = malloc(len + (8-(len%8)));
		
        if (pBuf)
        {
            pBuf[0] = HEADER_STM32,
            pBuf[1] = len & 0x00FF;
            memcpy(&pBuf[2],pData,len);
            pBuf[2 + len] = mlsUtilsCRC8Calculate(pData,len);
			
            sendData_USART(pBuf,len + OVERLOAD_SIZE,pCtrl->hUART);
            free(pBuf);
        }
    }
}

/*
 * @brief: wrapper for USART_SEND
 */
void sendData_USART(uint8_t * pData, uint8_t len, USART_TypeDef * hUART)
{
    while (len--)
    {
		USART_SendData(hUART,*pData++);
		/* This bit is set by hardware when the content of the TDR register has been transferred into the shift register */
		/* Wait until data has been sent */
		uint32_t CheckTimeOut = msTicks;
		
		while (USART_GetFlagStatus(hUART,USART_FLAG_TXE) == RESET)
		{
			if (msTicks - CheckTimeOut > UART_TIMEOUT_MS) 
			{
				break;
			}
		}
		USART_ClearFlag(hUART,USART_FLAG_TXE);
    }
}

uint8_t user_checkFlag(uint32_t *status,uint32_t flag)
{
   return (* status) & flag;
}

void user_setFlag(uint32_t *status, uint32_t flag)
{
   * status |= flag;
}

void user_clearFlag(uint32_t *status, uint32_t flag)
{
   * status &= (flag ^ 0xFFFFFFFF);
}
