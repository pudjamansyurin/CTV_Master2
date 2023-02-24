
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __API_H
#define __API_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/

/*******************************************************************************
 * @brief   SPI Write Packet Bytes
 *
 ******************************************************************************/
//Byte 0
#define SPI_MODE_BYTE							0x00

#define SPI_WRITE_REQ							0x05
#define SPI_READ_REQ							0x0A


//Byte 1 - WRITE COMMAND
#define SPI_WRITE_CMD_BYTE						0x01

#define SPI_SCAN_ON								0x10		//Scan ON.
#define SPI_SCAN_OFF							0x11		//Scan OFF.
#define SPI_SCAN_NOISE							0x12		//Start Scan Noise.
#define SPI_SCAN_SELF_TX						0x13		//Start Scan Self TX.
#define SPI_SCAN_SELF_RX						0x14		//Start Scan Self RX.
#define SPI_SCAN_MUTUAL							0x15		//Start Scan Mutual.


//Byte 2 - WRITE PARAMETER
#define SPI_WRITE_PRM_BYTE						0x02

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
	union {
		uint8_t u8_data[10];
		struct {
			uint8_t u8_modeRW;
			uint8_t u8_scanType;
			uint8_t u8_vRef;
			uint8_t u8_freq;
			uint8_t u8_txCnt;
			uint8_t u8_accCnt;
		};
	};
} sAfeCmd_t;

#ifdef __cplusplus
}
#endif

#endif /* __API_H */
