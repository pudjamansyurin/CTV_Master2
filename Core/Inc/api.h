
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __API_H
#define __API_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported macros -----------------------------------------------------------*/
#define TX_LEN		16
#define RX_LEN		18
#define FREQ_CNT	4

/*******************************************************************************
 * @brief   SPI Write Packet Bytes
 *
 ******************************************************************************/
//Byte 0
#define SPI_MODE_BYTE							0x00

//Byte 1 - WRITE COMMAND
#define SPI_WRITE_CMD_BYTE						0x01

//Byte 2 - WRITE PARAMETER
#define SPI_WRITE_PRM_BYTE						0x02

/* Exported enum -------------------------------------------------------------*/
typedef enum
{
	AFE_REQ_WRITE = 0x05,
	AFE_REQ_READ  = 0x0A,
} eAFE_REQ;

typedef enum
{
	AFE_CMD_ON = 0x10,
	AFE_CMD_OFF,
	AFE_CMD_SCAN_NOISE,
	AFE_CMD_SCAN_SELF_TX,
	AFE_CMD_SCAN_SELF_RX,
	AFE_CMD_SCAN_MUTUAL,
} eAFE_CMD;

typedef enum
{
	AFE_FREQ_500_KHz,
	AFE_FREQ_400_KHz,
	AFE_FREQ_250_KHz,
	AFE_FREQ_100_KHz
} eAFE_FREQ;


/* Exported struct -----------------------------------------------------------*/
/*
 * AFE command structure
 */
typedef struct
{
	union {
		int16_t s16_data[5];
		struct {
			uint8_t u8_mode;		/* eAFE_REQ - request type */
			uint8_t u8_cmd;			/* eAFE_CMD - command type */
			uint8_t u8_freq;		/* eAFE_FREQ - scan frequency */
			uint8_t u8_txCnt;		/* number of tx for scan */
			uint8_t u8_accCnt;		/* number of accumulation pulse */

			uint8_t u8_isVref:1;	/* floating voltage reference */
			uint8_t u8_isDiff:1;	/* differential/single-ended mode */
		};
	};
} sAfeCmd_t;

/*
 * AFE reply header structure
 */
typedef struct
{
	union {
		int16_t s16_data[2];
		struct {
			uint8_t  u8_cmd;		/* eAFE_CMD - command type */
			uint16_t u16_len;		/* payload's size in half-word */
		};
	};
} sAfeHeader_t;

/*
 * AFE reply structure
 */
typedef struct
{
	sAfeHeader_t	header;
	int16_t 		s16_buf[TX_LEN*RX_LEN];
} sAfeReply_t;

#ifdef __cplusplus
}
#endif

#endif /* __API_H */
