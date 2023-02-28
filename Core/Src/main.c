/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "api.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//====== PORT E ======//
#define portCTRL GPIOE

#define pinVREFH	0
#define pinCTVH		1
#define pinRSTACCH	2
#define pinACCPH	3
#define pinACCNH	4
#define pinCFB2H	5
#define pinCFB1H	6
#define pinCINJNH	7
#define pinCINJPH	8

#define pinVREFL	(pinVREFH+16)
#define pinCTVL		(pinCTVH+16)
#define pinRSTACCL		(pinRSTACCH+16)
#define pinACCPL	(pinACCPH+16)
#define pinACCNL	(pinACCNH+16)
#define pinCFB2L	(pinCFB2H+16)
#define pinCFB1L	(pinCFB1H+16)
#define pinCINJNL	(pinCINJNH+16)
#define pinCINJPL	(pinCINJPH+16)

//====== PORT G ======//
#define portADD GPIOG

#define pinLEDH		5
#define pinLEDL		(pinLEDH+16)

//====== PORT A ======//
#define portMSTR GPIOA

#define pinMSTRH	1
#define pinMSTRL	(pinMSTRH+16)

//====== PORT F ======//
#define portTX GPIOF


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TX_LEN		16
#define RX_LEN		18
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi6_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
//From GUI
uint8_t run_f = 0,
		Vref_f = 0,
		Freq_f = 0,
		Baseline_f = 0,
		initTx_f=0,
		nTx_f = TX_LEN,
		numAcc_f = 64,
		prev_Freq,
		dest_USART = 0,
		dest_SPI = 0,
		dest_I2C = 0;
uint8_t vfreq, vnTx = 10, vnAcc, vVref;

//TX Pin
uint8_t pinTXH[17] = {99,3,2,1,0,4,5,6,7,8,9,10,11,12,13,14,15}; //15->3
uint8_t pinTXL[17] = {99,19,18,17,16,20,21,22,23,24,25,26,27,28,29,30,31};

int var_debug=0;
char Rx_UART[100];
unsigned char i,j;
volatile uint8_t spi1_f=0, spi4_f=0, spi6_f=0, spi2_f=0, spi2t_f=0;

int pData[12],pData1[12],pData2[12],temp_pData[12];
int16_t frame[(sizeof(sAfeReply_t) + TX_LEN*RX_LEN) / sizeof(int16_t)];
int16_t SelfTx[TX_LEN], SelfRx[RX_LEN];

char str_Frame[1731]={'*'}; //1+6*16*18+2 -> *+data+&+\n
#define slave_addr (0x11<<1)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


static void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer = 0;

    pBuffer++;
  }
}

//delay2
void delay1(void)
{
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
}
//with Vref
void delay_AFE_Init(void)
{
	asm("NOP");
	asm("NOP");
	asm("NOP");
	portCTRL->BSRR |= (1<<pinVREFH);	//Vref
	delay1();
	delay1();
	portCTRL->BSRR |= (1<<pinVREFL);	//Vref
}
//without Vref (ON terus)
void delay_AFE_Init1(void)
{
	asm("NOP");
	asm("NOP");
	asm("NOP");
	portCTRL->BSRR |= (1<<pinVREFH);	//Vref
	delay1();
	delay1();
}

void Tx100k(int ch)
{
	uint8_t m=0,n=0,p=0,q=0;
		portCTRL->BSRR |= (1<<pinACCNL)|(1<<pinCINJNL);
	//	portCTRL->BSRR |= (1<<pinCINJNL);

		//WAITING DELAYYY
		do
		{
			asm("NOP");
		} while(q++<79);


		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");


		portCTRL->BSRR |= (1<<pinACCPH)|(1<<pinCINJPH);
	//	portCTRL->BSRR |= (1<<pinCINJPH);
		portTX->BSRR |= (1<<pinTXH[ch]);
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
	//	portCTRL->BSRR |= (1<<pinACCPL);//|(1<<pinCINJPL);
		do
		{
			asm("NOP");
		} while(m++<13);
		portCTRL->BSRR |= (1<<pinACCPL)|(1<<pinCINJPL);
	//	portCTRL->BSRR |= (1<<pinCINJPL);

		//WAITING DELAYYY
		do
		{
			asm("NOP");
		} while(p++<79);



		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");



		portCTRL->BSRR |= (1<<pinACCNH)|(1<<pinCINJNH);
	//	portCTRL->BSRR |= (1<<pinCINJNH);
		portTX->BSRR |= (1<<pinTXL[ch]);
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
	//	portCTRL->BSRR |= (1<<pinACCNL);//|(1<<pinCINJNL);
		do
		{
			asm("NOP");
		} while(n++<3);


}

void Tx500k(int ch)
{
	uint8_t m=0,n=0;
	portCTRL->BSRR |= (1<<pinACCNL)|(1<<pinCINJNL);
//	portCTRL->BSRR |= (1<<pinCINJNL);


	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");


	portCTRL->BSRR |= (1<<pinACCPH)|(1<<pinCINJPH);
//	portCTRL->BSRR |= (1<<pinCINJPH);
	portTX->BSRR |= (1<<pinTXH[ch]);
	asm("NOP");
	asm("NOP");

//	portCTRL->BSRR |= (1<<pinACCPL);//|(1<<pinCINJPL);
	do
	{
		asm("NOP");
	} while(m++<13);	//13
	portCTRL->BSRR |= (1<<pinACCPL)|(1<<pinCINJPL);
//	portCTRL->BSRR |= (1<<pinCINJPL);



	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");



	portCTRL->BSRR |= (1<<pinACCNH)|(1<<pinCINJNH);
//	portCTRL->BSRR |= (1<<pinCINJNH);
	portTX->BSRR |= (1<<pinTXL[ch]);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

//	portCTRL->BSRR |= (1<<pinACCNL);//|(1<<pinCINJNL);
	do
	{
		asm("NOP");
	} while(n++<3);	//3

}

void Tx400k(int ch)
{
	uint8_t m=0,n=0,p=0,q=0;
	portCTRL->BSRR |= (1<<pinACCNL)|(1<<pinCINJNL);
//	portCTRL->BSRR |= (1<<pinCINJNL);

	//WAITING DELAYYY
	do
	{
		asm("NOP");
	} while(q++<4);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");


	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");


	portCTRL->BSRR |= (1<<pinACCPH)|(1<<pinCINJPH);
//	portCTRL->BSRR |= (1<<pinCINJPH);
	portTX->BSRR |= (1<<pinTXH[ch]);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
//	portCTRL->BSRR |= (1<<pinACCPL);//|(1<<pinCINJPL);
	do
	{
		asm("NOP");
	} while(m++<13);
	portCTRL->BSRR |= (1<<pinACCPL)|(1<<pinCINJPL);
//	portCTRL->BSRR |= (1<<pinCINJPL);

	//WAITING DELAYYY
	do
	{
		asm("NOP");
	} while(p++<4);



	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");



	portCTRL->BSRR |= (1<<pinACCNH)|(1<<pinCINJNH);
//	portCTRL->BSRR |= (1<<pinCINJNH);
	portTX->BSRR |= (1<<pinTXL[ch]);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
//	portCTRL->BSRR |= (1<<pinACCNL);//|(1<<pinCINJNL);
	do
	{
		asm("NOP");
	} while(n++<3);


}

void Tx250k(int ch)
{
	uint8_t m=0,n=0,p=0,q=0;
	portCTRL->BSRR |= (1<<pinACCNL)|(1<<pinCINJNL);
//	portCTRL->BSRR |= (1<<pinCINJNL);

	//WAITING DELAYYY
	do
	{
		asm("NOP");
	} while(q++<19);


	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");


	portCTRL->BSRR |= (1<<pinACCPH)|(1<<pinCINJPH);
//	portCTRL->BSRR |= (1<<pinCINJPH);
	portTX->BSRR |= (1<<pinTXH[ch]);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
//	portCTRL->BSRR |= (1<<pinACCPL);//|(1<<pinCINJPL);
	do
	{
		asm("NOP");
	} while(m++<13);
	portCTRL->BSRR |= (1<<pinACCPL)|(1<<pinCINJPL);
//	portCTRL->BSRR |= (1<<pinCINJPL);

	//WAITING DELAYYY
	do
	{
		asm("NOP");
	} while(p++<19);



	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");



	portCTRL->BSRR |= (1<<pinACCNH)|(1<<pinCINJNH);
//	portCTRL->BSRR |= (1<<pinCINJNH);
	portTX->BSRR |= (1<<pinTXL[ch]);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
//	portCTRL->BSRR |= (1<<pinACCNL);//|(1<<pinCINJNL);
	do
	{
		asm("NOP");
	} while(n++<3);


}

//NEW RUN SCAN PARENT
void Run_TX(int freq, int ch)
{
	switch (freq)
	{
	case 0:	Tx500k(ch);	break;
	case 1: Tx400k(ch); break;
	case 2: Tx250k(ch); break;
	default: Tx100k(ch);
	}
}

//NEW Run scan 500khz 3 slave
void Run_Scans(uint8_t freq, uint8_t initTx, uint8_t nTx, uint8_t nAcc, uint8_t vreff, int16_t* s16p_frame)
{
	int16_t* s16p_frameN;
	uint8_t n=initTx;
	uint8_t o=initTx+nTx-1;

	do
	{
		s16p_frameN = s16p_frame + (n * RX_LEN);

		uint8_t m=0;
		//========================= TX(n) =============================

		GPIOA->BSRR |= (1<<20);
		GPIOE->BSRR |= (1<<27);
		GPIOG->BSRR |= (1<<24);

//		HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, GPIO_PIN_RESET);
		//===== SEND TX SIGNAL =====
		if(vreff == 1) delay_AFE_Init();
		else delay_AFE_Init1();

//		portRSTACC->BSRR |= (1<<pinRSTACCL);	//RST ACC low
//		delay1();
//		delay1();
//		delay1();
		delay1();
		portCTRL->BSRR |= (1<<pinCTVH);	//CTI ON PB7
		portCTRL->BSRR |= (1<<pinACCPL) | (1<<pinCINJPL);
		portCTRL->BSRR |= (1<<pinACCNL) | (1<<pinCINJNL);
		delay1();
		delay1();
		delay1();	//expect 10us

		portCTRL->BSRR |= (1<<pinRSTACCL) | (1<<pinCFB1L);
		//portTX->BSRR |= (1<<pinACCL);
		do
		{
			Run_TX(freq,n+1);
			//portTX->BSRR |= (1<<pinACCL);
		} while(m++ < nAcc-1);
		portCTRL->BSRR |= (1<<pinACCNL)|(1<<pinCINJNL);
//		portCTRL->BSRR |= (1<<pinCINJNL);
		portCTRL->BSRR |= (1<<pinCTVL);	//CTI OFF



		//=== SEND READ ADC CMD ===
		portMSTR->BSRR |= (1<<pinMSTRH);	//MSTR ON PC6-------
		GPIOB->BSRR |= (1<<11);
		//HAL_Delay(1);
//		portMSTR->BSRR |= (1<<pinMSTRL);	//MSTR OFF PC6

		//portCTRL->BSRR |= (1<<pinVREFL);	//Vref
		asm("NOP");
		asm("NOP");
		asm("NOP");


		//=== WAIT ADC COLLECTED ===
//		while(HAL_GPIO_ReadPin(SLV1_GPIO_Port, SLV1_Pin) == 0 || HAL_GPIO_ReadPin(SLV2_GPIO_Port, SLV2_Pin) == 0){}

//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");
//		asm("NOP");

//		HAL_GPIO_WritePin(RSTACC_GPIO_Port, RSTACC_Pin, GPIO_PIN_SET);
//		HAL_Delay(3);
//		HAL_GPIO_WritePin(RSTACC_GPIO_Port, RSTACC_Pin, GPIO_PIN_RESET);

		//=== RECEIVE DATA FROM SPI ===
		//===========================================================//
		//portCS->BSRR |= (1<<pinCS1H);	//SS1 ON PB14
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");

		//while(HAL_SPI_Receive(&hspi2, (uint8_t *)pData1, sizeof(pData1), 1000)!=HAL_OK){HAL_Delay(100);}
		while(spi1_f == 0){}
		spi1_f = 0;
		//-----------------------------------//
		portCTRL->BSRR |= (1<<pinVREFH);	//Vref

		s16p_frameN[0] = pData[0] - pData[1];
		s16p_frameN[1] = pData[2] - pData[3];
		s16p_frameN[2] = pData[4] - pData[5];
		s16p_frameN[3] = pData[6] - pData[7];
		s16p_frameN[4] = pData[8] - pData[9];
		s16p_frameN[5] = pData[10] - pData[11];
		GPIOB->BSRR |= (1<<27);
		GPIOA->BSRR |= (1<<4);
		//HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);

		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");

		while(spi4_f == 0){}
		spi4_f = 0;
		s16p_frameN[6] = pData1[0] - pData1[1];
		s16p_frameN[7] = pData1[2] - pData1[3];
		s16p_frameN[8] = pData1[4] - pData1[5];
		s16p_frameN[9] = pData1[6] - pData1[7];
		s16p_frameN[10] = pData1[8] - pData1[9];
		s16p_frameN[11] = pData1[10] - pData1[11];

		GPIOE->BSRR |= (1<<11);
		//HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);


		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");

		while(spi6_f == 0){}
		spi6_f = 0;
		s16p_frameN[12] = pData2[0] - pData2[1];
		s16p_frameN[13] = pData2[2] - pData2[3];
		s16p_frameN[14] = pData2[4] - pData2[5];
		s16p_frameN[15] = pData2[6] - pData2[7];
		s16p_frameN[16] = pData2[8] - pData2[9];
		s16p_frameN[17] = pData2[10] - pData2[11];

		GPIOG->BSRR |= (1<<8);
		//HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, GPIO_PIN_SET);

//		if(spi1_f == 0 && spi4_f == 0 && spi6_f == 0){}
//		spi1_f = 0;spi4_f = 0;spi6_f = 0;
//		HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, GPIO_PIN_SET);

		//=== RESET ACCUMULATOR ===
		portCTRL->BSRR |= (1<<pinRSTACCH)| (1<<pinCFB1H);// | (1<<pinCFB1H) | (1<<pinCFB2H);	//ACCRST ON PB6
		portMSTR->BSRR |= (1<<pinMSTRL);	//MSTR OFF PC6
		//portCS->BSRR |= (1<<pinCS2L);	// SS2 OFF PB12
		//===========================================================//
	} while(n++ < o);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int16_t* s16p_buf;
	uint16_t u16_bufLen;
	sAfeCmd_t AfeCmd;
	sAfeReply_t* p_AfeReply;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Flush_Buffer((uint8_t *)pData, sizeof(pData));
  Flush_Buffer((uint8_t *)pData1, sizeof(pData1));
  Flush_Buffer((uint8_t *)pData2, sizeof(pData2));

//  HAL_SPI_Receive_IT(&hspi1, (uint8_t *)pData, 24);
  HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)pData, sizeof(pData));
  HAL_SPI_Receive_DMA(&hspi4, (uint8_t *)pData1, sizeof(pData1));
  HAL_SPI_Receive_DMA(&hspi6, (uint8_t *)pData2, sizeof(pData2));
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)Rx_UART, 100);

  uint8_t b;
  for(b=0;b<TX_LEN;b++)	SelfTx[b] = b*10;
  for(b=0;b<RX_LEN;b++)	SelfRx[b] = b*10;

  // reply header
  p_AfeReply = (sAfeReply_t*) &frame[0];
  s16p_buf 	 = &frame[sizeof(sAfeReply_t)/sizeof(int16_t)];

  while (1)
  {
	  // wait command from host
	  spi2_f = 0;
	  HAL_SPI_Receive_IT(&hspi2,
		  	  	  	  	 (uint8_t*) &AfeCmd,
		  	  	  	  	 sizeof(sAfeCmd_t)/sizeof(int16_t));
	  while(0 == spi2_f) {};

	  // process command
	  switch (AfeCmd.u8_cmd)
	  {
		  case AFE_CMD_SCAN_NOISE:
			  //Freq_f 	= AfeCmd.u8_freq;
			  //nTx_f  	= AfeCmd.u8_txCnt;
			  numAcc_f 	= AfeCmd.u8_accCnt;
			  Vref_f 	= AfeCmd.u8_isVref;
			  initTx_f 	= 0;

			  initTx_f = 1; nTx_f = 4;
			  Run_Scans(0, initTx_f, nTx_f, numAcc_f, Vref_f, s16p_buf);

			  initTx_f = 5; nTx_f = 8;
			  Run_Scans(1, initTx_f, nTx_f, numAcc_f, Vref_f, s16p_buf);

			  initTx_f = 9; nTx_f = 12;
			  Run_Scans(2, initTx_f, nTx_f, numAcc_f, Vref_f, s16p_buf);

			  initTx_f = 13; nTx_f = 16;
			  Run_Scans(3, initTx_f, nTx_f, numAcc_f, Vref_f, s16p_buf);

			  s16p_buf   = (int16_t*) frame;
			  u16_bufLen = sizeof(frame);
			  break;

		  case AFE_CMD_SCAN_SELF_TX:
			  //Freq_f 	= AfeCmd.u8_freq;
			  nTx_f  	= AfeCmd.u8_txCnt;
			  //numAcc_f 	= AfeCmd.u8_accCnt;
			  //Vref_f 	= AfeCmd.u8_isVref;

			  // dummy process
			  HAL_Delay(1);

			  s16p_buf   = SelfTx;
			  u16_bufLen = nTx_f * sizeof(int16_t);
			  break;

		  case AFE_CMD_SCAN_SELF_RX:
			  // dummy process
			  HAL_Delay(1);

			  s16p_buf   = SelfRx;
			  u16_bufLen = sizeof(SelfRx);
			  break;

		  case AFE_CMD_SCAN_MUTUAL:
			  Freq_f 	= AfeCmd.u8_freq;
			  nTx_f  	= AfeCmd.u8_txCnt;
			  numAcc_f 	= AfeCmd.u8_accCnt;
			  Vref_f 	= AfeCmd.u8_isVref;
			  initTx_f 	= 0;

			  Run_Scans(Freq_f, initTx_f, nTx_f, numAcc_f, Vref_f, s16p_buf);
			  u16_bufLen = sizeof(frame);
			  break;

		  default:
			  u16_bufLen = 0;
			  break;
	  }

	  // inject header
	  p_AfeReply->u8_isOk = (NULL != s16p_buf);
	  p_AfeReply->u8_cmd  = AfeCmd.u8_cmd;
	  u16_bufLen += sizeof(sAfeReply_t);

	  // send scan result
	  spi2t_f = 0;
	  HAL_SPI_Transmit_IT(&hspi2,
		  	  	  	  	  (uint8_t*) frame,
						  u16_bufLen / sizeof(int16_t));
	  GPIOB->BSRR |= (1<<8);
	  GPIOB->BSRR |= (1<<24);
	  while(spi2t_f == 0){}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_1LINE;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_SLAVE;
  hspi6.Init.Direction = SPI_DIRECTION_1LINE;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ACCP_Pin|ACCN_Pin|CFB1_Pin|CFB2_Pin
                          |CINJN_Pin|GPIO_PIN_7|CINJP_Pin|CS2_Pin
                          |VREF_Pin|CTV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TX0_Pin|TX1_Pin|TX2_Pin|TX3_Pin
                          |TX4_Pin|TX5_Pin|TX6_Pin|TX7_Pin
                          |TX8_Pin|TX9_Pin|TX10_Pin|TX11_Pin
                          |TX12_Pin|TX13_Pin|TX14_Pin|TX15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MSTR_Pin|CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RSV1_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_Pin|CS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ACCP_Pin ACCN_Pin CFB1_Pin CFB2_Pin
                           CINJN_Pin PE7 CINJP_Pin CS2_Pin
                           VREF_Pin CTV_Pin */
  GPIO_InitStruct.Pin = ACCP_Pin|ACCN_Pin|CFB1_Pin|CFB2_Pin
                          |CINJN_Pin|GPIO_PIN_7|CINJP_Pin|CS2_Pin
                          |VREF_Pin|CTV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC6 PC7 PC8
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TX0_Pin TX1_Pin TX2_Pin TX3_Pin
                           TX4_Pin TX5_Pin TX6_Pin TX7_Pin
                           TX8_Pin TX9_Pin TX10_Pin TX11_Pin
                           TX12_Pin TX13_Pin TX14_Pin TX15_Pin */
  GPIO_InitStruct.Pin = TX0_Pin|TX1_Pin|TX2_Pin|TX3_Pin
                          |TX4_Pin|TX5_Pin|TX6_Pin|TX7_Pin
                          |TX8_Pin|TX9_Pin|TX10_Pin|TX11_Pin
                          |TX12_Pin|TX13_Pin|TX14_Pin|TX15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA3 PA7
                           PA8 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MSTR_Pin CS1_Pin */
  GPIO_InitStruct.Pin = MSTR_Pin|CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB3 PB4 PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG2 PG3 PG4
                           PG6 PG7 PG9 PG10
                           PG11 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SWRUN_Pin */
  GPIO_InitStruct.Pin = SWRUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWRUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE10 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RSV1_Pin */
  GPIO_InitStruct.Pin = RSV1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RSV1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD14 PD15 PD0 PD1
                           PD2 PD3 PD4 PD5
                           PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin CS3_Pin */
  GPIO_InitStruct.Pin = LED_Pin|CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	run_f = Rx_UART[1] - 48;
	Vref_f = Rx_UART[2] - 48;
	Freq_f = Rx_UART[3] - 48;
	Baseline_f = Rx_UART[4] - 48;
	nTx_f = Rx_UART[5] - 48;
	numAcc_f = Rx_UART[6] - 48;
	dest_USART = Rx_UART[7] - 48;
	dest_SPI = Rx_UART[8] - 48;
	dest_I2C = Rx_UART[9] - 48;

	//New Init
//	spi1_f = 0;
//	spi3_f = 0;
	Flush_Buffer((uint8_t *)pData1, 24);
	Flush_Buffer((uint8_t *)pData, 24);
//	HAL_SPI_DMAStop(&hspi1);
//	HAL_SPI_DMAStop(&hspi3);
//	HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)pData1, 24);
//	HAL_SPI_Receive_DMA(&hspi3, (uint8_t *)pData, 24);


//	if(Baseline_f == 1 && Freq_f != prev_Freq)	  Collect_Baseline();
//	prev_Freq = Freq_f;

}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi->Instance == SPI2)	spi2t_f = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
    if(hspi->Instance == SPI1)
    {
    	temp_pData[0] = pData[0];
    	temp_pData[1] = pData[1];
    	temp_pData[2] = pData[2];
    	temp_pData[3] = pData[3];
    	temp_pData[4] = pData[4];
    	temp_pData[5] = pData[5];
    	temp_pData[6] = pData[6];
    	temp_pData[7] = pData[7];
    	temp_pData[8] = pData[8];
    	temp_pData[9] = pData[9];
    	temp_pData[10] = pData[10];
    	temp_pData[11] = pData[11];
    	spi1_f = 1;
    	//Flush_Buffer((uint8_t*)pData, sizeof(pData));

    }
    if(hspi->Instance == SPI4)
    {

    	temp_pData[0] = pData1[0];
    	temp_pData[1] = pData1[1];
    	temp_pData[2] = pData1[2];
    	temp_pData[3] = pData1[3];
    	temp_pData[4] = pData1[4];
    	temp_pData[5] = pData1[5];
    	temp_pData[6] = pData1[6];
    	temp_pData[7] = pData1[7];
    	temp_pData[8] = pData1[8];
    	temp_pData[9] = pData1[9];
    	temp_pData[10] = pData1[10];
    	temp_pData[11] = pData1[11];
    	spi4_f = 1;
    }
    if(hspi->Instance == SPI6)
    {

    	temp_pData[0] = pData2[0];
    	temp_pData[1] = pData2[1];
    	temp_pData[2] = pData2[2];
    	temp_pData[3] = pData2[3];
    	temp_pData[4] = pData2[4];
    	temp_pData[5] = pData2[5];
    	temp_pData[6] = pData2[6];
    	temp_pData[7] = pData2[7];
    	temp_pData[8] = pData2[8];
    	temp_pData[9] = pData2[9];
    	temp_pData[10] = pData2[10];
    	temp_pData[11] = pData2[11];
    	spi6_f = 1;
    }
    if(hspi->Instance == SPI2)
    {
    	spi2_f = 1;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

