#include "stm32f4xx.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_rcc.h"

//#include "driver/uart.h"

#include "spi.h"


#define elm_strcmp(dat, x) memcmp(dat, x, strlen(x))
#define dbg_msg(x) uart0_tx_buffer(x, strlen(x))
#define byteToInt(x,i) (x[i]|(uint32_t)x[i+1]<<8|(uint32_t)x[i+2]<<16|(uint32_t)x[i+3]<<24)
#define asciiTobyte(c) (c>0x39?(c>0x46?c-0x61:c-0x41):c-0x30)

typedef unsigned char bool;
#define true  1
#define false 0

#define ELM_RCV_MAX_SIZE    128
#define ELM_TX_MAX_SIZE     128
uint8_t elm_resp[ELM_TX_MAX_SIZE];
uint8_t elm_recvData[ELM_RCV_MAX_SIZE] = {0};
static char dev_id[] = "4100BE3FA813";
static char last_cmd[ELM_RCV_MAX_SIZE] = {0};

enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

//CAN driver related definitions:
static CAN_HandleTypeDef CanHandle;
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

#define CANx CAN1
#define SPIx SPI4

//END of CAN defs

// Various constants.
const char ELM_OK[] = "OK\r\r>";
const char ELM_NA[] = "NA\r\r>";

struct elm_flags {
  char elm_echo;
  char elm_memory;
  char elm_linefeed;
  char elm_spaces;
  char elm_headers;
  char elm_adaptive;
  char elm_protocol;
  char elm_auto_proto;
}elm_flag = {'0','1','1','1','1','1','0','1'};


struct obd_flags {
  char obd_long_messages;
}obd_flag = {'0'};

struct can_flags {
  char can_monitor;
  char can_extended_addr;
}can_flag = {'0', '0'};

/***********FUNCTIONS***********/

size_t strlen(char const *s)
{
	size_t i;

	i= 0;
	while(s[i]) {
		i+= 1;
	}

	return i;
}

//From: https://github.com/bjornfor/stm32-test/blob/master/baselibc/src/strcpy.c
char *strcpy(char *dst, const char *src)
{
	char *q = dst;
	const char *p = src;
	char ch;

	do {
		*q++ = ch = *p++;
	} while (ch);

	return dst;
}

//From: https://github.com/bjornfor/stm32-test/blob/master/baselibc/src/strchr.c
char *strchr(const char *s, int c)
{
	while (*s != (char)c) {
		if (!*s)
			return NULL;
		s++;
	}

	return (char *)s;
}

//From https://github.com/bjornfor/stm32-test/blob/master/baselibc/src/strcat.c
char *strcat(char *dst, const char *src)
{
	strcpy(strchr(dst, '\0'), src);
	return dst;
}


bool is_digit(char c) {
    if (c >= '0' && c <= '9')
        return true;
}

bool is_hex(char c) {
    if ((c >= 'A' && c <= 'F') ||
        (c >= 'a' && c <= 'f') )
        return true;
    else return false;
}


char to_upper(char c) {
    if (is_digit(c))
        return c; 
    if (c < 'A')
        c = c - 'a' + 'A';
    return c;
}

//Converts one character to int from hex upper case or lower case
int chex2int(char c) {
    int val = -1;

    if (is_digit(c)) 
        val = c - '0';
    else if (is_hex(c))
        val = to_upper(c) - 'A' + 10;

    return val;
}

//TODO: handle both hhh and hh hh hh hh formats
int shex2int(char *s) {
    int res = 0;
    int i;
    for (i=0; i<strlen(s); i++) {
        int val = 0;
        val = chex2int(s[i]);
        if (val >= 0)
            res = (res << 4) + val; 
    }
}



//Some code is insipired by https://github.com/robots/STM32/blob/master/can_encoders/can.c


//Sends the data received on the CAN bus onto serial interface emulating elm327
void elm_tx_cb(CAN_HandleTypeDef* hcan) {
    if (can_flag.can_monitor == '1') {
        //Copy the data to response buffer
        memcpy(elm_resp, hcan->pRxMsg->Data, sizeof(hcan->pRxMsg->Data));

        //Send the data
        spi_tx_dma(elm_resp, strlen(elm_resp));
/* TODO: Echo
    char rsp_buf[ELM_TX_MAX_SIZE];
    int offset = 0;
    //Copy the input to output to implement echo
    if (elm_flag.elm_echo == '1'){
        strcpy(rsp_buf, elm_recvData);
        offset = strlen(elm_recvData);
    }
    strcpy(rsp_buf + offset, elm_resp);
*/

    }
}


//calclates the values of filter and mask based on the CAN msg format
//TODO: add support for extented CNA frames
void rid2filter(int rid, int *filter,  int *mask) {
    *filter = (0x0000FF00) && rid << 8;
    *mask = 0x0000FF00;
}

//calclates the values of filter and mask based on the CAN msg format
//TODO: add support for extented CNA frames
void tid2filter(int tid, int *filter,  int *mask) {
    *filter = (0x000000FF) && tid;
    *mask = 0x000000FF;
}


// send on CAN
void can_tx(uint8_t *data, int len) {

    int dpkt = 0;
    int i;
    for (dpkt = 0; dpkt < len; dpkt += 0x10) {
      uint32_t *tf = (uint32_t*)(&data[dpkt]);

      CanHandle.pTxMsg->StdId = 0x11;
      CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
      CanHandle.pTxMsg->IDE = CAN_ID_STD;
      CanHandle.pTxMsg->DLC = 8;
      CanHandle.pTxMsg->Data[0] = tf[3];
      CanHandle.pTxMsg->Data[1] = tf[2];
      
      if(HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK)
      {
        /* Transmition Error */
        Error_Handler();
      }
      
      if(HAL_CAN_GetState(&CanHandle) != HAL_CAN_STATE_READY)
      {
        return HAL_ERROR;
      }
    }
}

//Handle incoming request from ELM327 user (app such as Torque)
//Data is already in the buffer by DMA from SPI
void elm_process() {

  if (wTransferState != TRANSFER_COMPLETE)
    return;

  char *data = elm_recvData;
  //Any activity on the input bus (UART) stops the CAN bus monitoring
  can_flag.can_monitor = 0;

  if (elm_strcmp(data, "AT") == 0) {
    int hhh = shex2int(&(data[3]));
    int filter;
    int mask;

    data += 2;
    memset(elm_resp, 0, 128);
    if (elm_strcmp(data, "Z\r") == 0) {
      HAL_CAN_Init(&CanHandle);
      strcpy(elm_resp,"ELM327 v2.1\r\r>");
    } else if (data[0] == 'E') {
      elm_flag.elm_echo = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'M') {
      elm_flag.elm_memory = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'L') {
      elm_flag.elm_linefeed = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'S') {
      elm_flag.elm_spaces = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'H') {
      elm_flag.elm_headers = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "AT") == 0) {
      elm_flag.elm_adaptive = data[2];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'I') {
      strcpy(elm_resp,"ELM327 v2.1\r\r>");
//General commands:
    } else if (elm_strcmp(data, "@1\r") == 0) { //display the device description
      strcpy(elm_resp,"OBDII to RS232 Interpreter\r\r>");
    } else if (elm_strcmp(data, "@2\r") == 0) { //display the device identifier
      strcpy(elm_resp,dev_id);
    } else if (elm_strcmp(data, "@3") == 0) {   //store the device identifier
      strcpy(dev_id,&data[2]);
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "\r") == 0) {   //repeat the last command
      strcpy(data,last_cmd);
      elm_process(); //recursion
    } else if (elm_strcmp(data, "BRD") == 0) {   //Try baud rate divisor hh
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "BRT") == 0) {   //Set Baud Rate handshake timeout hh
      strcpy(elm_resp,ELM_NA);
//OBD commands:
    } else if (elm_strcmp(data, "AL\r") == 0) {   //allow long messages
      obd_flag.obd_long_messages = '1';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "NL\r") == 0) {   //disable long messages
      obd_flag.obd_long_messages = '0';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "AR\r") == 0) {   //Automatically receive
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "AT0\r") == 0) {   //Adaptive timing off
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "AT1\r") == 0) {   //Adaptive timing auto1
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "AT2\r") == 0) {   //Adaptive timing auto2
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "BD\r") == 0) {   //Perform a buffer dump
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "BI\r") == 0) {   //Bypass the initialization sequence
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "MA\r") == 0) {   //Monitor all
      //Clear all filters:
      can_filters_clear(CANx);
      //Start monitoring
      can_flag.can_monitor = '1';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "MR") == 0) {   //Monitor messages with receiver id hh
      hhh = shex2int(&(data[2]));
      rid2filter(hhh, &filter, &mask);
      //Start monitoring
      can_flag.can_monitor = '1';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "MT") == 0) {   //Monitor messages with transmitter id hh
      hhh = shex2int(&(data[2]));
      tid2filter(hhh, &filter, &mask);
      //Start monitoring
      can_flag.can_monitor = '1';
      strcpy(elm_resp,ELM_OK);
//CAN commands:
    } else if (elm_strcmp(data, "CAF0\r") == 0) {   //CAN automatic formatting off
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CAF1\r") == 0) {   //CAN automatic formatting on
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CEA\r") == 0) {   //turn off CAN extended addressing
        can_flag.can_extended_addr = '0';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CEA") == 0) {   //use CAN extended address hh
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CF") == 0) {   //set the id filter to hh hh hh hh or hhh
      can_filter_config(CANx, shex2int(&(data[2])), -1); 
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CFC0") == 0) {   //CAN flow control off
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CFC1") == 0) {   //CAN flow control on
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CM") == 0) {   //Set CAN ID mask to hhh or hh hh hh hh 
        can_filter_config(CANx, -1, shex2int(&(data[2]))); 
        strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CRA\r") == 0) {   //Reset CAN receive filters
        can_filters_clear(CAN1);
        strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CRA") == 0) {   //Set CAN receive address to hhh or hh hh hh hh
        hhh = shex2int(&(data[3]));
        rid2filter(hhh, &filter, &mask);
        can_filter_config(CANx, filter, mask); 
        strcpy(elm_resp,ELM_OK);

    } else if (elm_strcmp(data, "DPN") == 0) {
      elm_resp[0] = elm_flag.elm_protocol;
      strcat(elm_resp,"\r\r>");
    } else if (elm_strcmp(data, "SPA") == 0) { //Set protocol to Auto, start with h
      elm_flag.elm_auto_proto = '1'; 
      elm_flag.elm_protocol = data[3];
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "SP") == 0) { //Set protocol to h
      elm_flag.elm_auto_proto = '0'; 
      elm_flag.elm_protocol = data[2];
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "SP00") == 0) { //Set protocol to Auto
      elm_flag.elm_auto_proto = '1'; 
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "TPA") == 0) {  //Try protocol h with auto search
      elm_flag.elm_auto_proto = '1'; 
      strcpy(elm_resp,ELM_OK);
    }
    spi_tx_dma(elm_resp, strlen(elm_resp));
    strcpy(last_cmd, data);
  } else if(elm_strcmp(data, "01") == 0) {
      memset(elm_resp, 0, 128);
      switch(asciiTobyte(data[2]<<4)|asciiTobyte(data[3])) {
        case 0:
          strcpy(elm_resp,"4100BE3FA813\r");
        break;
     }
    spi_tx_dma(elm_resp, strlen(elm_resp));
  } else { //pass the command to the CAN bus
    can_tx(data, strlen(data));
  }
  
  wTransferState = TRANSFER_WAIT;
  //Start async RX request:
  if(HAL_SPI_Receive_IT(&SpiHandle, (uint8_t *)elm_recvData, sizeof(elm_recvData)) != HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }

  //TODO: For debugging
  //uart0_tx_buffer(elm_resp, strlen(elm_resp));
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while(1)
  {
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 200
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
#ifdef EXCLUDED
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
static void can_config(void)
{
  CAN_FilterConfTypeDef  sFilterConfig;
  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;
  
  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CANx;
  CanHandle.pTxMsg = &TxMessage;
  CanHandle.pRxMsg = &RxMessage;
    
  CanHandle.Init.TTCM = DISABLE;
  CanHandle.Init.ABOM = DISABLE;
  CanHandle.Init.AWUM = DISABLE;
  CanHandle.Init.NART = DISABLE;
  CanHandle.Init.RFLM = DISABLE;
  CanHandle.Init.TXFP = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SJW = CAN_SJW_1TQ;
  CanHandle.Init.BS1 = CAN_BS1_6TQ;
  CanHandle.Init.BS2 = CAN_BS2_8TQ;
  CanHandle.Init.Prescaler = 2;
  
  if(HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  
  if(HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
      
  /*##-3- Configure Transmission process #####################################*/
  CanHandle.pTxMsg->StdId = 0x321;
  CanHandle.pTxMsg->ExtId = 0x01;
  CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
  CanHandle.pTxMsg->IDE = CAN_ID_STD;
  CanHandle.pTxMsg->DLC = 2;
}

void elm_spi_init() {
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  SpiHandle.Init.Direction         = SPI_DIRECTION_1LINE;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.Mode              = SPI_MODE_MASTER;
  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


void elm327_init() {
  HAL_Init();
  
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  
  /*##-1- Configure the CAN peripheral #######################################*/
  can_config();
  
  /*##-2- Start the Reception process and enable reception interrupt #########*/
  if(HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  
  //Init SPI 
  elm_spi_init();
  //Start async RX request:
  if(HAL_SPI_Receive_IT(&SpiHandle, (uint8_t *)elm_recvData, sizeof(elm_recvData)) != HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }

}

void can_filters_clear(CAN_TypeDef *CAN) {
    can_filter_config(CAN, 0, 0);
}

//Sets the filter or the mask, assuming 32 bit filter 
//-1 means the value should not change
void can_filter_config(CAN_TypeDef *CAN, uint32_t filter, uint32_t mask) {

  CAN_FilterConfTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = (filter >> 16);
  sFilterConfig.FilterIdLow =  (filter & 0xFFFF);
  sFilterConfig.FilterMaskIdHigh = (mask >> 16);
  sFilterConfig.FilterMaskIdLow =  (mask & 0xFFFF);
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  
  if(HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
}

/**
  * @brief  Transmission complete callback in non blocking mode 
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
//Do not change the signature of this function
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
  /* Receive */
  if(HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  elm_tx_cb(CanHandle);
}


/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
//Do not change the signature of this funtion
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_COMPLETE;
}
