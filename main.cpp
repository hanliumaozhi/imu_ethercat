/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#include "EasyCAT.h"

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"


#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

static uint8_t usart_rx_dma_buffer[46];
volatile int rec_counter;
static uint8_t raw_imu_data[256];
static uint8_t imu_data[46];
static uint8_t goal_imu_data[46];
int raw_counter = 0;
int buffer_index=0;
int index_remain = 0;

EasyCAT easycat(DC_SYNC);

typedef union
{
    uint8_t data[4];
    volatile float fdata;
} byte_4; 

volatile byte_4 tem_byte;


Serial pc(SERIAL_TX, SERIAL_RX, 115200);

//DigitalOut led(LED1);
DigitalOut sto(PA_9);

static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);

int main()
{
    sto = 0;

    MX_DMA_Init();
    MX_USART3_UART_Init();
    
    if(easycat.Init() == true){
        pc.printf("initialized\n");                            
    }else{
        pc.printf("initialization failed\n");                                                           
        while(1){                                                   
            /*led = 1;                                           
            HAL_Delay(125);                                    
            led = 0;                                         
            HAL_Delay(125);*/                                   
        }                                                   
    }


    while (true){
        
        //HAL_Delay(1000);

        //t.start();
        easycat.MainTask();
        core_util_critical_section_enter();
        tem_byte.data[3] = imu_data[2];
        tem_byte.data[2] = imu_data[3];
        tem_byte.data[1] = imu_data[4];
        tem_byte.data[0] = imu_data[5];
        easycat.BufferIn.Cust.acc_x = tem_byte.fdata;

        tem_byte.data[3] = imu_data[6];
        tem_byte.data[2] = imu_data[7];
        tem_byte.data[1] = imu_data[8];
        tem_byte.data[0] = imu_data[9];
        easycat.BufferIn.Cust.acc_y = tem_byte.fdata;

        tem_byte.data[3] = imu_data[10];
        tem_byte.data[2] = imu_data[11];
        tem_byte.data[1] = imu_data[12];
        tem_byte.data[0] = imu_data[13];
        easycat.BufferIn.Cust.acc_z = tem_byte.fdata;

        tem_byte.data[3] = imu_data[16];
        tem_byte.data[2] = imu_data[17];
        tem_byte.data[1] = imu_data[18];
        tem_byte.data[0] = imu_data[19];
        easycat.BufferIn.Cust.gyr_x = tem_byte.fdata;

        tem_byte.data[3] = imu_data[20];
        tem_byte.data[2] = imu_data[21];
        tem_byte.data[1] = imu_data[22];
        tem_byte.data[0] = imu_data[23];
        easycat.BufferIn.Cust.gyr_y = tem_byte.fdata;

        tem_byte.data[3] = imu_data[24];
        tem_byte.data[2] = imu_data[25];
        tem_byte.data[1] = imu_data[26];
        tem_byte.data[0] = imu_data[27];
        easycat.BufferIn.Cust.gyr_z = tem_byte.fdata;

        tem_byte.data[3] = imu_data[30];
        tem_byte.data[2] = imu_data[31];
        tem_byte.data[1] = imu_data[32];
        tem_byte.data[0] = imu_data[33];
        easycat.BufferIn.Cust.q0 = tem_byte.fdata;

        tem_byte.data[3] = imu_data[34];
        tem_byte.data[2] = imu_data[35];
        tem_byte.data[1] = imu_data[36];
        tem_byte.data[0] = imu_data[37];
        easycat.BufferIn.Cust.q1 = tem_byte.fdata;

        tem_byte.data[3] = imu_data[38];
        tem_byte.data[2] = imu_data[39];
        tem_byte.data[1] = imu_data[40];
        tem_byte.data[0] = imu_data[41];
        easycat.BufferIn.Cust.q2 = tem_byte.fdata;

        tem_byte.data[3] = imu_data[42];
        tem_byte.data[2] = imu_data[43];
        tem_byte.data[1] = imu_data[44];
        tem_byte.data[0] = imu_data[45];
        easycat.BufferIn.Cust.q3 = tem_byte.fdata;

        if(easycat.BufferOut.Cust.led_type == 0){
            sto = 0;
        }else if(easycat.BufferOut.Cust.led_type == 1){
            sto = 1;
        }

        core_util_critical_section_exit();
            
    }
    
}

static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration  
  PC5   ------> USART3_RX
  PC10   ------> USART3_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART3 DMA Init */
  
  /* USART3_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_1, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);
	
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&USART3->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)usart_rx_dma_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ARRAY_LEN(usart_rx_dma_buffer));
	
	
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

  /* DMA interrupt init */
  NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 921600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableIT_IDLE(USART3);
	
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_Init 2 */
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
	LL_USART_Enable(USART3);

  /* USER CODE END USART3_Init 2 */

}

static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */

}

void usart_process_data(const void* data, size_t len) {
    const uint8_t* d = static_cast<const uint8_t *>(data);
		for(int i = 0; i != len; ++i){
			raw_imu_data[raw_counter++] = d[i];
		}
		if(raw_counter >= 46){
			for(buffer_index = 0; buffer_index < (raw_counter-1); ++buffer_index){
				if(raw_imu_data[buffer_index] == 0x0E && raw_imu_data[(buffer_index+1)] == 0x04){
						break;
				}
			}
			if(buffer_index != (raw_counter-1)){
				if((raw_counter - buffer_index) >= 46){
					if(raw_imu_data[(buffer_index+15)] == 0x05){
						memcpy(imu_data, &raw_imu_data[buffer_index], 46);
						index_remain = (raw_counter - buffer_index - 46) ;
						for(int i = 0; i != index_remain; ++i){
							raw_imu_data[i] = raw_imu_data[(buffer_index+46+i)];
						}
						raw_counter = index_remain;
					}
				}
			}
			
		}
		
		if(raw_counter >= 200){
			raw_counter = 0;
		}
}


void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }
    old_pos = pos;                              /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) {
        old_pos = 0;
    }
}

extern "C" void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
	 if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_HT1(DMA1)) {
        LL_DMA_ClearFlag_HT1(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
extern "C" void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
		if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
        LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
    }

  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}