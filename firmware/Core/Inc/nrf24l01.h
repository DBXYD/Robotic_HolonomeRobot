#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "main.h"

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

/* Payload size */
#define PAYLOAD_SIZE 32

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

HAL_StatusTypeDef NRF24L01_Init(void);
//HAL_StatusTypeDef NRF24L01_WriteRegister(uint8_t reg, uint8_t value);
HAL_StatusTypeDef NRF24L01_WriteCSN(GPIO_PinState PinState);
HAL_StatusTypeDef NRF24L01_WriteCE(GPIO_PinState PinState);
HAL_StatusTypeDef NRF24L01_GetStatus(uint8_t* spiRxBuffer, uint32_t timeout);
HAL_StatusTypeDef NRF24L01_ReadRegister(uint8_t reg, uint8_t* rxValue, uint8_t length);
HAL_StatusTypeDef NRF24L01_WriteRegister(uint8_t reg, uint8_t* txValue, uint8_t length);
HAL_StatusTypeDef NRF24L01_SetRxAddr(uint8_t pipe, uint8_t* rxAddr);
HAL_StatusTypeDef NFR24L01_SendCommand(uint8_t cmd);
HAL_StatusTypeDef NFR24L01_FlushTx(void);
HAL_StatusTypeDef NFR24L01_FlushRx(void);
HAL_StatusTypeDef NFR24L01_SetChannel(uint8_t channel);
HAL_StatusTypeDef NRF24L01_Reset(void);
HAL_StatusTypeDef NRF24L01_SetTxMode(uint8_t* addr, uint8_t channel);
HAL_StatusTypeDef NRF24L01_WritePayload(uint8_t* data, uint8_t length);
HAL_StatusTypeDef NRF24L01_ReadPayload(uint8_t* data, uint8_t length);
HAL_StatusTypeDef NRF24L01_Transmit(uint8_t* data);
uint8_t NF24L01_IsDataAvailable(uint8_t pipe);

#endif /* INC_NRF24L01_H_ */
