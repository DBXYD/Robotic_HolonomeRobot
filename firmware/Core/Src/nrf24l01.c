#include "nrf24l01.h"
#include "main.h"
// https://github.com/maniacbug/RF24/blob/master/RF24.cpp1
// https://github.com/maniacbug/RF24/blob/master/examples/led_remote/led_remote.pde
// https://github.com/controllerstech/NRF24L01/blob/master/NRF24L01.c
// https://www.youtube.com/watch?v=mB7LsiscM78
extern uint8_t uartTxBuffer[64];

HAL_StatusTypeDef NRF24L01_Init(void){
	HAL_StatusTypeDef ret;
	uint8_t txBuffer[1];

	NRF24L01_Reset();

	txBuffer[0]=0x00;
	NRF24L01_WriteRegister(CONFIG, txBuffer, 1);  // will be configured later
	txBuffer[0]=0x00;
	NRF24L01_WriteRegister(EN_AA, txBuffer, 1);  // No Auto ACK
	txBuffer[0]=0x00;
	NRF24L01_WriteRegister(EN_RXADDR, txBuffer, 1);  // Not Enabling any data pipe right now
	txBuffer[0]=0x03;
	NRF24L01_WriteRegister(SETUP_AW, txBuffer, 1);  // 5 Bytes for the TX/RX address
	txBuffer[0]=0x00;
	NRF24L01_WriteRegister(SETUP_RETR, txBuffer, 1);   // No retransmission
	txBuffer[0]=0x00;
	NRF24L01_WriteRegister(RF_CH, txBuffer, 1);  // will be setup during Tx or RX
	txBuffer[0]=0x0E;
	ret = NRF24L01_WriteRegister(RF_SETUP, txBuffer, 1);   // Power= 0db, data rate = 2Mbps

	return ret;
}

HAL_StatusTypeDef NRF24L01_WriteCSN(GPIO_PinState PinState){
	HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, PinState);
	return HAL_OK;
}

HAL_StatusTypeDef NRF24L01_WriteCE(GPIO_PinState PinState){
	HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, PinState);
	return HAL_OK;
}

HAL_StatusTypeDef NRF24L01_GetStatus(uint8_t* rxBuffer, uint32_t timeout){
	HAL_StatusTypeDef ret;

	NRF24L01_WriteCSN(RESET);
	ret = HAL_SPI_Receive(&hspi1, rxBuffer, 1, timeout);
	NRF24L01_WriteCSN(SET);

	return ret;
}

HAL_StatusTypeDef NRF24L01_ReadRegister(uint8_t reg, uint8_t* rxValue, uint8_t length){
	HAL_StatusTypeDef ret;
	uint8_t maskedReg;// = (R_REGISTER | (reg & REGISTER_MASK));
	while(length--){
		maskedReg = (R_REGISTER | (reg++ & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, (rxValue++), 1, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
	}
	return ret;
}

HAL_StatusTypeDef NRF24L01_SetRxAddr(uint8_t pipe, uint8_t* rxAddr){
	HAL_StatusTypeDef ret;
	uint8_t maskedReg;
	uint8_t rxAddrFlip[5];

	switch(pipe){
	case 0:
		maskedReg = (R_REGISTER | (RX_ADDR_P0 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 5, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		break;
	case 1:
		maskedReg = (R_REGISTER | (RX_ADDR_P1 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 5, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		break;
	case 2:
		maskedReg = (R_REGISTER | (RX_ADDR_P1 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 5, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		maskedReg = (R_REGISTER | (RX_ADDR_P2 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 1, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		break;
	case 3:
		maskedReg = (R_REGISTER | (RX_ADDR_P1 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 5, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 5, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		maskedReg = (R_REGISTER | (RX_ADDR_P3 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 1, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		break;
	case 4:
		maskedReg = (R_REGISTER | (RX_ADDR_P1 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 5, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 5, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		maskedReg = (R_REGISTER | (RX_ADDR_P4 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 1, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		break;
	case 5:
		maskedReg = (R_REGISTER | (RX_ADDR_P1 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 5, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 5, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		maskedReg = (R_REGISTER | (RX_ADDR_P5 & REGISTER_MASK));
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
		ret = HAL_SPI_Receive(&hspi1, rxAddrFlip, 1, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
		break;

	default :
		ret = HAL_ERROR;
		break;
	}

	// Flip LSB and MSB 8-bits datas
	rxAddr[0]=rxAddrFlip[4];
	rxAddr[1]=rxAddrFlip[3];
	rxAddr[2]=rxAddrFlip[2];
	rxAddr[3]=rxAddrFlip[1];
	rxAddr[4]=rxAddrFlip[0];

	return ret;
}

HAL_StatusTypeDef NRF24L01_SetTxAddr(uint8_t* txAddr){
	HAL_StatusTypeDef ret;
	uint8_t maskedReg = (W_REGISTER | (TX_ADDR & REGISTER_MASK));
	uint8_t txAddrFlip[5];

	txAddrFlip[0]=txAddr[4];
	txAddrFlip[1]=txAddr[3];
	txAddrFlip[2]=txAddr[2];
	txAddrFlip[3]=txAddr[1];
	txAddrFlip[4]=txAddr[0];

	NRF24L01_WriteCSN(RESET);
	ret = HAL_SPI_Transmit(&hspi1, &maskedReg, 1, HAL_MAX_DELAY);
	ret = HAL_SPI_Transmit(&hspi1, txAddrFlip, 5, HAL_MAX_DELAY);
	NRF24L01_WriteCSN(SET);

	return ret;
}

HAL_StatusTypeDef NRF24L01_WriteRegister(uint8_t reg, uint8_t* txValue, uint8_t length){
	HAL_StatusTypeDef ret;
	uint8_t txBuffer[2];
	while(length--){
		txBuffer[0] = (W_REGISTER | (reg++ & REGISTER_MASK));
		txBuffer[1] = *(txValue++);
		NRF24L01_WriteCSN(RESET);
		ret = HAL_SPI_Transmit(&hspi1, txBuffer, 2, HAL_MAX_DELAY);
		NRF24L01_WriteCSN(SET);
	}

	return ret;
}

HAL_StatusTypeDef NFR24L01_SendCommand(uint8_t cmd){
	HAL_StatusTypeDef ret;
	NRF24L01_WriteCSN(RESET);
	ret = HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
	NRF24L01_WriteCSN(SET);
	return ret;
}


HAL_StatusTypeDef NFR24L01_FlushTx(void){
	return NFR24L01_SendCommand(FLUSH_TX);
}

HAL_StatusTypeDef NFR24L01_FlushRx(void){
	return NFR24L01_SendCommand(FLUSH_RX);
}

HAL_StatusTypeDef NFR24L01_SetChannel(uint8_t channel){
	channel%=128;
	return NRF24L01_WriteRegister(RF_CH, &channel, 1);
}

HAL_StatusTypeDef NRF24L01_Reset(void){
	HAL_StatusTypeDef ret;
	uint8_t txBuffer[5];
	txBuffer[0] = 0x08;
	NRF24L01_WriteRegister(CONFIG, txBuffer, 1);
	txBuffer[0] = 0x3F;
	NRF24L01_WriteRegister(EN_AA, txBuffer, 1);
	txBuffer[0] = 0x03;
	NRF24L01_WriteRegister(EN_RXADDR, txBuffer, 1);
	txBuffer[0] = 0x03;
	NRF24L01_WriteRegister(SETUP_AW, txBuffer, 1);
	txBuffer[0] = 0x03;
	NRF24L01_WriteRegister(SETUP_RETR, txBuffer, 1);
	txBuffer[0] = 0x02;
	NRF24L01_WriteRegister(RF_CH, txBuffer, 1);
	txBuffer[0] = 0x0E;
	NRF24L01_WriteRegister(RF_SETUP, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(STATUS, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(OBSERVE_TX, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(CD, txBuffer, 1);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF24L01_WriteRegister(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	NRF24L01_WriteRegister(RX_ADDR_P1, rx_addr_p1_def, 5);
	txBuffer[0] = 0xC3;
	NRF24L01_WriteRegister(RX_ADDR_P2, txBuffer, 1);
	txBuffer[0] = 0xC4;
	NRF24L01_WriteRegister(RX_ADDR_P3, txBuffer, 1);
	txBuffer[0] = 0xC5;
	NRF24L01_WriteRegister(RX_ADDR_P4, txBuffer, 1);
	txBuffer[0] = 0xC6;
	NRF24L01_WriteRegister(RX_ADDR_P5, txBuffer, 1);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF24L01_WriteRegister(TX_ADDR, tx_addr_def, 5);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(RX_PW_P0, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(RX_PW_P1, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(RX_PW_P2, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(RX_PW_P3, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(RX_PW_P4, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(RX_PW_P5, txBuffer, 1);
	txBuffer[0] = 0x11;
	NRF24L01_WriteRegister(FIFO_STATUS, txBuffer, 1);
	txBuffer[0] = 0x00;
	NRF24L01_WriteRegister(DYNPD, txBuffer, 1);
	txBuffer[0] = 0x00;
	ret = NRF24L01_WriteRegister(FEATURE, txBuffer, 1);
	return ret;
}

HAL_StatusTypeDef NRF24L01_SetTxMode(uint8_t* addr, uint8_t channel){
	HAL_StatusTypeDef ret;
	uint8_t config;
	NRF24L01_WriteCE(RESET);
	NFR24L01_SetChannel(channel);
	NRF24L01_SetTxAddr(addr);
	NRF24L01_ReadRegister(CONFIG, &config, 1);
	config |= 0x02; // POWER UP
	config &= 0xF2; // DISABLE CRC and Set TX Mode
	ret = NRF24L01_WriteRegister(CONFIG, &config, 1);
	NRF24L01_WriteCE(SET);
	return ret;
}

HAL_StatusTypeDef NRF24L01_SetRxMode(uint8_t* addr, uint8_t channel, uint8_t pipe){
	HAL_StatusTypeDef ret;
	uint8_t config;
	uint8_t status=0x00;
	uint8_t payloadSize = PAYLOAD_SIZE;
	NRF24L01_WriteCE(RESET);
	NRF24L01_WriteRegister(STATUS, &status, 1);
	NFR24L01_SetChannel(channel);
	NRF24L01_SetRxAddr(pipe, addr);

	switch(pipe){
	case 0:
		NRF24L01_WriteCSN(RESET);
		NRF24L01_WriteRegister(RX_PW_P0, &payloadSize, 1);
		NRF24L01_WriteCSN(SET);
		break;
	case 1:
		NRF24L01_WriteCSN(RESET);
		NRF24L01_WriteRegister(RX_PW_P1, &payloadSize, 1);
		NRF24L01_WriteCSN(SET);
		break;
	case 2:
		NRF24L01_WriteCSN(RESET);
		NRF24L01_WriteRegister(RX_PW_P2, &payloadSize, 1);
		NRF24L01_WriteCSN(SET);
		break;
	case 3:
		NRF24L01_WriteCSN(RESET);
		NRF24L01_WriteRegister(RX_PW_P3, &payloadSize, 1);
		NRF24L01_WriteCSN(SET);
		break;
	case 4:
		NRF24L01_WriteCSN(RESET);
		NRF24L01_WriteRegister(RX_PW_P4, &payloadSize, 1);
		NRF24L01_WriteCSN(SET);
		break;
	case 5:
		NRF24L01_WriteCSN(RESET);
		NRF24L01_WriteRegister(RX_PW_P5, &payloadSize, 1);
		NRF24L01_WriteCSN(SET);
		break;

	default :
		ret = HAL_ERROR;
		break;
	}

	NRF24L01_ReadRegister(CONFIG, &config, 1);
	config |= 0x03; // POWER UP and Set RX Mode
	config &= 0xF3; // DISABLE CRC
	ret = NRF24L01_WriteRegister(CONFIG, &config, 1);
	NRF24L01_WriteCE(SET);

	return ret;
}

HAL_StatusTypeDef NRF24L01_WritePayload(uint8_t* data, uint8_t length){
	HAL_StatusTypeDef ret;
	uint8_t fifoStatus;
	uint8_t payload[PAYLOAD_SIZE];
	if(length>PAYLOAD_SIZE) length=PAYLOAD_SIZE;
	memcpy(payload,data,length);
	memcpy(payload+length,NULL,PAYLOAD_SIZE-length);
	NFR24L01_SendCommand(W_TX_PAYLOAD);
	NRF24L01_WriteCSN(RESET);
	HAL_SPI_Transmit(&hspi1, payload, PAYLOAD_SIZE, 1000);
	NRF24L01_WriteCSN(SET);
	HAL_Delay(1);
	NRF24L01_ReadRegister(FIFO_STATUS, &fifoStatus, 1);

//	uint8_t stringLength = snprintf((char *) uartTxBuffer,32,"Fifo Status : 0x%2x\r\n",fifoStatus);
//	HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
//	stringLength = snprintf((char *) uartTxBuffer,32,"Fifo Status : 0x%2x\r\n",0x01&(1<<TX_EMPTY));
//	HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
//	HAL_Delay(100);


	if((fifoStatus&(1<<TX_EMPTY))){
		return HAL_OK;
	}
	else{
		NFR24L01_FlushTx();
		return HAL_ERROR;
	}

}

HAL_StatusTypeDef NRF24L01_ReadPayload(uint8_t* data, uint8_t length){
	HAL_StatusTypeDef ret;
	uint8_t status;
	uint8_t payload[PAYLOAD_SIZE];
	memcpy(payload,0xFF,PAYLOAD_SIZE);
	NRF24L01_WriteCSN(RESET);
	NFR24L01_SendCommand(R_RX_PAYLOAD);
	ret = HAL_SPI_Receive(&hspi1, payload, PAYLOAD_SIZE, 1000);
	NRF24L01_WriteCSN(SET);
	NFR24L01_FlushRx();
	memcpy(data,payload,length);
	return ret;
}

uint8_t NF24L01_IsDataAvailable(uint8_t pipe){
	uint8_t rxStatus;
	uint8_t txStatus;
	NRF24L01_ReadRegister(STATUS, &rxStatus, 1);
	//if ((rxStatus&(1<<RX_DR))&&(rxStatus&(pipe<<1))){
	if (rxStatus&(1<<RX_DR)){
		txStatus = 1<<6;
		NRF24L01_WriteRegister(STATUS, &txStatus, 1);
		return HAL_OK;
	}
	return HAL_ERROR;
}











