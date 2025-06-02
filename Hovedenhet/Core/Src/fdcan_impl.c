/*
 * CAN.c
 *
 *  Created on: Apr 1, 2025
 *      Author: Thomas
 */

#include "fdcan_impl.h"

static FDCAN_HandleTypeDef* fdcan = 0;
FDCAN_TxHeaderTypeDef tx_header;
FDCAN_RxHeaderTypeDef rx_header;

#if FDCAN_FLEXIBLE_DATARATE
// fd can can have up to 64 bytes of data
uint8_t tx_buffer[64];
uint8_t rx_buffer[64];
#else
// classic can limited to 8 bytes of data











































uint8_t tx_buffer[8];
uint8_t rx_buffer[8];
#endif

/* Private function prototypes */
uint32_t bytes2dlc(uint32_t bytes);
uint32_t dlc2bytes(uint32_t data_length_code);

/* Can functions */
void FDCAN_init(FDCAN_HandleTypeDef* hfdcan)
{
	// Check if already initialised
	assert(fdcan == 0);// Already initialised! not supported yet

	// Save handle
	fdcan = hfdcan;

	/* Setup filters */
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;// do not change
	sFilterConfig.RxBufferIndex = 0;// ignored

	// START FILTERS
	//TODO SET YOUR FILTERS
	// EXAMPLE: Range Filter, IDs in [0x500 to 0x700] in RXFIFO0
	sFilterConfig.FilterIndex = 0; /* 0 to 127, always unique */
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x500; // Start id
	sFilterConfig.FilterID2 = 0x700; // End id

	if(HAL_FDCAN_ConfigFilter(fdcan, &sFilterConfig) != HAL_OK)
	{
		// Failed to setup filter
		Error_Handler();
	}

	//TODO SET YOUR FILTERS
	// EXAMPLE: Mask Filter, IDs in [0x3F0 to 0x3FF] in RXFIFO1
	// baseid=0x3F0, mask=0xFF0 => all IDs matching 0x3F? pass
	sFilterConfig.FilterIndex = 1; /* 0 to 127, always unique */
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x3F0; // Base id
	sFilterConfig.FilterID2 = 0xFF0; // Mask id

	if(HAL_FDCAN_ConfigFilter(fdcan, &sFilterConfig) != HAL_OK)
	{
		// Failed to setup filter
		Error_Handler();
	}

	// END FILTERS
	/* Configure Global Filters */
	if (HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}

	/* Activate new message interrupts */
	if (HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}

	/* Start can and apply/lock filters */
	if (HAL_FDCAN_Start(fdcan) != HAL_OK)
	{
		Error_Handler();
	}




	/* Setup Tx settings */
	memset(&tx_header, 0, sizeof(tx_header));
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
#if FDCAN_FLEXIBLE_DATARATE
	tx_header.FDFormat = FDCAN_FD_CAN;
#else
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.DataLength = FDCAN_DLC_BYTES_8; // changes if fd is enabled
#endif
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // store tx events? might be needed if retransmition is on
    tx_header.MessageMarker = 0;
	tx_header.Identifier = 0;
	tx_header.DataLength = 0;
	// rest of these are set when transmitting
	//tx_header.Identifier = addr;
	//tx_header.DataLength = data_frame_len;
	//tx_header.MessageMarker = id;
}

HAL_StatusTypeDef FDCAN_sendmsg(uint32_t id, void* pData, uint8_t size)
{
	// probably sending to many messages if this assert fails
	assert(HAL_FDCAN_GetTxFifoFreeLevel(fdcan) <= 0);

#if FDCAN_FLEXIBLE_DATARATE
	uint32_t data_length_code = bytes2dlc(size);
	// if assert failed: invalid size, must be any of the following sizes (in bytes)
	// [1,2,3,4,5,6,7,8,12,16,20,24,32,48,64]
	assert(data_length_code != 0);
	tx_header.DataLength = data_length_code;
#endif
	tx_header.Identifier = id;
	tx_header.MessageMarker = (id & 0xFF); // message tag, any 0-255

	// copy data we are going to send
	memcpy(tx_buffer, pData, size);

	// Send and return status
	return HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &tx_header, (uint8_t*)tx_buffer);
}

/* Receive message callbacks */
void _fdcan_rx_fifo_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo)
{
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, RxFifo) > 0)
	{
		// zero buffer first, just so its easier to debug..
		//memset(rx_buffer, 0, sizeof(rx_buffer));
		HAL_FDCAN_GetRxMessage(hfdcan, RxFifo, &rx_header, rx_buffer);
		uint32_t id = rx_header.Identifier;
#if FDCAN_FLEXIBLE_DATARATE
		uint8_t size = dlc2bytes(rx_header.DataLength);
#else
		const uint8_t size = 8;
#endif
		FDCAN_parse_message(id, rx_buffer, size);
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	_fdcan_rx_fifo_callback(hfdcan, FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	_fdcan_rx_fifo_callback(hfdcan, FDCAN_RX_FIFO1);
}

//void HAL_FDCAN_RxBufferCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
//{
//	// might implement later, Rxbuffer is good to use when data should be overwritten.
//	// ie for status flags, state etc.
//}

/* Helper functions */
uint32_t bytes2dlc(uint32_t bytes)
{
	assert(bytes > 0); // why would we ever send 0 bytes?
	switch (bytes) {
		//case 0: return FDCAN_DLC_BYTES_0;
		case 1: return FDCAN_DLC_BYTES_1;
		case 2: return FDCAN_DLC_BYTES_2;
		case 3: return FDCAN_DLC_BYTES_3;
		case 4: return FDCAN_DLC_BYTES_4;
		case 5: return FDCAN_DLC_BYTES_5;
		case 6: return FDCAN_DLC_BYTES_6;
		case 7: return FDCAN_DLC_BYTES_7;
		case 8: return FDCAN_DLC_BYTES_8;
		case 12: return FDCAN_DLC_BYTES_12;
		case 16: return FDCAN_DLC_BYTES_16;
		case 20: return FDCAN_DLC_BYTES_20;
		case 24: return FDCAN_DLC_BYTES_24;
		case 32: return FDCAN_DLC_BYTES_32;
		case 48: return FDCAN_DLC_BYTES_48;
		case 64: return FDCAN_DLC_BYTES_64;
		default: break;
	}
	return 0;
}

uint32_t dlc2bytes(uint32_t data_length_code)
{
	//assert(bytes > 0);
	switch (data_length_code) {
		case FDCAN_DLC_BYTES_0: return 0;
		case FDCAN_DLC_BYTES_1: return 1;
		case FDCAN_DLC_BYTES_2: return 2;
		case FDCAN_DLC_BYTES_3: return 3;
		case FDCAN_DLC_BYTES_4: return 4;
		case FDCAN_DLC_BYTES_5: return 5;
		case FDCAN_DLC_BYTES_6: return 6;
		case FDCAN_DLC_BYTES_7: return 7;
		case FDCAN_DLC_BYTES_8: return 8;
		case FDCAN_DLC_BYTES_12: return 12;
		case FDCAN_DLC_BYTES_16: return 16;
		case FDCAN_DLC_BYTES_20: return 20;
		case FDCAN_DLC_BYTES_24: return 24;
		case FDCAN_DLC_BYTES_32: return 32;
		case FDCAN_DLC_BYTES_48: return 48;
		case FDCAN_DLC_BYTES_64: return 64;
		default: break;
	}
	return 0;
}

/* Weak prototype functions */
__weak void FDCAN_parse_message(uint32_t id, void* pData, uint8_t size)
{
	UNUSED(id);
}
