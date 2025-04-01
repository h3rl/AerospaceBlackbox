/*
 * Gloval_Var.h
 *
 *  Created on: Apr 1, 2025
 *      Author: Thomas
 */

#ifndef INC_GLOBAL_VAR_H_
#define INC_GLOBAL_VAR_H_

//Variabel
uint8_t Start_Flight_Recording=0;
uint16_t Page_Index=0;
uint8_t TxData[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44};
uint8_t RxData[8];
uint8_t write_data[2048]={[0 ... 2047] = 0xFF};
uint8_t Read_data[2048];
uint16_t Buffer_Index=0;
uint16_t Block_Mem=0;


uint8_t SR_1=0;
uint8_t SR_2=0;
uint8_t SR_3=0;
uint8_t SR_1_Addr = 0xA0;
uint8_t SR_2_Addr = 0xB0;
uint8_t SR_3_Addr = 0xC0;

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;

#endif /* INC_GLOBAL_VAR_H_ */
