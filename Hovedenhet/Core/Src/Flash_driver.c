/*
 * Flash_driver.c
 *
 *  Created on: Feb 22, 2025
 *      Author: Thomas
 */
#include "main.h"
#include "EX_Global_Var.h"
#include "Flash_driver.h"

//Private Function prototype
uint8_t Read_Status_Register(uint8_t SR);
void Write_Status_Register(uint8_t SR, uint8_t REG_DATA);
void Write_Data_Buffer(uint16_t Buffer_Addr, uint8_t *Data, uint16_t len);
void Program_Page_Flash(uint16_t Page_Addr);
void Select_Page_Read(uint16_t Page_Addr);
void Read_Data_Buffer(uint8_t *Data, uint16_t len);
void Block_Erase(uint16_t Page_Addr);

//Flash OPCODE constants
enum{
	OP_Read_Register = 0x0F,
	OP_Write_Register = 0x1F,
	OP_Dev_Res = 0xFF,
	OP_JEDEC_ID = 0x9F,
	OP_Write_Enable = 0x06,
	OP_Write_Disable = 0x04,
	OP_Block_Erase = 0xD8,
	OP_Load_Program_Data = 0x02,
	OP_Program_Ex = 0x10,
	OP_Page_Data_Read = 0x13,
	OP_Read_Data = 0x03,
	OP_Fast_Read = 0x0B
};


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////User friendly code for implementation///////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


/*Initialize flash IC
BUF=1-> Buffer Read
BUF=0-> Continuous Read*/
void Flash_Init(uint8_t BUF){
	W25N_WaitForReady();
	//Retrive data from register 2 and set BUF=1
	if(BUF){
		uint8_t data = Read_Status_Register(SR.SR_2_Addr);
		data|=0x08;
		Write_Status_Register(SR.SR_2_Addr, data);
	}
	//Retrive data from register 2 and set BUF=0
	else{
		uint8_t data = Read_Status_Register(SR.SR_2_Addr);
		data&=0xF7;
		Write_Status_Register(SR.SR_2_Addr, data);
	}
	/*Retrive data from register 1 and set WP-E=1, BP3=0, BP2=0, BP1=0, BP0 and TP=0.
	This unlocks every block for writing and activates write protect switch*/
	uint8_t data = Read_Status_Register(SR.SR_1_Addr);
	data|=0x02;
	data&=0x83;
	Write_Status_Register(SR.SR_1_Addr, data);

	//Reading the flash chip to find next available page

	//Temp variables
	uint8_t Page_Data[16]={[0 ... 15] = 0x00};
	uint16_t Page_Bit=0x0000;
	uint16_t Temp_Page=0;

	/*While loop running through first page of each block. When the first 16 bytes on a page = 0xFF,
	go back to previous block (Temp_Page -= 64) and exit while loop.*/
	while(Page_Bit!=0xFFFF){
		Page_Bit = 0x0000;

		Read_Data(Temp_Page, &Page_Data[0], sizeof(Page_Data));

		for(int i = 0; i < 16; i++){
			if(Page_Data[i]==0xFF){
				Page_Bit |= 0x01 << i;
			}
		}

		if(Page_Bit!=0xFFFF){
			Temp_Page += 64;
		}

		else{
			if(Temp_Page == 0){
				break;
			}
			else Temp_Page -= 64;
		}
	}
	Page_Bit=0x0000;
	/*While loop running through every page of the block. When the first 16 bytes = 0xFF,
	exit while loop. This page will be the first available page on flash IC.*/
	while(Page_Bit!=0xFFFF){
		Page_Bit = 0x0000;

		Read_Data(Temp_Page, &Page_Data[0], sizeof(Page_Data));

		for(int i = 0; i < 16; i++){
			if(Page_Data[i]==0xFF){
				Page_Bit |= 0x01 << i;
			}
		}
		if(Page_Bit!=0xFFFF){
			Temp_Page++;
		}

	}
	//Update global variables
	Flash.Page_Index=Temp_Page;
	Flash.Block_Mem=(Flash.Page_Index/64);
	USART3_Printf("Current page is: %u\r\n", Flash.Page_Index);
}

//Read all status registers
void Read_Register(SR_Data SR){
	SR.SR_1 = Read_Status_Register(SR.SR_1_Addr);
	delay_ns(DELAY_NS);
	SR.SR_2 = Read_Status_Register(SR.SR_2_Addr);
	delay_ns(DELAY_NS);
	SR.SR_3 = Read_Status_Register(SR.SR_3_Addr);
	delay_ns(DELAY_NS);
}

//Write data to buffer in microcontroller
void Write_Data(uint8_t* data, uint16_t lenght){
	if((Flash.Page_Index==0)&&(Flash.Buffer_Index==0)){
		Block_Erase(0);
	}
	uint16_t count=0;
	while(count<lenght){
		*Flash.Buffer_p=*data;
		Flash.Buffer_p++;
		data++;
		Flash.Buffer_Index++;
		count++;
		if(Flash.Buffer_Index>=2048){
			Write_to_page();
		}
	}
}

//Write data to buffer in flash IC, then write buffer to page
void Write_to_page(void){
	if(Flash.Buffer_Select==0){
		Flash.Buffer_Select=1;
		Flash.Buffer_p=Flash.Buffer_1;
		Flash.Buffer_Index=0;
		Write_Data_Buffer(0, Flash.Buffer_0, sizeof(Flash.Buffer_0));
	}
	else{
		Flash.Buffer_Select=0;
		Flash.Buffer_p=Flash.Buffer_0;
		Flash.Buffer_Index=0;
		Write_Data_Buffer(0, Flash.Buffer_1, sizeof(Flash.Buffer_1));
	}
	Program_Page_Flash(Flash.Page_Index);
	Flash.Page_Index++;
	Flash.Buffer_Index=0;
	Automatic_Block_Managment(Flash.Page_Index);
}

//Read data from page and transfer to data
void Read_Data(uint16_t page, uint8_t* data, uint16_t len){
	Select_Page_Read(page);
	Read_Data_Buffer(data, len);
}

//Check if page is located in new block. If it is located in new block, erase block
void Automatic_Block_Managment(uint16_t Page_Index){
	uint16_t Block=Page_Index/64;
	if(!(Flash.Block_Mem==Block)){
		Block_Erase(Page_Index);
		Flash.Block_Mem=Block;
	}
	else{
		Flash.Block_Mem=Block;
	}
}

//Erase all flash memory on IC
void Chip_Erase(void){
	USART3_Printf("Vil du slette alt minne for lagra flydata? Y/N\r\n");
	HAL_UART_Receive(&huart3, &command, 1, HAL_MAX_DELAY);
	//ASCII for Y
	if(command == 0x59){
		USART3_Printf("Sletter minne ...\r\n");
		for(int i = 0; i <= 1024; i++){
			Block_Erase(i*64);
		}
		Flash.Buffer_Index=0;
		Flash.Page_Index=0;
		Flash.Block_Mem=0;
		Flash.Buffer_Select=0;
		Flash.Buffer_p=Flash.Buffer_0;

		Flash_Data* pointer = &Flash;
		memset(pointer->Buffer_0, 0xFF, sizeof(pointer->Buffer_0));
		memset(pointer->Buffer_1, 0xFF, sizeof(pointer->Buffer_1));
		USART3_Printf("Ferdig\r\n");
	}
	else{
		USART3_Printf("Sletter IKKE minne\r\n");
	}
}

//Read data continuous from IC, then print data to Virtual COM PORT
void Read_Data_Cont(uint16_t len){
	Select_Page_Read(0);
	uint8_t Data_Buffer[len];

	SPI.Tx_Buffer[0]=OP_Read_Data;
	SPI.Tx_Buffer[1]=0x00;
	SPI.Tx_Buffer[2]=0x00;
	SPI.Tx_Buffer[3]=0x00;
	csLOW();

	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer,4,100);
	HAL_SPI_Receive(&hspi1, Data_Buffer, len, HAL_MAX_DELAY);

	uint16_t CAN_Temp = *(uint16_t*)&Data_Buffer[1];
	uint32_t Data_Temp = *(uint32_t*)&Data_Buffer[3];
	uint32_t Time_Temp = *(uint32_t*)&Data_Buffer[11];

	while((Data_Buffer[0]==0xF0)&&(Data_Buffer[15]==0x0F)){
		USART3_Printf("CANID:%u, DATA:%u, Time:%u\r\n", (unsigned int)CAN_Temp, (unsigned int)Data_Temp, (unsigned int)Time_Temp);
		HAL_SPI_Receive(&hspi1, Data_Buffer, len, HAL_MAX_DELAY);

		CAN_Temp = *(uint16_t*)&Data_Buffer[1];
		Data_Temp = *(uint32_t*)&Data_Buffer[3];
		Time_Temp = *(uint32_t*)&Data_Buffer[11];
	}
	csHIGH();
}


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////Low-level code for interfacing with Flash IC////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

//Enable WEL(Write enable latch) in flash IC
void Write_Enable(void){
	SPI.Tx_Buffer[0] = OP_Write_Enable;
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 1, 100);
	csHIGH();
	delay_ns(DELAY_NS);
}

//Disable WEL(Write enable latch) in flash IC
void Write_Disable(void){
	SPI.Tx_Buffer[0] = OP_Write_Disable;
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 1, 100);
	csHIGH();
	delay_ns(DELAY_NS);
}

/*Read status register.
SR->Select register address to read*/
uint8_t Read_Status_Register(uint8_t SR){
	SPI.Tx_Buffer[0]=OP_Read_Register;
	SPI.Tx_Buffer[1]=SR;
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 2, 100);
	HAL_SPI_Receive(&hspi1, SPI.Rx_Buffer, 1, 100);
	csHIGH();
	return SPI.Rx_Buffer[0];
}

/*Write to status register
SR->register address to write
REG_DATA->Register data to write to register*/
void Write_Status_Register(uint8_t SR, uint8_t REG_DATA){
	SPI.Tx_Buffer[0]=OP_Write_Register;
	SPI.Tx_Buffer[1]=SR;
	SPI.Tx_Buffer[2]=REG_DATA;
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 3, 100);
	csHIGH();
	delay_ns(DELAY_NS);
}

//Read JEDEC ID from flash IC. Useful to check that Flash IC is connected correctly
uint32_t Read_ID(void){
	uint8_t Buffer[3];
	SPI.Tx_Buffer[0] = OP_JEDEC_ID;
	SPI.Tx_Buffer[1] = 0x00;
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 2, 100);
	HAL_SPI_Receive(&hspi1, &Buffer[0], 4, 100);
	csHIGH();
	delay_ns(DELAY_NS);
	return ((Buffer[0]<<16)|(Buffer[1]<<8|Buffer[2]));
}

//Write data to buffer in flash IC
void Write_Data_Buffer(uint16_t Buffer_Addr, uint8_t *Data, uint16_t len){
	Write_Enable();
	SPI.Tx_Buffer[0]=OP_Load_Program_Data;
	SPI.Tx_Buffer[1]=(uint8_t)(Buffer_Addr>>8);
	SPI.Tx_Buffer[2]=(uint8_t)Buffer_Addr;
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 3, 100);
	HAL_SPI_Transmit(&hspi1, Data, len, HAL_MAX_DELAY);
	csHIGH();
	delay_ns(DELAY_NS);
}

//Program page with data in buffer
void Program_Page_Flash(uint16_t Page_Addr){
	Write_Enable();
	SPI.Tx_Buffer[0]=OP_Program_Ex;
	SPI.Tx_Buffer[1]=0x00;
	SPI.Tx_Buffer[2]=(uint8_t)(Page_Addr>>8);
	SPI.Tx_Buffer[3]=(uint8_t)(Page_Addr);
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 4, 100);
	csHIGH();
	W25N_WaitForReady();
}

//Select page to read
void Select_Page_Read(uint16_t Page_Addr){
	SPI.Tx_Buffer[0]=OP_Page_Data_Read;
	SPI.Tx_Buffer[1]=0x00;
	SPI.Tx_Buffer[2]=(uint8_t)(Page_Addr>>8);
	SPI.Tx_Buffer[3]=(uint8_t)(Page_Addr);
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer,4,100);
	csHIGH();
	W25N_WaitForReady();
}

/*Read data from selected page
NB: First use Select_Page_Read to select page*/
void Read_Data_Buffer(uint8_t *Data, uint16_t len){
	SPI.Tx_Buffer[0]=OP_Read_Data;
	SPI.Tx_Buffer[1]=0x00;
	SPI.Tx_Buffer[2]=0x00;
	SPI.Tx_Buffer[3]=0x00;
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer,4,100);
	HAL_SPI_Receive(&hspi1, Data, len, HAL_MAX_DELAY);
	csHIGH();
}

/*Erase Block where page is located
Page_Addr-> Address to page, where the block which includes page is erased*/
void Block_Erase(uint16_t Page_Addr){
	Write_Enable();
	SPI.Tx_Buffer[0]=OP_Block_Erase;
	SPI.Tx_Buffer[1]=0x00;
	SPI.Tx_Buffer[2]=(uint8_t)(Page_Addr>>8);
	SPI.Tx_Buffer[3]=(uint8_t)(Page_Addr);
	csLOW();
	HAL_SPI_Transmit(&hspi1, SPI.Tx_Buffer, 4, 100);
	csHIGH();
	W25N_WaitForReady();
}
