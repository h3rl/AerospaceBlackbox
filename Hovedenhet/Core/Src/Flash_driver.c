/*
 * Flash_driver.c
 *
 *  Created on: Feb 22, 2025
 *      Author: Thomas
 */
#include "flash_driver.h"

#include "spi.h"

#define numBLOCK 1024  // Antall blokker
#define numPAGES 65536 // Antall sider totalt. 64 sider per. blokk
#define numBYTES 2048  // Antall bytes per. side

#define SR_1_Addr 0xA0
#define SR_2_Addr 0xB0
#define SR_3_Addr 0xC0

flash_data_t flash = {0};

//Private Function prototype
uint8_t Read_Status_Register(uint8_t sr_address);
void Write_Status_Register(uint8_t sr_address, uint8_t data);
void Write_Data_Buffer(uint16_t buffer_addr, uint8_t *pData, uint16_t size);
void Program_Page_Flash(uint16_t Page_Addr);
void Select_Page_Read(uint16_t Page_Addr);
void Read_Data_Buffer(uint8_t *Data, uint16_t len);
void Block_Erase(uint16_t Page_Addr);
void delay_ns(uint32_t ns);
void W25N_WaitForReady(void);


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
		uint8_t data = Read_Status_Register(SR_2_Addr);
		data|=0x08;
		Write_Status_Register(SR_2_Addr, data);
	}
	//Retrive data from register 2 and set BUF=0
	else{
		uint8_t data = Read_Status_Register(SR_2_Addr);
		data&=0xF7;
		Write_Status_Register(SR_2_Addr, data);
	}
	/*Retrive data from register 1 and set WP-E=1, BP3=0, BP2=0, BP1=0, BP0 and TP=0.
	This unlocks every block for writing and activates write protect switch*/
	uint8_t data = Read_Status_Register(SR_1_Addr);
	data|=0x02;
	data&=0x83;
	Write_Status_Register(SR_1_Addr, data);

	//Reading the flash chip to find next available page

	//Temp variables
	uint8_t page_data[16] = {0};
	uint16_t page_bit = 0;
	uint16_t temp_page = 0;

	/*While loop running through first page of each block. When the first 16 bytes on a page = 0xFF,
	go back to previous block (Temp_Page -= 64) and exit while loop.*/
	while(page_bit != 0xFFFF){
		page_bit = 0;

		Read_Data(temp_page, &page_data[0], sizeof(page_data));

		for(int i = 0; i < 16; i++){
			if(page_data[i]==0xFF){
				page_bit |= 0x01 << i;
			}
		}
		if(page_bit!=0xFFFF){
			temp_page += 64;
		}
		else{
			if(temp_page == 0) break;
			else temp_page -= 64;
		}
	}


	page_bit=0x0000;
	/*While loop running through every page of the block. When the first 16 bytes = 0xFF,
	exit while loop. This page will be the first available page on flash IC.*/
	while(page_bit!=0xFFFF){
		page_bit = 0x0000;

		Read_Data(temp_page, &page_data[0], sizeof(page_data));

		for(int i = 0; i < 16; i++){
			if(page_data[i]==0xFF){
				page_bit |= 0x01 << i;
			}
		}
		if(page_bit!=0xFFFF){
			temp_page++;
		}

	}
	//Update global variables
	flash.Page_Index=temp_page;
	flash.Block_Mem=(flash.Page_Index/64);
	print("Current page is: %u\r\n", flash.Page_Index);
}

//Read all status registers
void Read_All_Status_Register(void){
	flash.SR_1 = Read_Status_Register(SR_1_Addr);
	delay_ns(DELAY_NS);
	flash.SR_2 = Read_Status_Register(SR_2_Addr);
	delay_ns(DELAY_NS);
	flash.SR_3 = Read_Status_Register(SR_3_Addr);
	delay_ns(DELAY_NS);
}

//Write data to buffer in microcontroller
void Write_Data(uint8_t* data, uint16_t lenght){
	if(flash.Page_Index == 0 && flash.Buffer_Index == 0){
		Block_Erase(0);
	}
	uint16_t count=0;
	while(count<lenght){
		if(flash.Memory_Full == 1){
			return;
		}

		*flash.Buffer_p=*data;
		flash.Buffer_p++;
		data++;
		flash.Buffer_Index++;
		count++;
		if(flash.Buffer_Index>=2048){
			Write_to_page();
		}
	}
}

//Write data to buffer in flash IC, then write buffer to page
void Write_to_page(void){

	//Check if it is the last page
	if(flash.Page_Index >= 0xFFFF){

		//Set memory full flag and disable flight recording
		flash.Memory_Full = 1;
//		Start_Flight_Recording = 0;

		//Store data to the last page and return
		if(flash.Buffer_Select==0){
			Write_Data_Buffer(0, flash.Buffer_0, sizeof(flash.Buffer_0));
		}
		else{
			Write_Data_Buffer(0, flash.Buffer_1, sizeof(flash.Buffer_1));
		}
		Program_Page_Flash(flash.Page_Index);
		return;
	}
	if(flash.Buffer_Select==0){
		flash.Buffer_Select=1;
		flash.Buffer_p=flash.Buffer_1;
		flash.Buffer_Index=0;
		Write_Data_Buffer(0, flash.Buffer_0, sizeof(flash.Buffer_0));
	}
	else{
		flash.Buffer_Select=0;
		flash.Buffer_p=flash.Buffer_0;
		flash.Buffer_Index=0;
		Write_Data_Buffer(0, flash.Buffer_1, sizeof(flash.Buffer_1));
	}
	Program_Page_Flash(flash.Page_Index);
	flash.Page_Index++;
	flash.Buffer_Index=0;
	Automatic_Block_Managment(flash.Page_Index);
}

//Read data from page and transfer to data
void Read_Data(uint16_t page, uint8_t* data, uint16_t len){
	Select_Page_Read(page);
	Read_Data_Buffer(data, len);
}

//Check if page is located in new block. If it is located in new block, erase block
void Automatic_Block_Managment(uint16_t Page_Index){
	uint16_t Block=Page_Index/64;
	if(!(flash.Block_Mem==Block)){
		Block_Erase(Page_Index);
		flash.Block_Mem=Block;
	}
	else{
		flash.Block_Mem=Block;
	}
}

//Erase all flash memory on IC
void Chip_Erase(void){
	print("Sletter minne ...\r\n");
	for(int i = 0; i <= 1024; i++){
		Block_Erase(i*64);
	}
	flash.Buffer_Index=0;
	flash.Page_Index=0;
	flash.Block_Mem=0;
	flash.Buffer_Select=0;
	flash.Memory_Full=0;
	flash.Buffer_p=flash.Buffer_0;

	memset(&flash.Buffer_0, 0xFF, sizeof(flash.Buffer_0));
	memset(&flash.Buffer_1, 0xFF, sizeof(flash.Buffer_1));
	print("Ferdig\r\n");
}

//Read data continuous from IC, then print data to Virtual COM PORT
void Read_Data_Cont(uint16_t length){
	Select_Page_Read(0);
	uint8_t data_buffer[length];
	uint8_t tx_buffer[4] = {0};
	tx_buffer[0] = OP_Read_Data;

	csLOW();

	HAL_SPI_Transmit(&hspi1, tx_buffer,4,100);
	HAL_SPI_Receive(&hspi1, data_buffer, length, HAL_MAX_DELAY);

	uint16_t CAN_Temp = *(uint16_t*)&data_buffer[1];
	uint32_t Data_Temp = *(uint32_t*)&data_buffer[3];
	uint32_t Time_Temp = *(uint32_t*)&data_buffer[11];

	while((data_buffer[0]==0xF0)&&(data_buffer[length-1]==0x0F)){
		print("CANID:%u, DATA:%u, Time:%u\r\n", (unsigned int)CAN_Temp, (unsigned int)Data_Temp, (unsigned int)Time_Temp);
		HAL_SPI_Receive(&hspi1, data_buffer, length, HAL_MAX_DELAY);

		CAN_Temp = *(uint16_t*)&data_buffer[1];
		Data_Temp = *(uint32_t*)&data_buffer[3];
		Time_Temp = *(uint32_t*)&data_buffer[11];
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
	uint8_t buf = OP_Write_Enable;
	csLOW();
	HAL_SPI_Transmit(&hspi1, &buf, 1, 100);
	csHIGH();
	delay_ns(DELAY_NS);
}

//Disable WEL(Write enable latch) in flash IC
void Write_Disable(void){
	uint8_t buf = OP_Write_Disable;
	csLOW();
	HAL_SPI_Transmit(&hspi1, &buf, 1, 100);
	csHIGH();
	delay_ns(DELAY_NS);
}

/*Read status register.
SR->Select register address to read*/
uint8_t Read_Status_Register(uint8_t sr_address){
	uint8_t buffer[2];
	buffer[0] = OP_Read_Register;
	buffer[1] = sr_address;
	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer, 2, 100);
	uint8_t rx_buffer = 0;
	HAL_SPI_Receive(&hspi1, &rx_buffer, 1, 100);
	csHIGH();
	return rx_buffer;
}

/*Write to status register
SR->register address to write
REG_DATA->Register data to write to register*/
void Write_Status_Register(uint8_t sr_address, uint8_t data){
	uint8_t buffer[3];
	buffer[0] = OP_Write_Register;
	buffer[1] = sr_address;
	buffer[2] = data;
	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer, 3, 100);
	csHIGH();
	delay_ns(DELAY_NS);
}

//Read JEDEC ID from flash IC. Useful to check that Flash IC is connected correctly
uint32_t Read_ID(void){
	uint8_t buffer[4] = {0};
	buffer[0] = OP_JEDEC_ID;
	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer, 2, 100);
	memset(buffer, 0, sizeof(buffer));
	HAL_SPI_Receive(&hspi1, &buffer[0], 4, 100); // double check that we should get 3 or 4 bytes
	csHIGH();
	delay_ns(DELAY_NS);
	return ((buffer[0]<<16)|(buffer[1]<<8|buffer[2]));
}

//Write data to buffer in flash IC
void Write_Data_Buffer(uint16_t buffer_addr, uint8_t *pData, uint16_t size){
	Write_Enable();
	uint8_t buffer[3];
	buffer[0]=OP_Load_Program_Data;
	buffer[1]=(uint8_t)(buffer_addr>>8);
	buffer[2]=(uint8_t)buffer_addr;
	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer, 3, 100);
	HAL_SPI_Transmit(&hspi1, pData, size, HAL_MAX_DELAY);
	csHIGH();
	delay_ns(DELAY_NS);
}

//Program page with data in buffer
void Program_Page_Flash(uint16_t Page_Addr){
	Write_Enable();
	uint8_t buffer[4];
	buffer[0]=OP_Program_Ex;
	buffer[1]=0x00;
	buffer[2]=(uint8_t)(Page_Addr>>8);
	buffer[3]=(uint8_t)(Page_Addr);
	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer, 4, 100);
	csHIGH();
	W25N_WaitForReady();
}

//Select page to read
void Select_Page_Read(uint16_t Page_Addr){
	uint8_t buffer[4];
	buffer[0]=OP_Page_Data_Read;
	buffer[1]=0x00;
	buffer[2]=(uint8_t)(Page_Addr>>8);
	buffer[3]=(uint8_t)(Page_Addr);

	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer,4,100);
	csHIGH();
	W25N_WaitForReady();
}

/*Read data from selected page
NB: First use Select_Page_Read to select page*/
void Read_Data_Buffer(uint8_t *Data, uint16_t len){
	uint8_t buffer[4] = {0};
	buffer[0]=OP_Read_Data;
	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer,4,100);
	HAL_SPI_Receive(&hspi1, Data, len, HAL_MAX_DELAY);
	csHIGH();
}

/*Erase Block where page is located
Page_Addr-> Address to page, where the block which includes page is erased*/
void Block_Erase(uint16_t Page_Addr){
	Write_Enable();
	uint8_t buffer[4] = {0};
	buffer[0]=OP_Block_Erase;
	buffer[1]=0x00;
	buffer[2]=(uint8_t)(Page_Addr>>8);
	buffer[3]=(uint8_t)(Page_Addr);
	csLOW();
	HAL_SPI_Transmit(&hspi1, buffer, 4, 100);
	csHIGH();
	W25N_WaitForReady();
}

void W25N_WaitForReady(void) {
	delay_ns(DELAY_NS);
    while (Read_Status_Register(SR_3_Addr) & 0x01) {
    	delay_ns(DELAY_NS);  // Wait until flash is ready
    }
}


void delay_ns(uint32_t ns) {
    uint32_t cycles_per_ns = SystemCoreClock / 1000000000; // Convert clock speed to cycles per ns
    uint32_t start = DWT->CYCCNT;                         // Get start cycle count
    uint32_t delay_cycles = ns * cycles_per_ns;           // Calculate required cycles

    while ((DWT->CYCCNT - start) < delay_cycles);         // Wait until delay is met
}
