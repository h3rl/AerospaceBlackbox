/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fdcan.h"
#include "memorymap.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_driver.h"
#include "camera_driver.h"
#include "fdcan_impl.h"
#include "util.h"

#include <sys/unistd.h>
#include <errno.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t g_local_time = 0;
static uint8_t g_flash_logging_enabled = 0;
static uint32_t g_gopro_release_timestamp = 0; // timestamp(ms) when the gopro button should be released
static uint32_t g_last_can_message_timestamp = 0; // timestamp(ms) when the last canmessage was recieved

static camera_data_t CAM1 = {0};
static camera_data_t CAM2 = {0};
static camera_data_t CAM3 = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void process_incomming_commands(uint8_t command);
void send_status(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* fdcan parse message, called for every message recieved */
/* this is a callback function that is called by the fdcan fifo handler */
void FDCAN_parse_message(uint32_t id, void* pData, uint8_t size)
{
	g_last_can_message_timestamp = HAL_GetTick();
	uint8_t* pbData = (uint8_t*)pData;

	// Logging to console
	print("Received CAN Message: ");
    for (int i = 0; i < 8; i++) {
    	print("%02X ", pbData[i]);
    }
    print("\r\n");

	switch(id){
	//CAN ID = 201 is CAN message with GNSS time from flight estimator used to update local time
	case 201:
		uint32_t GNSS_time = *(uint32_t*)pData;

		uint32_t H = GNSS_time/10000000;
		uint32_t M = (GNSS_time/100000) % 100;
		uint32_t S = (GNSS_time/1000) % 100;
		uint32_t MS = GNSS_time % 1000;

		g_local_time = ((H*3600UL + M*60UL + S)*1000UL) + MS;
		break;
	//CAN ID = 401 is CAN message for commands to black box
	case 401:

		if(pbData[6] == pbData[7]){
			uint8_t command = pbData[6];
			process_incomming_commands(command);
		}
		break;
	//CAN ID = 402 is CAN message for manual update of current page
	case 402:
		uint16_t page_index = *(uint16_t*)(pData+6);

		Automatic_Block_Managment(page_index);
		flash.Buffer_Index=0;
		flash.Page_Index=page_index;
		flash.Buffer_Select=0;
		flash.Buffer_p=flash.Buffer_0;
		break;
	default:
		break;
	}

	if(g_flash_logging_enabled != 0){
		// Save message to flash
		uint8_t buffer[16] = {0};
		uint8_t* ptr = buffer;

		//Start byte
		*ptr = 0xF0;
		ptr++;

		//CAN ID Stored in 2 first bytes
		*ptr = (uint16_t)(id & 0xFFFF);
		ptr += sizeof(uint16_t);

		//8 bytes with CAN data
		memcpy(ptr, pData, 8);
		ptr += 8;

		//Clock (uint32_t)
		//uint32_t t = g_local_time;
		uint32_t t = HAL_GetTick();
		memcpy(ptr, &t, sizeof(t));
		ptr += sizeof(uint32_t);

		//Stop byte
		*ptr = 0x0F;
		ptr++;

		//Write to flash
		Write_Data(buffer, sizeof(buffer));
	}
}

void send_status(void)
{
	uint32_t status = 0;

	if(HAL_GPIO_ReadPin(GPIOC, CAM1_PWR_Pin)){
		status |= (1 << 0);
	}
	if(HAL_GPIO_ReadPin(GPIOE, CAM2_PWR_Pin)){
		status |= (1 << 1);
	}
	if(HAL_GPIO_ReadPin(GPIOB, CAM3_PWR_Pin)){
		status |= (1 << 2);
	}
	if(CAM1.status[0] == 0x42){
		status |= (1 << 3);
	}
	if(CAM2.status[0] == 0x42){
		status |= (1 << 4);
	}
	if(CAM3.status[0] == 0x42){
		status |= (1 << 5);
	}
	if(g_last_can_message_timestamp + 1000 < HAL_GetTick()){
		status |= (1 << 6);
	}
	if(g_flash_logging_enabled){
		status |= (1 << 7);
	}

	uint8_t buffer[6];
	*(uint32_t*)(buffer+0) = status;
	*(uint16_t*)(buffer+4) = flash.Page_Index;

	FDCAN_sendmsg(400, buffer, sizeof(buffer));
}

void process_incomming_commands(uint8_t command)
{
    switch (command) {
	//CAM to IDLE - stop recording cameras
      case 0x41:
        CAM_send_command(&CAM1, CAMERA_IDLE);
        CAM_send_command(&CAM2, CAMERA_IDLE);
        CAM_send_command(&CAM3, CAMERA_IDLE);
        break;
  	  //CAM to REC - start recording cameras
	    case 0x42:
        CAM_send_command(&CAM1, CAMERA_RECORDING);
        CAM_send_command(&CAM2, CAMERA_RECORDING);
        CAM_send_command(&CAM3, CAMERA_RECORDING);
        break;

	  //CAM to FORMAT - format sd card
	    case 0x43:
        CAM_send_command(&CAM1, CAMERA_FORMAT);
        CAM_send_command(&CAM2, CAMERA_FORMAT);
        CAM_send_command(&CAM3, CAMERA_FORMAT);
        break;

	  //CAM to REBOOT - restart cameras
      case 0x44:
        CAM_send_command(&CAM1, CAMERA_REBOOT);
        CAM_send_command(&CAM3, CAMERA_REBOOT);
        CAM_send_command(&CAM2, CAMERA_REBOOT);
        break;

	  //CAM to DEBUG - enable wifimodule for cameras
    case 0x45:
		  CAM_send_command(&CAM1, CAMERA_DEBUG);
		  CAM_send_command(&CAM2, CAMERA_DEBUG);
		  CAM_send_command(&CAM3, CAMERA_DEBUG);
      break;

	  //Reboot MCU - reset self
	  case 0x47:
		  NVIC_SystemReset();
      break;

	  //Start GoPro filming
    case 0x48:
      HAL_GPIO_WritePin (GPIOD, GOPRO_Pin, GPIO_PIN_SET);
	  g_gopro_release_timestamp = HAL_GetTick()+1000;
      break;

	  //Stop GoPro filming
    case 0x49:
		  HAL_GPIO_WritePin (GPIOD, GOPRO_Pin, GPIO_PIN_SET);
		  g_gopro_release_timestamp = HAL_GetTick()+1000;
      break;

	  //Turn on GoPro
	  case 0x4A:
		  HAL_GPIO_WritePin (GPIOD, GOPRO_Pin, GPIO_PIN_SET);
		  g_gopro_release_timestamp = HAL_GetTick()+5000;
      break;

	  //Turn off GoPro
	  case 0x4B:
		  HAL_GPIO_WritePin (GPIOD, GOPRO_Pin, GPIO_PIN_SET);
		  g_gopro_release_timestamp = HAL_GetTick()+5000;
      break;

	  //Erase flight REC
	  case 0x4C:
		  Chip_Erase();
		  break;

	  //Start fligt REC
	  case 0x4D:
		  if(flash.Memory_Full == 0){
			  g_flash_logging_enabled=1;
		  }
		  break;

	  //Stop flight REC
	  case 0x4E:
		  g_flash_logging_enabled=0;
		  break;

	  //Read flight REC
	  case 0x52:
		  Read_Data_Cont(16);
		  break;

		  // No command
	  case 0:
		  break;

    default:
    	print("unknown command 0x%X\r\n", command);
      break;
    }
    command = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	camera_data_t* cam = 0;

	if(huart->Instance == CAM1.huart->Instance)
		cam = &CAM1;
	else if(huart->Instance == CAM2.huart->Instance)
		cam = &CAM2;
	else if(huart->Instance == CAM3.huart->Instance)
		cam = &CAM3;

	if(cam != 0)
	{
		// receive status
		HAL_UART_Receive_IT(cam->huart, cam->status, 2);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//Assigning status register addresses
	flash.SR_1 = 0;
	flash.SR_2 = 0;
	flash.SR_3 = 0;

	memset(&flash.Buffer_0, 0xFF, sizeof(flash.Buffer_0));
	memset(&flash.Buffer_1, 0xFF, sizeof(flash.Buffer_1));

	flash.Buffer_Index = 0;
	flash.Buffer_Select = 0;
	flash.Memory_Full = 0;
	flash.Block_Mem = 0;
	flash.Page_Index = 0;
	flash.ID = 0;
	flash.Buffer_p = flash.Buffer_0;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter


  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_UART8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  FDCAN_init(&hfdcan1);

	CAM_init(&CAM1, &huart2);
	CAM_init(&CAM2, &huart8);
	CAM_init(&CAM3, &huart5);

// TODO: check if its necessary to get camstatus on startup. i think these are retrieved via interrupt anyways (might check later)
  HAL_UART_Receive_IT(CAM1.huart, CAM1.status, 2);
  HAL_UART_Receive_IT(CAM2.huart, CAM2.status, 2);
  HAL_UART_Receive_IT(CAM3.huart, CAM3.status, 2);

  Flash_Init(0);
  flash.ID=Read_ID();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t lasttime_ledblink = HAL_GetTick();
  uint32_t lasttime_statussendt = HAL_GetTick();
  uint32_t lasttime_flash_status_checked = HAL_GetTick();

  while (1)
  {
	  uint32_t now = HAL_GetTick();

	  // Blink led
	  if(lasttime_ledblink + 500 > now)
	  {
		  lasttime_ledblink = now;
		  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
	  }

	  // Check if gopro button has been pressed for long enough
	  if(g_gopro_release_timestamp != 0 && g_gopro_release_timestamp < now){
		  HAL_GPIO_WritePin (GPIOD, GOPRO_Pin, GPIO_PIN_RESET);
		  g_gopro_release_timestamp = 0;
	  }

	  // Read status registers from flash memory
	  if(lasttime_flash_status_checked + 1000 > now)
	  {
		  lasttime_flash_status_checked = now;
		  Read_All_Status_Register();
	  }

	  // Send status
	  if(lasttime_statussendt + 1000 > now)
	  {
		  lasttime_statussendt = now;
		  send_status();
	  }

	  uint8_t command = 0;
	  HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, &command,1, 20);
	  if (status == HAL_OK) {
	      // Data received successfully
	      // 'command' contains the received byte
		  print("Received: 0x%02X\r\n", command);
	  }


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// For printf function to work
int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    hstatus = HAL_UART_Transmit(&huart3,(uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {
  (void)len;
  HAL_StatusTypeDef hstatus;

  if (fd == STDIN_FILENO) {
    hstatus = HAL_UART_Receive(&huart3, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return 1;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  // HAL_FDCAN_Start(&hfdcan1);
  __disable_irq();
  printf("Error_Handler!\r\n");
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
  printf("Assertion failed: file %s on line %lu\r\n", file, line);
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
