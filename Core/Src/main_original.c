///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
//  * All rights reserved.</center></h2>
//  *
//  * This software component is licensed by ST under BSD 3-Clause license,
//  * the "License"; You may not use this file except in compliance with the
//  * License. You may obtain a copy of the License at:
//  *                        opensource.org/licenses/BSD-3-Clause
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "stdarg.h"
//#include "lfs_util.h"
//#include "lfs.h"
//#include "MT25Q.h"
//#include "nor.h"
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
////#define CDC_USB_DEBUG
//#define UART_DEBUG
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//#define sector_size 32
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//void myprintf(const char *fmt, ...);
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//RTC_HandleTypeDef hrtc;
//
//SPI_HandleTypeDef hspi1;
//SPI_HandleTypeDef hspi2;
//
//UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart2;
//
//PCD_HandleTypeDef hpcd_USB_FS;
//
///* USER CODE BEGIN PV */
////uint8_t tx[]={'S','A','N','G','A','M'};
//
//uint16_t address = 0x00;
//uint8_t status_reg=0;
//uint8_t READ_FLAG=0;
//int tx[70];
//
//DEVICE_ID dev_id;
//
//uint8_t DEBUG_DATA_RX_FLAG = 0;
//// variables used by the filesystem
//typedef struct{
//	uint32_t secCount;
//	uint32_t bootCount;
//}app_count_t;
//	lfs_file_t File;
//		char Text[20];
//		app_count_t Counter = {0};
//		lfs_t Lfs,Lfs2;
//		nor_t Nor;
//
//		static struct lfs_config LfsConfig,LfsConfig2 = {0};
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_SPI1_Init(void);
//static void MX_SPI2_Init(void);
//static void MX_USART1_UART_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_USB_PCD_Init(void);
//static void MX_RTC_Init(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//// Function to read data from a file in LittleFS
//void read_file_from_littlefs(lfs_t *lfs, const char *filename) {
//    lfs_file_t file;
//    HAL_UART_Transmit(&huart2, filename,sizeof(filename),1000);
//    // Open the file for reading
//    int err = lfs_file_open(lfs, &file, filename, LFS_O_RDONLY);
//    if (err < 0) {
//        printf("Failed to open file: %s\n", filename);
//        return;
//    }
//
//    // Get the file size
//    lfs_soff_t file_size = lfs_file_size(lfs, &file);
//    if (file_size < 0) {
//        printf("Failed to get file size for: %s\n", filename);
//        lfs_file_close(lfs, &file);
//        return;
//    }
//
//    // Allocate a buffer to hold the file data
//    float *buffer = malloc(file_size);
//    if (buffer == NULL) {
//        printf("Failed to allocate buffer for reading file: %s\n", filename);
//        lfs_file_close(lfs, &file);
//        return;
//    }
//
//    // Read the file content into the buffer
//    lfs_ssize_t bytes_read = lfs_file_read(lfs, &file, buffer, file_size);
//    if (bytes_read < 0) {
//        printf("Failed to read file: %s\n", filename);
//    } else {
//    	char x;
//        // Successfully read the file, print its content (if it's text data)
//        HAL_UART_Transmit(&huart2, buffer, (int)bytes_read,1000);
//        for(int i=0;i<(int) bytes_read;){
////        	printf(buffer[i]);
//         	x=buffer[i];
//        	i++;
//        }
//
//
//        printf("File Content (%s):\n%.*s\n", filename, (int)bytes_read, buffer);
//    }
//
//    // Clean up
//    free(buffer);
//    lfs_file_close(lfs, &file);
//}
//
//volatile uint8_t DmaEnd = 0;
//
//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
//	DmaEnd = 1;
//}
//
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
//	DmaEnd = 1;
//}
//
//void nor_delay_us(uint32_t us){
////	if (us >= __HAL_TIM_GET_AUTORELOAD(&htim2)){
////		us = __HAL_TIM_GET_AUTORELOAD(&htim2) - 1;
////	}
////	__HAL_TIM_SET_COUNTER(&htim2, 0);
////	HAL_TIM_Base_Start(&htim2);
////	while (__HAL_TIM_GET_COUNTER(&htim2) < us);
////	HAL_TIM_Base_Stop(&htim2);
////	HAL_Delay(1000);
//}
//
//void nor_cs_assert(){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//}
//
//void nor_cs_deassert(){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//}
//
//void nor_spi_tx(uint8_t *pData, uint32_t Size){
////	HAL_SPI_Transmit(&hspi3, pData, Size, 100);
//	DmaEnd = 0;
//	HAL_SPI_Transmit(&hspi2, pData, Size, 1000);
////	while (DmaEnd == 0);
//}
//
//void nor_spi_rx(uint8_t *pData, uint32_t Size){
////	HAL_SPI_Receive(&hspi3, pData, Size, 100);
//	DmaEnd = 0;
//	HAL_SPI_Receive(&hspi2, pData, Size, 1000);
////	DmaEnd =0;
////	while (DmaEnd == 0);
//}
//
//void __init_nor(){
//	Nor.config.CsAssert = nor_cs_assert;
//	Nor.config.CsDeassert = nor_cs_deassert;
//	Nor.config.DelayUs = nor_delay_us;
//	Nor.config.SpiRxFxn = nor_spi_rx;
//	Nor.config.SpiTxFxn = nor_spi_tx;
//
//	if (NOR_Init(&Nor) != NOR_OK){ //NOR_Init
//		Error_Handler();
//	}
//}
//
///** Start LittleFs **/
//
//int _fs_read(const struct lfs_config *c, lfs_block_t block,
//            lfs_off_t off, void *buffer, lfs_size_t size){
//
//	if (NOR_ReadSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
//		return 0;
//	}
//
//	return LFS_ERR_IO;
//}
//
//int _fs_write(const struct lfs_config *c, lfs_block_t block,
//        lfs_off_t off, const void *buffer, lfs_size_t size){
//
//	if (NOR_WriteSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
//		return 0;
//	}
//
//	return LFS_ERR_IO;
//}
//
//int _fs_erase(const struct lfs_config *c, lfs_block_t block){
//	if (NOR_EraseSector(&Nor, block) == NOR_OK){
//		return 0;
//	}
//
//	return LFS_ERR_IO;
//}
//
//int _fs_sync(const struct lfs_config *c){
//	return 0;
//}
//
//// Function to list all files and directories in the filesystem
//void list_files(lfs_t *lfs) {
//    lfs_dir_t dir;
//    struct lfs_info info;
//
//    // Open the root directory
//    int err = lfs_dir_open(lfs, &dir, "/");
//    if (err) {
//        printf("Failed to open directory\n");
//        return;
//    }
//
//    // Loop through all files in the directory
//    while (true) {
//        err = lfs_dir_read(lfs, &dir, &info);
//        if (err < 0) {
//            printf("Failed to read directory\n");
//            break;
//        }
//
//        // If no more files, break
//        if (err == 0) {
//            break;
//        }
//        uint8_t dir[100];
//        // Print the type and name of the file
//        if (info.type == LFS_TYPE_REG) {
//            sprintf(dir,"File: %s\n\0", info.name);
//            HAL_UART_Transmit(&huart2, dir, strlen(dir),1000);
//        } else if (info.type == LFS_TYPE_DIR) {
//        	sprintf(dir,"Directory: %s\n\0", info.name);
//
//            HAL_UART_Transmit(&huart2, dir, strlen(dir),1000);
//        }
//    }
//
//    // Close the directory
//    lfs_dir_close(lfs, &dir);
//}
//
//void __init_littefs(){
//	// because of static qualifier, this variable
//	// will have a dedicated address
//		int Error;
//
//		LfsConfig.read_size = 256;
//		LfsConfig.prog_size = 256;
//		LfsConfig.block_size = Nor.info.u16SectorSize;
//		LfsConfig.block_count =  16384;//Nor.info.u32SectorCount;
//		LfsConfig.cache_size = Nor.info.u16PageSize;
//		LfsConfig.lookahead_size = 256;//Nor.info.u32SectorCount/8;
//		LfsConfig.block_cycles = 100;
//
//		LfsConfig.read = _fs_read;
//		LfsConfig.prog = _fs_write;
//		LfsConfig.erase = _fs_erase;
//		LfsConfig.sync = _fs_sync;
//
//		Error = lfs_mount(&Lfs, &LfsConfig);
//		if (Error != LFS_ERR_OK){
//			lfs_format(&Lfs, &LfsConfig);
//			Error = lfs_mount(&Lfs, &LfsConfig);
//			if (Error != LFS_ERR_OK){
//				Error_Handler();
//			}
//		}
//
//
//		LfsConfig2.read_size = 256;
//		LfsConfig2.prog_size = 256;
//		LfsConfig2.block_size = Nor.info.u16SectorSize;
//		LfsConfig2.block_count =  16384;//Nor.info.u32SectorCount;
//		LfsConfig2.cache_size = Nor.info.u16PageSize;//1024
//		LfsConfig2.lookahead_size = 256;//Nor.info.u32SectorCount/8;
//		LfsConfig2.block_cycles = 100;
//
//		LfsConfig2.read = _fs_read;
//		LfsConfig2.prog = _fs_write;
//		LfsConfig2.erase = _fs_erase;
//		LfsConfig2.sync = _fs_sync;
//
//		LfsConfig2.context = (void*) (8192 * Nor.info.u16SectorSize);
//		Error = lfs_mount(&Lfs2, &LfsConfig2);
//				if (Error != LFS_ERR_OK){
//					lfs_format(&Lfs2, &LfsConfig2);
//					Error = lfs_mount(&Lfs2, &LfsConfig2);
//					if (Error != LFS_ERR_OK){
//						Error_Handler();
//					}
//				}
//
//}
//
//
//#define PATH_MAX_LEN 256
//
//// Recursive function to list files and directories with full paths
//void list_files_with_size(lfs_t *lfs, const char *path) {
//    lfs_dir_t dir;
//    struct lfs_info info;
//
//    // Open the directory at the given path
//    int err = lfs_dir_open(lfs, &dir, path);
//    if (err) {
//        printf("Failed to open directory: %s\n", path);
//        return;
//    }
//
//    // Loop through all files in the directory
//    while (true) {
//        err = lfs_dir_read(lfs, &dir, &info);
//        if (err < 0) {
//            printf("Failed to read directory: %s\n", path);
//            break;
//        }
//
//        // If no more files, break
//        if (err == 0) {
//            break;
//        }
//
//        // Build the full path for the current file/directory
//        char full_path[PATH_MAX_LEN];
//        snprintf(full_path, sizeof(full_path), "%s/%s", path, info.name);
//        char pa[500];
//        // Check if the entry is a file or directory
//        if (info.type == LFS_TYPE_REG) {
//            sprintf(pa,"File: %s, Size: %ld bytes\n", full_path, info.size);
//            HAL_UART_Transmit(&huart2, pa, strlen(pa),1000);
//        } else if (info.type == LFS_TYPE_DIR && strcmp(info.name, ".") != 0 && strcmp(info.name, "..") != 0) {
//            sprintf(pa,"Directory: %s\n", full_path);
//            HAL_UART_Transmit(&huart2, pa, strlen(pa),1000);
//            // Recursively list the contents of the directory
//            list_files_with_size(lfs, full_path);
//        }
//    }
//
//    // Close the directory
//    lfs_dir_close(lfs, &dir);
//}
//void __init_storage(){
//	__init_nor();
//	__init_littefs();
//}
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_SPI1_Init();
//  MX_SPI2_Init();
//  MX_USART1_UART_Init();
//  MX_USART2_UART_Init();
//  MX_USB_PCD_Init();
//  MX_RTC_Init();
//  /* USER CODE BEGIN 2 */
////simple
////  //First erase flash memory
////  	Sector_Erase_4B(&hspi3, address, sector_size);
////	//id read
//  while(1)
//  {
//	Read_ID(&hspi2, &dev_id);
//	if(dev_id.MAN_ID == 32){
//		break;
//	}
//  }
////
////	HAL_Delay(100);
//
//	//status_reg = Status_Reg(&hspi3);
//	//  int add = 0;
//	//  for(int i=0;i<300;i++){
//	//	  Sector_Erase_4B(&hspi3, add, 64);
//	//	  add+=65536;
//	//  }
//
//	  // myprintf("Starting LittleFS application........\n");
//    HAL_Delay(100);
//
//  HAL_UART_Transmit(&huart2,"EPDM is starting *********\n", sizeof("EPDM is starting *********\n"),1000);
//
//  HAL_UART_Transmit(&huart2,"Chip erase starting....\n", sizeof("Chip erase starting....\n"),1000);
////  Chip_Erase(&hspi3);
//
//  HAL_UART_Transmit(&huart2,"Chip erase ending....\n", sizeof("Chip erase ending....\n"),1000);
//  HAL_UART_Transmit(&huart2,"Chip erase ending....\n", sizeof("Chip erase ending....\n"),1000);
//
//
//int j=0;
//while(1){
//	j++;
//		  __init_storage();
//		  list_files(&Lfs);
//		  list_files(&Lfs2);
//		//  char path[200];
//		  char txt[40];//="\nprem is writing it manually";
////		  txt[0] =j;
//		  sprintf(txt,"prem is writing it manually %d\n.",j);
//		__init_littefs();
//		  list_files_with_size(&Lfs, "/");
//
//		  list_files_with_size(&Lfs2, "/");
////		  		  lfs_file_open(&Lfs, &File, "common1.txt", LFS_O_CREAT | LFS_O_RDWR  |LFS_O_APPEND | LFS_O_CREAT );
////		  		  lfs_file_write(&Lfs, &File, &txt, sizeof(txt));
////		  		lfs_file_write(&Lfs, &File, j, sizeof(j));
////		  		  lfs_file_close(&Lfs, &File);
//		  read_file_from_littlefs(&Lfs2, "common1.txt");
//		//  read_file_from_littlefs(&Lfs, "sat_health.txt");
////		  read_file_from_littlefs(&Lfs, "common.txt");
//		  lfs_unmount(&Lfs2);
//		  lfs_unmount(&Lfs);
//		  HAL_Delay(10000);
//	}
////  read_file_from_littlefs(&Lfs, "epdm.txt");
////   lfs_file_open(&Lfs, &File, "/sat_health.txt", LFS_O_RDWR );
//////   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
////
////   lfs_file_read(&Lfs, &File, &tx, sizeof(tx));
////   HAL_UART_Transmit(&huart2,tx,strlen(tx),1000);
////   lfs_file_close(&Lfs, &File);
////
////   lfs_file_open(&Lfs, &File, "flags.txt", LFS_O_RDWR );
////  //   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
////
////     lfs_file_read(&Lfs, &File, &tx, sizeof(tx));
////     HAL_UART_Transmit(&huart2,tx,strlen(tx),1000);
////     lfs_file_close(&Lfs, &File);
////
////     lfs_file_open(&Lfs, &File, "epdm.txt", LFS_O_RDWR );
////    //   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
////
////       lfs_file_read(&Lfs, &File, &tx, sizeof(tx));
////       HAL_UART_Transmit(&huart2,tx,strlen(tx),1000);
////       lfs_file_close(&Lfs, &File);
//   Counter.bootCount += 1;
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  { sprintf(Text, "Bt %lu |Ct %lu\n", Counter.bootCount, Counter.secCount);
////	  HAL_UART_Transmit(&huart2,Text, sizeof(Text),1000);
////
////	  HAL_UART_Transmit(&huart2,"*******\n", sizeof("*******\n"),1000);
//
////		  lfs_file_open(&Lfs, &File, "count.txt", LFS_O_RDWR | LFS_O_CREAT |LFS_O_APPEND);
////		  lfs_file_write(&Lfs, &File, &Counter.secCount, 32);
////		  lfs_file_close(&Lfs, &File);
//
////		  while ((HAL_GetTick() - HalTickAux) < 1000);
////		  HAL_Delay(1000);
//
//		  Counter.secCount += 1;
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//
//
//
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
//  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
//  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief RTC Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_RTC_Init(void)
//{
//
//  /* USER CODE BEGIN RTC_Init 0 */
//
//  /* USER CODE END RTC_Init 0 */
//
//  /* USER CODE BEGIN RTC_Init 1 */
//
//  /* USER CODE END RTC_Init 1 */
//  /** Initialize RTC Only
//  */
//  hrtc.Instance = RTC;
//  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
//  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
//  if (HAL_RTC_Init(&hrtc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RTC_Init 2 */
//
//  /* USER CODE END RTC_Init 2 */
//
//}
//
///**
//  * @brief SPI1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI1_Init(void)
//{
//
//  /* USER CODE BEGIN SPI1_Init 0 */
//
//  /* USER CODE END SPI1_Init 0 */
//
//  /* USER CODE BEGIN SPI1_Init 1 */
//
//  /* USER CODE END SPI1_Init 1 */
//  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI1_Init 2 */
//
//  /* USER CODE END SPI1_Init 2 */
//
//}
//
///**
//  * @brief SPI2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI2_Init(void)
//{
//
//  /* USER CODE BEGIN SPI2_Init 0 */
//
//  /* USER CODE END SPI2_Init 0 */
//
//  /* USER CODE BEGIN SPI2_Init 1 */
//
//  /* USER CODE END SPI2_Init 1 */
//  /* SPI2 parameter configuration*/
//  hspi2.Instance = SPI2;
//  hspi2.Init.Mode = SPI_MODE_MASTER;
//  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi2.Init.NSS = SPI_NSS_SOFT;
//  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
//  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi2.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI2_Init 2 */
//
//  /* USER CODE END SPI2_Init 2 */
//
//}
//
///**
//  * @brief USART1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}
//
///**
//  * @brief USB Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USB_PCD_Init(void)
//{
//
//  /* USER CODE BEGIN USB_Init 0 */
//
//  /* USER CODE END USB_Init 0 */
//
//  /* USER CODE BEGIN USB_Init 1 */
//
//  /* USER CODE END USB_Init 1 */
//  hpcd_USB_FS.Instance = USB;
//  hpcd_USB_FS.Init.dev_endpoints = 8;
//  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
//  hpcd_USB_FS.Init.low_power_enable = DISABLE;
//  hpcd_USB_FS.Init.lpm_enable = DISABLE;
//  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
//  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USB_Init 2 */
//
//  /* USER CODE END USB_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, SS1_Pin|SS2_Pin|SS3_Pin|SS4_Pin
//                          |CS_FM_Pin|DRDY4_Pin|DRDY3_Pin|DRDY2_Pin
//                          |DRDY1_Pin|MSN_EN3_Pin|MSN_EN1_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, MSN_EN4_Pin|MSN_EN2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pins : SS1_Pin SS2_Pin SS3_Pin SS4_Pin
//                           CS_FM_Pin DRDY4_Pin DRDY3_Pin DRDY2_Pin
//                           DRDY1_Pin MSN_EN3_Pin MSN_EN1_Pin */
//  GPIO_InitStruct.Pin = SS1_Pin|SS2_Pin|SS3_Pin|SS4_Pin
//                          |CS_FM_Pin|DRDY4_Pin|DRDY3_Pin|DRDY2_Pin
//                          |DRDY1_Pin|MSN_EN3_Pin|MSN_EN1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : MSN_EN4_Pin MSN_EN2_Pin */
//  GPIO_InitStruct.Pin = MSN_EN4_Pin|MSN_EN2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
//void myprintf(const char *fmt, ...) {
//    va_list args;
//    va_start(args, fmt);
//    char buffer[100];
//    vsnprintf(buffer, sizeof(buffer), fmt, args);
//    HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
//    va_end(args);
//}
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
