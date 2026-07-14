/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"
#include "mpu6050.h"
#include "i2c_manager.h"
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

/* Definitions for BME280_Task */
osThreadId_t BME280_TaskHandle;
const osThreadAttr_t BME280_Task_attributes = {
  .name = "BME280_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MPU6050_Task */
osThreadId_t MPU6050_TaskHandle;
const osThreadAttr_t MPU6050_Task_attributes = {
  .name = "MPU6050_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2c_mutex */
osMutexId_t i2c_mutexHandle;
const osMutexAttr_t i2c_mutex_attributes = {
  .name = "i2c_mutex"
};
/* USER CODE BEGIN PV */
bme280_handle_t my_sensor = NULL;
mpu6050_handle_t my_mpu6050 = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void Start_BME280_Task(void *argument);
void Start_MPU6050_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c_mutex */
  i2c_mutexHandle = osMutexNew(&i2c_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BME280_Task */
  BME280_TaskHandle = osThreadNew(Start_BME280_Task, NULL, &BME280_Task_attributes);

  /* creation of MPU6050_Task */
  MPU6050_TaskHandle = osThreadNew(Start_MPU6050_Task, NULL, &MPU6050_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */
  LL_I2C_Enable(I2C1);
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_BME280_Task */
/**
  * @brief  Function implementing the BME280_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_BME280_Task */
void Start_BME280_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	bme280_user_configs my_sensor_config = {0};
	my_sensor_config.i2c_handle = I2C1;
	my_sensor_config.i2c_addr = bme280_i2c_addr0;
	my_sensor_config.ctrlhum_t.bits.osrs_h = 0b011;
	my_sensor_config.ctrlmeas_t.bits.osrs_t = 0b011;
	my_sensor_config.ctrlmeas_t.bits.osrs_p = 0b011;
	my_sensor_config.ctrlmeas_t.bits.mode = 0b11;
	my_sensor_config.config_t.bits.filter = 0b000;
	my_sensor_config.config_t.bits.t_sb = 0b011;

	if(i2c_manager_check_device(I2C1, bme280_i2c_addr0) == I2C_MANAGER_OK){
		my_sensor = bme280_init(&my_sensor_config);
	}

  /* Infinite loop */
  for(;;)
  {
	  if(my_sensor != NULL){
		  if(bme280_read_data_poll(my_sensor) == _bme280_ok){
			  bme280_get_value(my_sensor);
		  }
	  }
	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_MPU6050_Task */
/**
* @brief Function implementing the MPU6050_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MPU6050_Task */
void Start_MPU6050_Task(void *argument)
{
  /* USER CODE BEGIN Start_MPU6050_Task */

	// 1. YAPILANDIRMA (Yeni struct isimleriyle ve gereksiz kalabalıklardan arındırılmış)
	mpu6050_user_config_t my_mpu6050_config = {0};
	my_mpu6050_config.i2c_handle = I2C1;
	my_mpu6050_config.i2c_addr = 0xD0; // Adresi artık doğrudan config içine veriyoruz
	my_mpu6050_config.sample_rate = 255;

	my_mpu6050_config.reg_config.bits.dlpf_cfg = 4;
	my_mpu6050_config.reg_config.bits.ext_sync_set = 0;

	my_mpu6050_config.reg_accel_config.bits.fs_sel = 0;
	my_mpu6050_config.reg_gyro_config.bits.fs_sel = 0;

	my_mpu6050_config.reg_pwr_mgmt_1.bits.clksel = 1;
	my_mpu6050_config.reg_pwr_mgmt_1.bits.cycle = 0;
	my_mpu6050_config.reg_pwr_mgmt_1.bits.device_reset = 0;
	my_mpu6050_config.reg_pwr_mgmt_1.bits.sleep = 0;
	my_mpu6050_config.reg_pwr_mgmt_1.bits.temp_dis = 0;

	// 2. KONTROL VE BAŞLATMA
	// Kapı zili fonksiyonumuzu çağırıyoruz
	if(mpu6050_find_or_check_device(&my_mpu6050_config) == _mpu6050_ok){
		my_mpu6050 = mpu6050_init(&my_mpu6050_config); // Artık sadece config alıyor
		if(my_mpu6050 != NULL){
			mpu6050_configurate(my_mpu6050, &my_mpu6050_config);
		}
	}

  /* Infinite loop */
  for(;;)
  {
	  if(my_mpu6050 != NULL){
		  // İsimler yeni kütüphanedeki fonksiyonlarla eşleştirildi
		  if(mpu6050_read_data_poll(my_mpu6050) == _mpu6050_ok){
			  mpu6050_get_values(my_mpu6050);
			  mpu6050_get_values_ing_ins(my_mpu6050, my_mpu6050_config);
		  }
	  }

	  osDelay(10); // 100 Hz okuma hızı (Zırhın dengesi için hızlı tepkime)
  }
  /* USER CODE END Start_MPU6050_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
