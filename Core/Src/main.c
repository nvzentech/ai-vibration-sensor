/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "app_lorawan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NanoEdgeAI.h"
#include "lis3dh_driver.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "lora_app.h"
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
//#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "sys_conf.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "lis3dh_driver.h"   																			// Accelerometer(LIS3DHT) driver.h inclusion
//#include "hdc3022_driver.h"																		 	   	// Humidity Sensor (HDC3022) driver.h inclusion
#include "lora_comm.h"
#include "gps_parser.h"
#include "globals.h"
#include "flash_if.h"
#include "sht40.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern UART_HandleTypeDef huart1;                          // UART for debugging
//extern LoRaWAN_HandleTypeDef hLoRaWAN ;																// LoRaWAN handler

#define TEMP_BUFFER_SIZE 1
char temp_buffer[TEMP_BUFFER_SIZE];

bool timer_flag = false;
bool skip_set_flag = true;
bool state_change_flag = false;

extern I2C_HandleTypeDef hi2c2;

#define FIFO_WINDOW_SIZE  100   // Statistical smoothing window (100 samples)
#define HOP_SIZE          20    // Trigger inference every 20 samples (0.2s update at 100 Hz)

/* USER CODE BEGIN PFP */
uint8_t Process_Sample_Rolling_FIFO(float x, float y, float z,
		int *out_class_id);
/* USER CODE END PFP */

// Create a variable to hold the sensor readings
SHT40_Data_t sht40_data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Emergency Button interrupt callback -- START ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)								//
{																			//
	//
}																			//
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Emergency button interrupt callback -- END   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float input_signal[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];
float probabilities[NEAI_NUMBER_OF_CLASSES];
int id_class = 0;

const char *class_names[4] = { "LOW_VIBRATION",  // ID 0
		"IDLE",            // ID 1
		"HIGH_VIBRATION",    // ID 2
		"ABNORMAL_VIBRATION" //ID 3
		};

/* --- Real-Time State Tracking --- */
uint16_t current_vibration_state = 1;     // Current confirmed state (e.g. IDLE)
uint16_t previous_vibration_state = 1;      // Last state transmitted
uint8_t state_debounce_count = 0;      // Filter counter to prevent false alarms
#define STATE_DEBOUNCE_THRESHOLD  2         // Must see the new state 2 times in a row

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ UART PRINTF FUNCTION START ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

GETCHAR_PROTOTYPE {
	uint8_t ch = 0;
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	HAL_UART_Receive(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  UART PRINTF FUNCTION END  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (!skip_set_flag) {
		if (htim->Instance == TIM2)   // Check if interrupt is from TIM2
		{
			timer_flag = true;  // set the flag every 1 minute
		}
	} else {
		skip_set_flag = false;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	setvbuf(stdin, NULL, _IONBF, 0);					// UART PRINTF FUNCTION
	//char payload[512];										// Payload
	//float x_accel , y_accel , z_accel ;
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

	printf("   STM32 LoRaWAN Tracker Starting...\r\n");

	/* Erase LoRaWAN NVM to clear stale DevNonce (fixes "DevNonce already used" error) */
	printf("Erasing LoRaWAN NVM to reset DevNonce...\r\n");
	FLASH_IF_Erase((void*) 0x0803F000UL, FLASH_PAGE_SIZE);
	printf("NVM erased. Fresh OTAA join will start.\r\n");

	printf("[LoRa] Initializing LoRaWAN Stack...\r\n");
	MX_LoRaWAN_Init();
	printf("[LoRa] Stack initialized successfully.\r\n");

	MX_I2C2_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);		// PMIC - PA0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	HAL_UART_Receive_IT(&huart2, (uint8_t*) temp_buffer, 1);// Start the interrupt  ----> INTERRUPT ENABLE
	// Enable accelerometer functions
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	/* 1. Declare the variables ONCE */
	int8_t txPower;       // Note: int8_t fixes the signedness warning!
	bool ADR = false;
	LmHandlerErrorStatus_t errorStats;

	/* 2. Assign values to them WITHOUT the type name in front */
	errorStats = LmHandlerGetAdrEnable(&ADR);
	printf("1st GET ADR is %s\r\n", ADR ? "Enabled" : "Disabled");

	errorStats = LmHandlerGetTxPower(&txPower);
	printf("TX Power : %d \r\n", txPower);

	/* 3. Tell the compiler we know we aren't checking the error status to clear the warning */
	(void) errorStats;

	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	//Accelerometer Initialization
	if (LIS3DH_Init(&hi2c2) != HAL_OK) {
		printf("LIS3DH Init Failed! Check I2C address & wiring.\r\n");
	} else {
		printf("LIS3DH Initialized.\r\n");
	}

	/* USER CODE BEGIN 2 */

	enum neai_state neai_status = neai_classification_init();
	if (neai_status != NEAI_OK) {
		printf("NanoEdge AI init failed: %d\r\n", neai_status);
	} else {
		printf("NanoEdge AI initialized successfully.\r\n");
	}

	/* Print model configurations once during startup */
	printf("NEAI Buffer Length: %u samples | Axes: %u\r\n",
	NEAI_INPUT_SIGNAL_LENGTH, NEAI_INPUT_AXIS_NUMBER);
	/* USER CODE END 2 */

	while (1) {
		/* USER CODE END WHILE */
		MX_LoRaWAN_Process();
		const char *tx_reason = "";
		/* When timer fires: read sensor, then send payload */

		if (LIS3DH_ReadWindow(&hi2c2, input_signal, NEAI_INPUT_SIGNAL_LENGTH)
				== HAL_OK) {
			neai_classification(input_signal, probabilities, &id_class);

			// Check if confidence is high AND the detected state is different from current state
			if ((probabilities[id_class] >= 0.99f)
					&& (id_class != current_vibration_state)) {
				printf(
						"\r\n[EVENT] State Changed: %s -> %s (ID: %d) | Conf: %.1f%%\r\n",
						class_names[current_vibration_state],
						class_names[id_class], id_class,
						probabilities[id_class] * 100.0f);

				// Update current vibration state
				current_vibration_state = id_class;

				// Trigger the transmission block
				state_change_flag = true;
				tx_reason = "STATE_CHANGE";

				// Reset hardware timer counter so next periodic heartbeat happens 1 full minute from now
				__HAL_TIM_SET_COUNTER(&htim2, 0);
			}
		}

		if (timer_flag || state_change_flag) {
			/* Read fresh temperature & humidity from SHT40 */

			if (SHT40_Read(&hi2c2, &sht40_data) == HAL_OK) {
				printf("Temperature: %.2f C | Humidity: %.2f %%\r\n",
						sht40_data.temperature, sht40_data.humidity);
			} else {
				printf("SHT40 read failed!\r\n");
			}

			printf(
					"[LoRa TX] Reason: %s | State: %s (ID: %d) | Conf: %.1f%%\r\n",
					tx_reason, class_names[current_vibration_state],
					current_vibration_state,
					probabilities[current_vibration_state] * 100.0f);

			// Send payload via LoRa
			sendLoRaPayload();
			if (timer_flag) {
				timer_flag = false; // Clear timer interrupt flag
			}
			if (state_change_flag) {
				state_change_flag = false;
			}
			/* USER CODE END 3 */

		}
	}
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
