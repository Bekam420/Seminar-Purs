/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Definiranje portova redova (ROWS)
#define ROWS_GPIO_PORT_A GPIOA  // GPIO port A za redove
#define ROWS_GPIO_PORT_B GPIOB  // GPIO port B za redove
#define ROWS_GPIO_PORT_C GPIOC  // GPIO port C za redove

// Definiranje portova stupaca (COLS)
#define COLS_GPIO_PORT_A GPIOA  // GPIO port A za stupce
#define COLS_GPIO_PORT_B GPIOB  // GPIO port B za stupce

// Definiranje pinova redova (ROWS)
#define ROW1_PIN GPIO_PIN_6  // Red 1, povezan na Port A, Pin 6
#define ROW2_PIN GPIO_PIN_7  // Red 2, povezan na Port A, Pin 7
#define ROW3_PIN GPIO_PIN_6  // Red 3, povezan na Port B, Pin 6
#define ROW4_PIN GPIO_PIN_7  // Red 4, povezan na Port C, Pin 7

// Definiranje pinova stupaca (COLS)
#define COL1_PIN GPIO_PIN_8   // Stupac 1, povezan na Port A, Pin 8
#define COL2_PIN GPIO_PIN_5   // Stupac 2, povezan na Port B, Pin 5
#define COL3_PIN GPIO_PIN_4   // Stupac 3, povezan na Port B, Pin 4
#define COL4_PIN GPIO_PIN_10  // Stupac 4, povezan na Port B, Pin 10

// Definiranje pina za LED
#define LED_PIN GPIO_PIN_5 // Pin A5

// Definiranje pina za relejni modul
#define GPIO_PIN_RELAY GPIO_PIN_0 // Pin 0
#define GPIO_PORT_RELAY GPIOC // Port C

// Definiranje duljine lozinke
#define PASSWORD_LENGTH 4

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char Keypad_Scan(void)
{
  char key = '\0';

  // Podesi svaki stupac na nisku razinu, jedan po jedan, i provjeri dali su redovi na visokoj razini (pritisnutu tipku).

 for (int col = 0; col < 4; col++)
 {
   // Postavi sve stupce na visoku razinu (osim trenutnog stupca)
   HAL_GPIO_WritePin(COLS_GPIO_PORT_A, COL1_PIN, GPIO_PIN_SET);
   HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL2_PIN | COL3_PIN | COL4_PIN, GPIO_PIN_SET);

   // Postavi trenutni stupac na nisku razinu
    switch (col)
    {
    case 0:
      HAL_GPIO_WritePin(COLS_GPIO_PORT_A, COL1_PIN, GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL2_PIN, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL3_PIN, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL4_PIN, GPIO_PIN_RESET);
      break;
    }

    // Provjeri redove za visoku razinu (pritisnuta tipka)
    if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_A, ROW1_PIN) == GPIO_PIN_RESET)
    {
      key = (col == 0) ? '1' : (col == 1) ? '2' : (col == 2) ? '3' : 'A';
      break;
    }
    else if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_A, ROW2_PIN) == GPIO_PIN_RESET)
    {
      key = (col == 0) ? '4' : (col == 1) ? '5' : (col == 2) ? '6' : 'B';
      break;
    }
    else if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_B, ROW3_PIN) == GPIO_PIN_RESET)
    {
      key = (col == 0) ? '7' : (col == 1) ? '8' : (col == 2) ? '9' : 'C';
      break;
    }
    else if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_C, ROW4_PIN) == GPIO_PIN_RESET)
    {
      key = (col == 0) ? '*' : (col == 1) ? '0' : (col == 2) ? '#' : 'D';
      break;
    }
  }

  return key;
}
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

  static uint8_t index = 0;
  char stored_password[PASSWORD_LENGTH + 1] = {0}; // +1 za null znak
  char entered_password[PASSWORD_LENGTH + 1] = {0}; // +1 za null znak
  uint8_t password_set = 0; // 0 ako lozinka nije postavljena, 1 ako je postavljena

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   char key = Keypad_Scan();
   if (key != '\0' && key == '#')
   {
     if (index == PASSWORD_LENGTH) // Provjera dali je u indeksu koliko je zaporuka duga
     {
       entered_password[index] = '\0'; // Završavanje null znakom
       if (password_set == 0)
       {
         // Postavljanje lozinke
         strcpy(stored_password, entered_password);
         password_set = 1;
         HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_SET); // Upali LED
         HAL_Delay(1000); // Pričekaj 1 sekunde
         HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_RESET); // Ugasi LED
       }
       else
       {
         // Provjera lozinke
         if (strcmp(stored_password, entered_password) == 0)
         {
           HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_SET); // Upali LED
           HAL_GPIO_WritePin(GPIO_PORT_RELAY, GPIO_PIN_RELAY, GPIO_PIN_SET); // Upali relej
           HAL_Delay(3000); // Pričekaj 3 sekunde
           HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_RESET); // Ugasi LED
           HAL_GPIO_WritePin(GPIO_PORT_RELAY, GPIO_PIN_RELAY, GPIO_PIN_RESET); // Ugasi relej
         }
       }
       // Resetiranje unesene lozinke i indeksa
       memset(entered_password, 0, sizeof(entered_password));
       index = 0;
     }
   }
   else if (key == 'C') // C kao master switch
   {
     // Brisanje pohranjene lozinke
     memset(stored_password, 0, sizeof(stored_password));
     password_set = 0;
     // Resetiranje unesene lozinke i indeksa
     memset(entered_password, 0, sizeof(entered_password));
     index = 0;
     // Gašenje LED-ova i releja kao indikacija resetiranja lozinke
     HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIO_PORT_RELAY, GPIO_PIN_RELAY, GPIO_PIN_RESET);
   }
   else if (index < PASSWORD_LENGTH && key >= '0' && key <= '9')
   {
     entered_password[index++] = key;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
