/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Smallest unit length is a dit .
 * Dit length set to 15ms by default, suitable for audio but blinking.
 * Dah (-) length is 3 times that of a dit
 * Space between characters is 3 times that of a dit
 * Space between words is 7 times that of a dit
 */
uint8_t DIT = 15;
#define DAH DIT * 3
#define SPC_C HAL_Delay(DIT * 3)
#define SPC_W HAL_Delay(DIT * 7)

/*
 * Dit length can be increased or decreased in intervals of DIT_INC and DIT_DEC milliseconds.
 */
#define DIT_MIN    0
#define DIT_MAX  255
#define DIT_INC    5
#define DIT_DEC    5


/*
 * First encoding scheme of characters works for any character with 5 or less dits/dahs.
 * Characters are stored as 8-bit values as follows:
 * Bits 7-5: Length of character (i.e. number of dits and dahs)
 * Bits 4-0: Left-to-right - 0 for dit, 1 for dah
 *           If any bits remain they are set to 0
 *
 * Example: R in Morse code is .-. (dit dah dit)
 *          Length is 3, so bits 7-5 are 011
 *          Next 3 bits are 010 (dit dah dit)
 *          Remaining bits are 0
 *
 *                        LEN CODE
 *          R  =  .-.  =  011 01000  =  01101000  =  0x68
 *                          3 .-.
 *
 *          CODE will not be read past LEN characters to avoid interpreting remaining 0s as dits.
 */
#define MRS_A      0x48
#define MRS_B      0x90
#define MRS_C      0x94
#define MRS_D      0x70
#define MRS_E      0x20
#define MRS_F      0x84
#define MRS_G      0x78
#define MRS_H      0x80
#define MRS_I      0x40
#define MRS_J      0x8E
#define MRS_K      0x74
#define MRS_L      0x88
#define MRS_M      0x58
#define MRS_N      0x50
#define MRS_O      0x7C
#define MRS_P      0x8C
#define MRS_Q      0x9A
#define MRS_R      0x68
#define MRS_S      0x60
#define MRS_T      0x30
#define MRS_U      0x64
#define MRS_V      0x82
#define MRS_W      0x6C
#define MRS_X      0x92
#define MRS_Y      0x96
#define MRS_Z      0x98
#define MRS_1      0xAF
#define MRS_2      0xA7
#define MRS_3      0xA3
#define MRS_4      0xA1
#define MRS_5      0xA0
#define MRS_6      0xB0
#define MRS_7      0xB8
#define MRS_8      0xBC
#define MRS_9      0xBE
#define MRS_0      0xBF
#define MRS_PL     0xAA  // +
#define MRS_BS     0xB2  // /
#define MRS_EQ     0xB1  // =

/*
 * Second encoding scheme allows for encoding of characters with 6 dits/dahs.
 * If bits 7-6 are 11, the length must be 6.
 * Instead of using bits 7-5 as LEN and bits 4-0 as CODE, use bits 7-6 as LEN and bits 5-0 as CODE.
 *
 * Example: ! in Morse code is -.-.-- (dah dit dah dit dah dah)
 *          Length is 6, so bits 7-6 are 11
 *          Remaining bits are 101011 (dah dit dah dit dah dah)
 *
 *                           LEN CODE
 *          !  =  -.-.--  =   11 101011  =  11101011  =  0xEB
 *                          3 .-.
 *
 */
#define MRS_QM   0xCC  // ?
#define MRS_EM   0xEB  // !
#define MRS_PD   0xD5  // .
#define MRS_CM   0xF3  // ,
#define MRS_SC   0xEA  // ;
#define MRS_CO   0xF8  // :
#define MRS_MN   0xE1  // -



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BIT_SET(n, b) ((n) & (1 << (b)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t MILLISECONDS = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

struct __FILE{
  int handle;
};


/*  RECEIVING USER INPUT
 *  UART2 will write 1 character from user to a global buffer called rxBuffer.
 *  This triggers the interrupt function which copies the character (rxBuffer[0]) to BUF[rxRead].
 *  It will then increment rxRead modulo RXLEN (circular queue).
 *
 *  rxRead is the current index in BUF where the user's character will be written.
 *  rxWrite is the index in BUF of the character currently being written to output (UART, speaker, and LED).
 *  Two indexes are needed since the user will likely type input faster than it can be printed in Morse.
 *
 *  When the write index "catches up" to the read index, it will stop writing output.
 *  This avoids writing characters past what has been input.
 *
 *  An example of reading/writing with RXLEN of 8:
 *
 *  * denotes a NULL byte
 *  r denotes the read index rxRead
 *  w denotes the write index rxWrite
 *
 *  First, an unfilled buffer
 *  BUF
 *  ********    * denotes NULL byte
 *  01234567    index of BUF
 *  r           rxRead = 0
 *  w           rxWrite = 0
 *
 *  User begins to type "Hello World!"
 *  BUF
 *  H*******
 *  01234567    index of BUF
 *   r          rxRead = 1
 *  w           rxWrite = 0
 *
 *  BUF
 *  He******
 *  01234567    index of BUF
 *    r         rxRead = 2
 *  w           rxWrite = 0
 *
 *  BUF
 *  Hel*****
 *  01234567    index of BUF
 *     r        rxRead = 3
 *  w           rxWrite = 0
 *
 *  While user is typing, rxWrite prints characters at its index.
 *  This can vary greatly depending on character, unit length, and speed of user's input.
 *  In this example, rxWrite is roughly 3 indexes behind.
 *
 *  BUF
 *  Hell****
 *  01234567    index of BUF
 *      r       rxRead = 4
 *   w          rxWrite = 1
 *
 *  Suppose the user pauses after "Hello".
 *  rxWrite can now catch up.
 *
 *  BUF
 *  Hello***
 *  01234567    index of BUF
 *       r      rxRead = 5
 *       w      rxWrite = 5
 *
 *  Since rxWrite == rxRead, output pauses.
 *  The user resumes typing and reaches the end of the buffer.
 *  Since it's a circular queue, the user's input wraps around to BUF[0].
 *
 *  BUF
 *  rello Wo
 *  01234567    index of BUF
 *   r          rxRead = 1
 *         w    rxWrite = 7
 *
 *  The user finishes typing and rxWrite catches up again.
 *
 *  BUF
 *  rld!o Wo
 *  01234567    index of BUF
 *      r       rxRead = 4
 *      w       rxWrite = 4
 *
 *  As buffer size increases, the chance of the user's input wrapping around
 *  and catching up to rxWrite decreases.
 */
#define RXLEN  255
uint8_t rxBuffer[2] = {0};
uint8_t rxRead = 0;
uint8_t rxWrite = 0;
uint8_t BUF[RXLEN] = {0};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif



#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&huart2);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

int ferror(FILE *f)
{
  return 0;
}

void printTimeElapsed()
{
  uint32_t timer_val = HAL_GetTick();
  int totalSeconds = timer_val / 1000;
  int seconds = totalSeconds % 60;
  int minutes = (totalSeconds % 3600) / 60;
  int hours = totalSeconds / 3600;
  printf("%02dh%02dm%02ds\r\n", hours, minutes, seconds);
}

void blinkShort()
{
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  HAL_Delay(DIT);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
}

void blinkLong()
{
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  HAL_Delay(DAH);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
}

int blinkChar(char c)
{
  if (c == ' ') return 1;

  if      (c >= '0' && c <= '9') ;
  else if (c >= 'A' && c <= 'Z') ;
  else if (c >= 'a' && c <= 'z') c = c - 0x20;
  else if (c == '?' || c == '!' || c == '.' ||
		   c == ',' || c == ';' || c == ':' ||
		   c == '+' || c == '-' || c == '/' || c == '=') ;
  else goto error;

  uint_fast8_t letter = 0;

  switch(c){
  case 'A':
	  letter = MRS_A; break;
  case 'B':
	  letter = MRS_B; break;
  case 'C':
	  letter = MRS_C; break;
  case 'D':
	  letter = MRS_D; break;
  case 'E':
	  letter = MRS_E; break;
  case 'F':
	  letter = MRS_F; break;
  case 'G':
	  letter = MRS_G; break;
  case 'H':
	  letter = MRS_H; break;
  case 'I':
	  letter = MRS_I; break;
  case 'J':
	  letter = MRS_J; break;
  case 'K':
	  letter = MRS_K; break;
  case 'L':
	  letter = MRS_L; break;
  case 'M':
	  letter = MRS_M; break;
  case 'N':
	  letter = MRS_N; break;
  case 'O':
	  letter = MRS_O; break;
  case 'P':
	  letter = MRS_P; break;
  case 'Q':
	  letter = MRS_Q; break;
  case 'R':
	  letter = MRS_R; break;
  case 'S':
	  letter = MRS_S; break;
  case 'T':
	  letter = MRS_T; break;
  case 'U':
	  letter = MRS_U; break;
  case 'V':
	  letter = MRS_V; break;
  case 'W':
	  letter = MRS_W; break;
  case 'X':
	  letter = MRS_X; break;
  case 'Y':
	  letter = MRS_Y; break;
  case 'Z':
	  letter = MRS_Z; break;
  case '1':
	  letter = MRS_1; break;
  case '2':
	  letter = MRS_2; break;
  case '3':
	  letter = MRS_3; break;
  case '4':
	  letter = MRS_4; break;
  case '5':
	  letter = MRS_5; break;
  case '6':
	  letter = MRS_6; break;
  case '7':
	  letter = MRS_7; break;
  case '8':
	  letter = MRS_8; break;
  case '9':
	  letter = MRS_9; break;
  case '0':
	  letter = MRS_0; break;
  case '?':
	  letter = MRS_QM; break;
  case '!':
	  letter = MRS_EM; break;
  case '.':
	  letter = MRS_PD; break;
  case ',':
	  letter = MRS_CM; break;
  case ';':
	  letter = MRS_SC; break;
  case ':':
	  letter = MRS_CO; break;
  case '+':
	  letter = MRS_PL; break;
  case '-':
	  letter = MRS_MN; break;
  case '/':
	  letter = MRS_BS; break;
  case '=':
	  letter = MRS_EQ; break;
  }

  /* If a character with length 6, interpret as second encoding */
  uint_fast8_t len = 0;
  uint_fast8_t signal = 0;
  uint_fast8_t shift = 0;
  if ((letter & 0xC0) == 0xc0){
    len = 6;
    signal = letter & 0x3F;
    shift = 5;
  }
  /* Otherwise interpret as first encoding */
  else{
    len = letter >> 5;
    signal = letter & 0x1F;
    shift = 4;
  }

  printf("%c   ", c);

  for (uint_fast8_t i = 0; i < len; i++){
    if(BIT_SET(signal, shift - i)){
      printf("-");
      fflush(stdout);
      blinkLong();
    }
    else{
      printf(".");
      fflush(stdout);
      blinkShort();
    }
    SPC_C;
  }

  printf("\r\n");

  return 0;

error:
  return -1;
}

void blinkLine(const char str[])
{
  for (uint32_t i = 0; i < strlen(str); i++){
	int ret = blinkChar(str[i]);
    if (ret == 1){ printf("\r\n"); SPC_W; }
    if (ret == 0) SPC_C;
  }
  printf("\r\n");
}

void DIT_decrease()
{
  if (DIT <= DIT_MIN){
    printf("Unit length cannot go below %dms\r\n", DIT_MIN);
    DIT = DIT_MIN;
    return;
  }

  DIT -= DIT_DEC;
  printf("Unit length decreased to %dms\r\n", DIT);
}

void DIT_increase()
{
  if (DIT >= DIT_MAX){
    printf("Unit length cannot go above %dms\r\n", DIT_MAX);
    DIT = DIT_MAX;
    return;
  }

  DIT += DIT_INC;
  printf("Unit length increased to %dms\r\n", DIT);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  //HAL_TIM_Base_Start_IT(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  fflush(stdout);
  printf("\r\n\r\n=== MORSE CODE INTERACTIVE TERMINAL ===\r\n");
  printf("Version: 0.3\r\nUnit length: %dms\r\n", DIT);
  printf("Decrease unit length with [ key\r\nIncrease unit length with ] key\r\n\r\n");

  //setvbuf(stdin, NULL, _IONBF, 0);

  uint8_t ch = 0;
  HAL_UART_Receive_IT(&huart2, rxBuffer, 1);

  while (1)
  {
    // Run outer loop until
	while (rxWrite != rxRead){
      ch = BUF[rxWrite];
      rxWrite = (rxWrite + 1) % RXLEN;

	  if (ch == ' '){
	    printf("\r\n");
	    SPC_W;
	    continue;
	  }
	  if (ch == '['){
	    DIT_decrease();
	    continue;
	  }
	  if (ch == ']'){
	    DIT_increase();
	    continue;
	  }

	  blinkChar(ch);
	  SPC_C;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Callback: timer rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    MILLISECONDS++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart2, rxBuffer, 1);
  if (rxRead + 1 == rxWrite){
	  printf("INPUT BUFFER FULL - SLOW INPUT\r\n");
	  return;
  }
  BUF[rxRead] = rxBuffer[0];
  rxRead = (rxRead + 1) % RXLEN;
}

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
