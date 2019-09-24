/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sd_hal_mpu6050.h"
#include "math.h"
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
SD_MPU6050 mpu1;

typedef enum {
  // used for indicating mcu orientation
  CURRENT_ORIENT_RECOV = 0x00,
  CURRENT_ORIENT_LEFT,
  CURRENT_ORIENT_RIGHT,
  CURRENT_ORIENT_UP,
  CURRENT_ORIENT_DOWN
} current_orient;

typedef enum {
  // used for controlling mcu operation
  STATE_IDLE = 0x00,
  STATE_OPERATE,
  STATE_CALIBRATE,
  STATE_TEST
} state_t;

volatile state_t state = STATE_IDLE;
volatile uint8_t error_cnt = 0;      // used for resetting state
volatile uint8_t calibrate_cnt = 0;  // used for calibration counter

char digits[10] = "0123456789";

// read data from sensor
int16_t a_x = 0;
int16_t a_y = 0;
int16_t a_z = 0;

int16_t g_x = 0;
int16_t g_y = 0;
int16_t g_z = 0;

// calibration offset
int32_t offset_a_x = 0;
int32_t offset_a_y = 0;
int32_t offset_a_z = 0;
int32_t offset_g_x = 0;
int32_t offset_g_y = 0;
int32_t offset_g_z = 0;

// corrected value
double cor_a_x = 0;
double cor_a_y = 0;
double cor_a_z = 0;
double cor_g_x = 0;
double cor_g_y = 0;
double cor_g_z = 0;

// roll, pitch, yaw values
double roll = 0;
double pitch = 0;
double yaw = 0;

current_orient orient = 0x00;
volatile uint8_t turbo = 0; // turbo command flag

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void acce_uart_transmit(float a_x, float a_y, float a_z, current_orient orient);
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
  SD_MPU6050_Result result;
  uint8_t mpu_ok[16] = {"MPU WORK FINE\r\n"};
  uint8_t mpu_not[18] = {"MPU NOT WORKING\r\n"};
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // turn on mpu6050 sensor
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  
  HAL_TIM_Base_Init(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (state)
    {
    case STATE_IDLE:
      result = SD_MPU6050_Init(&hi2c2,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_500s );
      HAL_Delay(10);

      // check if MPU6050 is active
      if (result == SD_MPU6050_Result_Ok)
      {
        HAL_UART_Transmit(&huart2, mpu_ok, 16, HAL_MAX_DELAY);
        state = STATE_OPERATE;
      }
      break;

    case STATE_OPERATE:
      // read accelero values
      SD_MPU6050_ReadAccelerometer(&hi2c2, &mpu1);
      a_x = mpu1.Accelerometer_X + offset_a_x;
      a_y = mpu1.Accelerometer_Y + offset_a_y;
      a_z = mpu1.Accelerometer_Z + offset_a_z;

      // read gyro values
      SD_MPU6050_ReadGyroscope(&hi2c2, &mpu1);
      g_x = mpu1.Gyroscope_X + offset_g_x;
      g_y = mpu1.Gyroscope_Y + offset_g_y;
      g_z = mpu1.Gyroscope_Z + offset_g_z;

      // correct accelero values
      cor_a_x = (double) a_x * mpu1.Acce_Mult;
      cor_a_y = (double) a_y * mpu1.Acce_Mult;
      cor_a_z = (double) a_z * mpu1.Acce_Mult;

      // correct gyro values
      cor_g_x = (double) g_x * mpu1.Gyro_Mult;
      cor_g_y = (double) g_y * mpu1.Gyro_Mult;
      cor_g_z = (double) g_z * mpu1.Gyro_Mult;

      // roll, pitch, yaw calculation
      roll = 180 * atan(cor_g_x / sqrt(cor_g_x * cor_g_x + cor_g_z + cor_g_z)) / M_PI; 
      pitch = 180 * atan(cor_g_x / sqrt(cor_g_y * cor_g_y + cor_g_z * cor_g_z)) / M_PI;
      yaw = 180 * atan (cor_g_z / sqrt(cor_g_x * cor_g_x + cor_g_z * cor_g_z)) / M_PI;

      // if (orient == CURRENT_ORIENT_RECOV)
      // {
      //   // right
      //   if (cor_a_x < -0.5)
      //   {
      //     orient = CURRENT_ORIENT_RIGHT;
      //     acce_uart_transmit(cor_a_x, cor_a_y, cor_a_z, orient);
      //   }
      //   // left
      //   else if (cor_a_x > 0.5)
      //   {
      //     orient = CURRENT_ORIENT_LEFT;
      //     acce_uart_transmit(cor_a_x, cor_a_y, cor_a_z, orient);
      //   }

      //   // up
      //   if (cor_a_y < -0.3)
      //   {
      //     orient = CURRENT_ORIENT_UP;
      //     acce_uart_transmit(cor_a_x, cor_a_y, cor_a_z, orient);
      //   }
      //   // down
      //   else if (cor_a_y > 0.3)
      //   {
      //     orient = CURRENT_ORIENT_DOWN;
      //     acce_uart_transmit(cor_a_x, cor_a_y, cor_a_z, orient);
      //   }
      //   else
      //   {
      //     acce_uart_transmit(cor_a_x, cor_a_y, cor_a_z, orient);
      //   }
      // }
      // else
      // {
      //   // orientation recovery
      //   switch (orient)
      //   {
      //   case CURRENT_ORIENT_RIGHT:
      //     if (cor_a_x > 0.1)
      //     {
      //       orient = CURRENT_ORIENT_RECOV;
      //     }  
      //     break;
      //   case CURRENT_ORIENT_LEFT:
      //     if (cor_a_x < -0.1)
      //     {
      //       orient = CURRENT_ORIENT_RECOV;
      //     }
      //     break;
      //   case CURRENT_ORIENT_UP:
      //     if (cor_a_y > 0.1)
      //     {
      //       orient = CURRENT_ORIENT_RECOV;
      //     }
      //     break;
      //   case CURRENT_ORIENT_DOWN:
      //     if (cor_a_y < -0.1)
      //     {
      //       orient = CURRENT_ORIENT_RECOV;
      //     }
      //     break;
      //   default:
      //     break;
      //   }

      //   acce_uart_transmit(cor_a_x, cor_a_y, cor_a_z, orient);
      // }      
      break;

    case STATE_CALIBRATE:
      // disable interrupt on LINE8. This will prevent interrupt button works 
      // while performing calibration.
      EXTI->IMR = 0x2200;

      // turn calibration LED on
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(500);

      // set all offsets to zero
      offset_a_x = 0;
      offset_a_y = 0;
      offset_a_z = 0;
      offset_g_x = 0;
      offset_g_y = 0;
      offset_g_z = 0;

      for (calibrate_cnt = 0; calibrate_cnt < 10; ++calibrate_cnt)
      {
        // accumulate sampling values of Accelero and Gyro
        SD_MPU6050_ReadAccelerometer(&hi2c2, &mpu1);
        offset_a_x += mpu1.Accelerometer_X;
        offset_a_y += mpu1.Accelerometer_Y;
        offset_a_z += mpu1.Accelerometer_Z;

        SD_MPU6050_ReadGyroscope(&hi2c2, &mpu1);
        offset_g_x += mpu1.Gyroscope_X;
        offset_g_y += mpu1.Gyroscope_Y;
        offset_g_z += mpu1.Gyroscope_Z;

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        HAL_Delay(200);
      }
      
      // average to obtain offset
      offset_a_x = offset_a_x / -10;
      offset_a_y = offset_a_y / -10;
      offset_a_z = offset_a_z / -10;
      offset_g_x = offset_g_x / -10;
      offset_g_y = offset_g_y / -10;
      offset_g_z = offset_g_z / -10;
      
      // turn off calibration LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      
      // go back to operate
      state = STATE_OPERATE;

      // turn interrupt back on
      EXTI->IMR = 0x2300;

      break;

    case STATE_TEST:
      // for test purpose
      break;

    default:
      break;
    }

    HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

///////////////////
// *--- ISR ---* //
///////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_9)
  {
    turbo = 1;
  }
  else if (GPIO_Pin == GPIO_PIN_8)
  {
    state = STATE_CALIBRATE;
  }
}


//////////////////////////////////////
// *--- user defined functions ---* //
//////////////////////////////////////
void acce_uart_transmit(float a_x, float a_y, float a_z, current_orient orient)
{
  // command array
  uint8_t command[18];
  uint8_t temp_bit;
  uint8_t index = 15;

  //  turbo bug when orient is X
  if (turbo)
  // sending turbo command
  {
    turbo = 0;
    command[0] = (uint8_t) 'T'; // Turbo command
    
    for (index = 1; index < 16; ++index)
      command[index] = 0;

    command[16] = (uint8_t) '\r';
    command[17] = (uint8_t) '\n';

    HAL_UART_Transmit(&huart2, command, 18, HAL_MAX_DELAY);

    return 0;
  }

  // accelerometer transmit values
  int16_t a_x_transmit = 0;
  int16_t a_y_transmit = 0;
  int16_t a_z_transmit = 0;

  // cast float to uint16
  a_x_transmit = a_x * 1000;
  a_y_transmit = a_y * 1000;
  a_z_transmit = a_z * 1000;

  // *--- bits assignment ---* //

  // *- HEADER -* //
  // orientation bit
  if (orient == CURRENT_ORIENT_UP) command[0] = (uint8_t) 'U';
  else if (orient == CURRENT_ORIENT_DOWN) command[0] = (uint8_t) 'D';
  else if (orient == CURRENT_ORIENT_LEFT) command[0] = (uint8_t) 'L';
  else if (orient == CURRENT_ORIENT_RIGHT) command[0] = (uint8_t) 'R';
  else if (orient == CURRENT_ORIENT_RECOV) command[0] = (uint8_t) 'X';
  else command[0] = (uint8_t) 'X';

  // sign bytes
  // a_x
  if (a_x_transmit > 0)
  {
    command[1] = (uint8_t) 'P';
  } 
  else
  {
    command[1] = (uint8_t) 'N';
    a_x_transmit = - a_x_transmit; // convert to positive
  }

  // a_y
  if (a_y_transmit > 0)
  {
    command[2] = (uint8_t) 'P';
  } 
  else
  {
    command[2] = (uint8_t) 'N';
    a_y_transmit = - a_y_transmit; // convert to positive
  }

  // a_z
  if (a_z_transmit > 0)
  {
    command[3] = (uint8_t) 'P';
  } 
  else
  {
    command[3] = (uint8_t) 'N';
    a_z_transmit = - a_z_transmit; // convert to positive
  }


  // *- PAYLOAD -* //

  // *-- a_z bytes --*
  while (index > 11)
  {
    temp_bit = a_z_transmit % 10;
    command[index] = digits[temp_bit];
    a_z_transmit /= 10;
    index--;
  }

  // *-- a_y bytes --*
  while (index > 7)
  {
    temp_bit = a_y_transmit % 10;
    command[index] = digits[temp_bit];
    a_y_transmit /= 10;
    index--;
  }
  
  // *-- a_x bytes --*
  while (index > 3)
  {
    temp_bit = a_x_transmit % 10;
    command[index] = digits[temp_bit];
    a_x_transmit /= 10;
    index--;
  }
  command[16] = (uint8_t) '\r';
  command[17] = (uint8_t) '\n';

  HAL_UART_Transmit(&huart2, command, 18, HAL_MAX_DELAY);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
