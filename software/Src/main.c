/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mbport.h"
#include "arm_math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pwmMax 330	//pwm����޷�����ֹ�ջ���������Լ7.8V
#define targetMax 80	//������ռ�ձ��ٶ�ԼΪ+-118rpm���ڴ�����Ϊ+-80rpm
#define en2rpm 6.6	//���ٶ��������ֵ����ϵ��
#define ADDR_24LCxx_Write 0xA0
#define ADDR_24LCxx_Read 0xA1
#define BufferSize 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t target_1=40;//Ŀ����ٶȣ�RPM��  ����ٶ� +-80RPM 
uint16_t target_2=60;//�ڶ���Ŀ����ٶȣ�RPM��  ����ٶ� +-80RPM 
uint16_t Target=0;
int16_t output=0;//pid���ֵ
float kp=0,ki=0,kd=0;//pid����
//float kp=1.2,ki=2.5,kd=0;//pid����
uint16_t pwmVal=0;//pwmֵ
float speed=0;//�����ǰ�ٶ�
uint8_t start=0;//���������־
int16_t err=0,lerr=0,llerr=0;//�������
uint16_t cnt;	//������
uint32_t sum;	//���
USHORT   usRegHoldingStart = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
BOOL parm_read()
{
	uint8_t buf[BufferSize];
	uint8_t state=0;
	for(int i=0;i<BufferSize;i++)
	{
		state += HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, i, I2C_MEMADD_SIZE_8BIT,&buf[i],1,0xff);//ʹ��I2C�����������˲��ô��ַ�ʽ��������ֽ�д��
		HAL_Delay(5);//�˴���ʱ�ؼӣ���AT24C02дʱ���й�
	}
	if(state==0)
	{
		for(int i=0;i<BufferSize/2;i++)
		{
			static uint8_t j=0;
			usRegHoldingBuf[i] = ((uint16_t)buf[j++])|0x00;
			usRegHoldingBuf[i] |= (((uint16_t)buf[j++])<<8);
		}
		
		return TRUE;
	}
	else
		return FALSE;
}

BOOL parm_write(uint16_t* dat)
{
	uint8_t buf[BufferSize];
	uint8_t state=0;
	for(int i=0;i<BufferSize/2;i++)
	{
		static uint8_t j=0;
		buf[j++] = (uint8_t)(dat[i]&0x00ff);
		buf[j++] = (uint8_t)((dat[i]>>8)&0x00ff);
	}
	for(int i=0;i<BufferSize;i++)
	{
		state += HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, i, I2C_MEMADD_SIZE_8BIT,&buf[i],1,0xff);//ʹ��I2C�����������˲��ô��ַ�ʽ��������ֽ�д��
		HAL_Delay(5);//�˴���ʱ�ؼӣ���AT24C02дʱ���й�
	}
	if(state==0)
		return TRUE;
	else
		return FALSE;
}

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//������ʱ��2�����PWM��
  HAL_TIM_Base_Start_IT(&htim3);       //ͨ�����д��룬���жϵķ�ʽ������ʱ��3
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);      //������������ʱ��
  eMBInit( MB_RTU, 0x01, 1, 115200, MB_PAR_NONE);//��ʼ��modbus����modbusRTU����վ��ַΪ0x01���˿�Ϊ1��
  eMBEnable( );
//	parm_read();
	sum = (usRegHoldingBuf[0]+usRegHoldingBuf[1]+usRegHoldingBuf[2]+usRegHoldingBuf[3]+usRegHoldingBuf[5]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ( void )eMBPoll();
		usRegHoldingBuf[4] = (USHORT)(speed*100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(sum != (usRegHoldingBuf[0]+usRegHoldingBuf[1]+usRegHoldingBuf[2]+usRegHoldingBuf[3]+usRegHoldingBuf[5]))
		{
//			parm_write(usRegHoldingBuf);
			sum = (usRegHoldingBuf[0]+usRegHoldingBuf[1]+usRegHoldingBuf[2]+usRegHoldingBuf[3]+usRegHoldingBuf[5]);
			Target = usRegHoldingBuf[0];
			kp = (float)(usRegHoldingBuf[1]/100);
			ki = (float)(usRegHoldingBuf[2]/100);
			kd = (float)(usRegHoldingBuf[3]/100);
			start = usRegHoldingBuf[5];
		}
	  
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)	//�ж��ж��Ƿ������ڶ�ʱ��4
	{
		prvvTIMERExpiredISR( );
	}
	if(htim == &htim3)  //�ж��ж��Ƿ������ڶ�ʱ��3
	{
		cnt += 1;	//��ʱ��ÿ100ms��1
		if(start)
		{
			if(fabs(Target) > targetMax)	//���趨���ٶȽ����޷�
			{
				Target = Target>0?targetMax:-targetMax;
			}
			speed = (short)(__HAL_TIM_GetCounter(&htim1))/en2rpm;    //��ȡ��������ʱ���еļ���ֵ����ת��Ϊʵ�ʽ��ٶ�
			err = Target - speed;//����ƫ��
			output += kp*(err-lerr) + ki*err + kd*(err-2*lerr+llerr);//����ʽpid
			llerr = lerr;//��������һ��ƫ��ֵ
			lerr = err;//������һ��ƫ��ֵ
			pwmVal = abs(output);//���ȡ����ֵ
			pwmVal = (pwmVal>pwmMax)?pwmMax:pwmVal;//����޷�
			if(output>0)//�������ֵ�������Ƶ��ת��
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);//A1��������͵�ƽ
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);//A2��������ߵ�ƽ
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);//A1��������ߵ�ƽ
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);//A2��������׵�ƽ
			}
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwmVal);//�������ֵ��С���Ƶ��ת��
			__HAL_TIM_SetCounter(&htim1,0);//���������������еļ���ֵ����
		}
		else
		{
			err = 0;	//�������������ֹӰ���´�����
			output = 0;
			llerr = 0;
			lerr = 0;
			speed = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCounter(&htim1,0);
		}
	}
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
