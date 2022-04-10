/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * 	Created on: 23.02.2019
 *	Modified on:  23.02.2021
 *
 *	Copyright 2021 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person
 *obtaining a copy of this software and associated documentation files
 *(the "Software"), to deal in the Software without restriction,
 *including without limitation the rights to use, copy, modify, merge,
 *publish, distribute, sublicense, and/or sell copies of the Software,
 *and to permit persons to whom the Software is furnished to do so,
 *subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be
 *included in all copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 *BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 *ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ov2640.h"

#include "STM_registers.h"
// LCD
#include "LCD_Driver.h"
#include "LCD_GUI.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * Code debugging option
 */
//#define DEBUG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**
 * Resolution selection
 */

//#define RES160X120
//#define RES320X240
//#define RES640X480
//#define RES800x600
//#define RES1024x768
//#define RES1280x960

//#define STM160x120
#define STM320x240
//#define STM480x272
//#define STM640x480

#ifdef STM160x120
unsigned imgRes                     = RES_STM160x120;
uint8_t frameBuffer[RES_STM160x120] = {0};
#endif

#ifdef STM320x240
unsigned imgRes                     = RES_STM320x240;
uint8_t frameBuffer[RES_STM320x240] = {0};
#endif

#ifdef STM480x272
unsigned imgRes                     = RES_STM480x272;
uint8_t frameBuffer[RES_STM480x272] = {0};
#endif

#ifdef STM640x480
unsigned imgRes                     = RES_STM640x480;
uint8_t frameBuffer[RES_STM640x480] = {0};
#endif

#ifdef RES160X120
enum imageResolution imgRes      = RES_160X120;
uint8_t frameBuffer[RES_160X120] = {0};
#endif

#ifdef RES320X240
enum imageResolution imgRes      = RES_320X240;
uint8_t frameBuffer[RES_320X240] = {0};
#endif

#ifdef RES640X480
enum imageResolution imgRes      = RES_640X480;
uint8_t frameBuffer[RES_640X480] = {0};
#endif

#ifdef RES800x600
enum imageResolution imgRes      = RES_800x600;
uint8_t frameBuffer[RES_800x600] = {0};
#endif

#ifdef RES1024x768
enum imageResolution imgRes       = RES_1024x768;
uint8_t frameBuffer[RES_1024x768] = {0};
#endif

#ifdef RES1280x960
enum imageResolution imgRes       = RES_1280x960;
uint8_t frameBuffer[RES_1280x960] = {0};
#endif

ushort mutex           = 0;
uint16_t bufferPointer = 0;
ushort headerFound     = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vprint(const char* fmt, va_list argp)
{
    char string[200];
    if (0 < vsprintf(string, fmt, argp)) // build string
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)string, strlen(string),
                          0xffffff); // send message via UART
    }
}

void my_printf(const char* fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

int32_t firstNonZeroValue(const uint8_t* array, int32_t size)
{
    for (int32_t i = 0; i < size; ++i)
    {
        if (array[i] != 0U)
            return i;
    }
    return -1;
}

/// Refreshes DCMI status information on display
void refreshStatusInfo()
{
    const char* text;
    switch (HAL_DCMI_GetState(&hdcmi))
    {
        case HAL_DCMI_STATE_RESET:
        {
            text = "DCMI not yet initialized or disabled.";
            break;
        }
        case HAL_DCMI_STATE_READY:
        {
            text = "DCMI initialized and ready for use.";
            break;
        }
        case HAL_DCMI_STATE_BUSY:
        {
            text = "DCMI internal processing is ongoing.";
            break;
        }
        case HAL_DCMI_STATE_TIMEOUT:
            text = "DCMI timeout state.";
            break;
        case HAL_DCMI_STATE_ERROR:
            text = "DCMI error state.";
            break;
        case HAL_DCMI_STATE_SUSPENDED:
            text = "DCMI suspend state.";
            break;
        default:
            text = "Unknown state.";
    }
    my_printf("HAL_DCMI_GetState: %s", text);
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef* hdcmi)
{
    // NOTE : This function Should not be modified, when the callback
    // is needed, the HAL_DCMI_FrameEventCallback could be implemented in
    // the user file

    // This is the user implementation.

    my_printf("End of shooting\r\n");
    // HAL_UART_DMAStop(&huart1);FF D8 FF E0
    my_printf("%x  %x  %x  %x\r\n", frameBuffer[0], frameBuffer[1],
              frameBuffer[2], frameBuffer[3]);

    int32_t index = firstNonZeroValue(frameBuffer, imgRes);
    if (index != -1)
        my_printf("Success\r\n");
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef* hdcmi)
{
    // NOTE : This function Should not be modified, when the callback
    // is needed, the HAL_DCMI_ErrorCallback could be implemented in
    // the user file
    //
    // This is the user implementation.

    char* text = "Unknown error";
    switch (HAL_DCMI_GetError(hdcmi))
    {
        case HAL_DCMI_ERROR_NONE:
            text = "No error";
            break;
        case HAL_DCMI_ERROR_OVR:
            text = "Overrun error";
            break;
        case HAL_DCMI_ERROR_SYNC:
            text = "Synchronization error";
            break;
        case HAL_DCMI_ERROR_TIMEOUT:
            text = "Timeout error";
            break;
        case HAL_DCMI_ERROR_DMA:
            text = "DMA error";
        default:
            break;
    }

    my_printf("DCMI Error callback: %s.\r\n", text);
    refreshStatusInfo();
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

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_DCMI_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT; // SCAN_DIR_DFT = D2U_L2R
    LCD_Init(Lcd_ScanDir, 1000);
    GUI_Clear(WHITE);

    OV2640_Init(&hi2c1, &hdcmi);
    HAL_Delay(10);
    // OV2640_ResolutionOptions(imgRes);
    STM_OV2640_ResolutionConfiguration(imgRes);
    HAL_Delay(10);

    /**
     * Extra options
     */
    // OV2640_Brightness(Brightness_2);
    // HAL_Delay(10);
    // OV2640_Saturation(Saturation_2);
    // HAL_Delay(10);
    // OV2640_Contrast(Contrast_2);
    // HAL_Delay(10);
    // OV2640_SpecialEffect(Bluish);
    // HAL_Delay(10);
    // OV2640_LightMode(Auto);
    // HAL_Delay(10);
#ifdef DEBUG
    my_printf("Finishing configuration \r\n");
#endif

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    /**
     * Pressing button (B1) on the Nucleo board will take a picture
     * and return JPEG via the serial port.
     */
    while (1)
    {
        if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
        {
            if (mutex == 1)
            {
                my_printf("Button pushed. \r\n");
                memset(frameBuffer, 0, sizeof frameBuffer);
                OV2640_CaptureSnapshot((uint32_t)frameBuffer, imgRes);
                // while (1)
                {
                    if (headerFound == 0 &&
                        frameBuffer[bufferPointer] == 0xFF &&
                        frameBuffer[bufferPointer + 1] == 0xD8)
                    {
                        headerFound = 1;
#ifdef DEBUG
                        my_printf("Found header of JPEG file \r\n");
#endif
                    }
                    if (headerFound == 1 &&
                        frameBuffer[bufferPointer] == 0xFF &&
                        frameBuffer[bufferPointer + 1] == 0xD9)
                    {
                        bufferPointer = bufferPointer + 2;
#ifdef DEBUG
                        my_printf("Found EOF of JPEG file \r\n");
#endif
                        headerFound = 0;
                        break;
                    }

                    if (bufferPointer >= 65535)
                    {
                        break;
                    }
                    bufferPointer++;
                }
#ifdef DEBUG
                my_printf("Image size: %d bytes \r\n", bufferPointer);
#endif

                HAL_UART_Transmit_DMA(
                    &huart3, frameBuffer,
                    bufferPointer); // Use of DMA may be necessary for
                                    // larger data streams.
                GUI_DrawImage(LCD_X, LCD_Y, frameBuffer, imgRes);
                bufferPointer = 0;
                mutex         = 0;
                my_printf("Displayed \r\n");

                int i = firstNonZeroValue(frameBuffer, imgRes);
                if (i != -1)
                    my_printf("There is no zero value idx: %d\r\n", i);
                // dump bin value
                // C:\Users\norbe\STM32CubeIDE\workspace_1.8.0\Test_DCMI\out\dumpt.jpg
                // frameBuffer
            }
        }
        else
        {
            mutex = 1;
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

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 210;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 9;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
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
    /* User can add his own implementation to report the HAL error
     * return state */

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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and
     line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
