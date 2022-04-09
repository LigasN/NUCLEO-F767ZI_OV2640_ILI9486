/*****************************************************************************
 * | File      	:	DEV_Config.c
 * | Author      :   Waveshare team
 * | Function    :	Show SDcard BMP picto LCD
 * | Info        :
 *   Provide the hardware underlying interface
 *----------------
 * |	This version:   V1.0
 * | Date        :   2018-01-11
 * | Info        :   Basic version
 *
 ****************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f7xx_hal.h"

#define USE_SPI_4W 1
#define USE_IIC 0

// SPI
#define SPI1_CLK_0                                                             \
    HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_RESET)
#define SPI1_CLK_1                                                             \
    HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_SET)

#define GET_SPI1_MISO HAL_GPIO_ReadPin(SPI1_MISO_GPIO_Port, SPI1_MISO_Pin)

#define SPI1_MOSI_0                                                            \
    HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_RESET)
#define SPI1_MOSI_1                                                            \
    HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_SET)

// LCD
#define LCD_CS_0 HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_1 HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)

#define LCD_RST_0                                                              \
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RST_1                                                              \
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)

#define LCD_DC_0 HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_1 HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)

// Touch
#define TP_CS_0 HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET)
#define TP_CS_1 HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET)

#define GET_TP_BUSY HAL_GPIO_ReadPin(TP_BUSY_GPIO_Port, TP_BUSY_Pin)

#define GET_TP_IRQ HAL_GPIO_ReadPin(TP_IRQ_GPIO_Port, TP_IRQ_Pin)

// SD card
#define MAX_BMP_FILES 25
#define SD_CS_0 HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_1 HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

/*--------------------------------------------------------------------------*/
void System_Exit(void);
void PWM_SetValue(uint16_t value);
uint8_t SPI4W_Write_Byte(uint8_t value);
uint8_t SPI4W_Read_Byte(uint8_t value);

void Driver_Delay_ms(uint32_t xms);
void Driver_Delay_us(uint32_t xus);

#ifdef __cplusplus
}
#endif

#endif
