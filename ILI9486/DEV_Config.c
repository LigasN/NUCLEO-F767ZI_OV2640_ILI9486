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
#include "DEV_Config.h"
#include "spi.h"
#include "stm32f7xx_hal.h"
#include "tim.h"

/*****************************************************************************
function:	System Init
note:
        Initialize the communication method
*****************************************************************************/
void System_Exit(void) {}

void PWM_SetValue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/*********************************************
function:	Hardware interface
note:
        SPI4W_Write_Byte(value) :
                Register hardware SPI
*********************************************/
uint8_t SPI4W_Write_Byte(uint8_t value)
{
    HAL_SPI_Transmit(&hspi2, &value, 1, 1000);
    return 0;
}

uint8_t SPI4W_Read_Byte(uint8_t value)
{
    uint8_t RxData = 0;
    HAL_SPI_TransmitReceive(&hspi2, &value, &RxData, 1, 1000);
    return RxData;
}

/*****************************************************************************
function:	Delay function
note:
        Driver_Delay_ms(xms) : Delay x ms
        Driver_Delay_us(xus) : Delay x us
*****************************************************************************/
void Driver_Delay_ms(uint32_t xms) { HAL_Delay(xms); }

void Driver_Delay_us(uint32_t xus)
{
    for (uint32_t j = xus; j > 0; j--)
        ;
}
