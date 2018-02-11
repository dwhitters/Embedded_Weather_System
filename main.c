/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author:
*******************************************************************************/
/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "ST7735.h"
#include "Pics.h"

/** The number of ADC pins used. */
#define NUM_ADC_READS 2u

/** Holds the multi-sequence ADC read values. */
uint16_t ADC_Vals[NUM_ADC_READS] = {0};
/** Holds the normalized ADC read values. */
float Normalized_ADC_Vals[NUM_ADC_READS] = {0.0};
/** Set to 1 when the ADC has been read. Otherwise 0. */
uint8_t ADC_Updated = 0u;
/** Holds pointers to all pictures for easily switching between them. */
const unsigned short * Pics[5] = {Night, Twilight, Overcast, Partly_Cloudy, Sunny};

void clockInit48MHzXTL(void) {  /* Sets the clock module to use the external 48 MHz crystal. */
    /* Configuring pins for peripheral/crystal usage */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    CS_setExternalClockSourceFrequency(32000,48000000); /* Enables getMCLK, getSMCLK to know externally set frequencies. */

    /* Starting HFXT in non-bypass mode without a timeout. Before we start
     * we have to change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);  // false means that there are no timeouts set, will return when stable

    /* Initializing MCLK to HFXT (effectively 48MHz) */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

/**
    Interrupt handler for Timer32. Triggers an ADC14 conversion.
 */
void Timer32_0_IRQ(void)
{
    /* Get the results of the last conversion. */
    ADC14_getMultiSequenceResult(ADC_Vals);

    int i = 0;
    for(i = 0; i < NUM_ADC_READS; ++i)
    {
        /* (Result x Ref_Voltage) / Max_14_bit_val */
        Normalized_ADC_Vals[i] = (ADC_Vals[i] * 1.2) / 16384;
    }

    /* Trigger a new conversion. The results will be checked next time this interrupt is handled. */
    ADC14_toggleConversionTrigger();

    ADC_Updated = 1u;

    Timer32_clearInterruptFlag(TIMER32_0_BASE); /* Clear the interrupt flag. */
}

/*
 * Timer32_0 Setup
 */
void Timer32_0_Init(void) {
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
                       TIMER32_PERIODIC_MODE);
    Timer32_registerInterrupt(TIMER32_0_INTERRUPT, Timer32_0_IRQ);
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    /* Set the timer to trigger an interrupt every 500ms. */
    Timer32_setCount(TIMER32_0_BASE, 24000000);
    Timer32_startTimer(TIMER32_0_BASE, false);
    Timer32_enableInterrupt(TIMER32_0_BASE);
}

/**
    Initialize the ADC14 module.
 */
void ADC_Init(void)
{
    /* Setting reference voltage to 1.2V and enabling reference */
    REF_A_setReferenceVoltage(REF_A_VREF1_2V);
    REF_A_enableReferenceVoltage();

    /* Conserve power usage by only setting the reference voltage when converting. */
    (void)ADC14_enableReferenceBurst();

    /* Setting Flash wait state */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializing ADC (MCLK/4/4) = 3MHz */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_4,
           0);

    /* Configuring GPIOs (5.5 A0) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
                                                  GPIO_TERTIARY_MODULE_FUNCTION);
    /* Configuring ADC Memory */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    /* Configure ADC memory for the photo-resistor. */
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                       ADC_INPUT_A0, false);
    /* Configure ADC memory for the TMP36. */
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                       ADC_INPUT_A1, false);

    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
}

/**
    Converts a voltage from the light sensor (0 to 1.2V) to a light level.

    @param voltage
        The voltage read from the sensor.

    @return
        The light level. 0-4.
 */
uint8_t ADC_GetLightLevel(float voltage)
{
    voltage *= 5;
    uint8_t ret_val = voltage; /* Will range from 0 to 5. */

    ret_val = (ret_val == 5) ? 4 : ret_val; /* Return value will range from 0 to 4. */

    return ret_val;
}

/**
    Returns the temperature in celsius based on the output voltage read from
    the temperature sensor.

    @param voltage
        The voltage read from the temperature sensor. Max 1.2V

    @return
        The temperature the sensor detected in celsius.
 */
float ADC_GetTempInC(float voltage)
{
    return ((voltage * 1000) - 500) / 10;
}

/**
    Returns the temperature in Fahrenheit based on the output voltage read from
    the temperature sensor.

    @param voltage
        The voltage read from the temperature sensor. Max 1.2V

    @return
        The temperature the sensor detected in Fahrenheit.
 */
float ADC_GetTempInF(float voltage)
{
    return (ADC_GetTempInC(voltage) * 1.8) + 32.0;
}

/**
    Clears the area of the screen that the string will be displayed then draws
    the string.

    @param x
        The column to start the string on.
    @param y
        The row to start the string on.
    @param pt
        Pointer to the string to draw.
    @param text_color
        The color to draw the text.
    @param back_color
        The color to draw the background.
    @param size
        The font size of the string.

    @return
        The return value of the ST7735 library draw string function.
 */
uint32_t DrawString(uint16_t x, uint16_t y, char *pt, int16_t text_color, int16_t back_color, int16_t size)
{
    ST7735_FillRect(x, y, strlen(pt), size, ST7735_WHITE);
    return ST7735_DrawString(x, y, pt, text_color, back_color, size);
}

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();
    /* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking(); /* Allow FPU operation in ISRs. */

    /* Init functions. */
    clockInit48MHzXTL();
    Timer32_0_Init();
    ADC_Init();
    ST7735_InitR(INITR_REDTAB); // initialize LCD controller IC
    ST7735_SetRotation(3);      // Sets the image rotation to type 3.
    ST7735_FillScreen(0xFFFF);            // set screen to white

    char temp_str[16] = {0};
    char light_str[16] = {0};

    uint8_t prev_light_level = 0u;
    uint8_t light_level = 0u;


    while(1)
    {
        /* Update the display when the ADC values are read. */
        if(ADC_Updated == 1u)
        {
            light_level = ADC_GetLightLevel(Normalized_ADC_Vals[0]);
            ST7735_DrawBitmap(0, 127, Pics[light_level], 160, 128); // Display the sunny image at (0, 160) with a size of 128x160

            if(prev_light_level != light_level)
            {
                /* TODO: Only update the bitmap when the light level changes. */
                /* Figure out how to clear a single line of text before overwriting it. */
            }
            sprintf(light_str, "L Lvl: %d", ADC_GetLightLevel(Normalized_ADC_Vals[0]));
            sprintf(temp_str, "Temp: %.2f", ADC_GetTempInF(Normalized_ADC_Vals[1]));
            DrawString(10, 10, light_str, ST7735_BLUE, ST7735_BLUE, 2);
            DrawString(10, 30, temp_str, ST7735_BLUE, ST7735_BLUE, 2);
            ADC_Updated = 0u;
            prev_light_level = light_level;
        }
    }
}
