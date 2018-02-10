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

#include "ST7735.h"

/** The number of ADC pins used. */
#define NUM_ADC_READS 2u

/** Holds the multi-sequence ADC read values. */
uint16_t ADC_Vals[NUM_ADC_READS] = {0};
/** Holds the normalized ADC read values. */
float Normalized_ADC_Vals[NUM_ADC_READS] = {0};

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

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    /* Init functions. */
    clockInit48MHzXTL();
    Timer32_0_Init();

    /* Setting reference voltage to 1.2V and enabling reference */
    REF_A_setReferenceVoltage(REF_A_VREF1_2V);
    REF_A_enableReferenceVoltage();

    /* Conserve power usage by only setting the reference voltage when converting. */
    (void)ADC14_enableReferenceBurst();

    /* Setting Flash wait state */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();

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

    while(1)
    {
        MAP_PCM_gotoLPM0(); /* Sleep until an interrupt occurs. */
    }
}
