/*
 * Copyright (c) 2013, ADVANSEE - http://www.advansee.com/
 * Benoît Thébaudeau <benoit.thebaudeau@advansee.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538dk-adc-sensor
 * @{
 *
 * \file
 *  Driver for the SmartRF06EB ADC
 */

// Core
#include "contiki.h"
#include "sys/clock.h"

// CPU
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "dev/adc.h"
#include "dev/udma.h"
#include "dev/gptimer.h"
#include "dev/nvic.h"

// Platform
#include "dev/hound_adc.h"


// Standard Libraries
#include <stdint.h>
#include <stdlib.h>
 
//#include <malloc.h>


// #define ADC_ALS_PWR_PORT_BASE    GPIO_PORT_TO_BASE(ADC_ALS_PWR_PORT)
// #define ADC_ALS_PWR_PIN_MASK     GPIO_PIN_MASK(ADC_ALS_PWR_PIN)
// #define ADC_ALS_OUT_PIN_MASK     GPIO_PIN_MASK(ADC_ALS_OUT_PIN)


/**
 * HOUND ADC Configuration
 *
 * PA0 - Current Socket 1
 * PA1 - Current Socket 2
 * PA2 - Voltage System
 * PA3 - Additional Analog Input
 *
 * ADC is configured to use Sequential DMA, storing one sample from each of the 3
 * (or 4) ADC Channels.  Data is moved to a temporary array for analysis.
 *
 */

HOUND_DATA hound_raw_data;

int init_hound_data(uint8_t data_size)
{
	hound_raw_data.data_size = data_size;

	hound_raw_data.current_socket_1 = (uint16_t *)calloc(data_size, sizeof(uint16_t));
	hound_raw_data.current_socket_2 = (uint16_t *)calloc(data_size, sizeof(uint16_t));
	hound_raw_data.voltage = (uint16_t *)calloc(data_size, sizeof(uint16_t));

	if (hound_raw_data.current_socket_1 == NULL
		|| hound_raw_data.current_socket_2 == NULL
		|| hound_raw_data.voltage == NULL)
	{
		// ERROR: Could not allocate memory
		return -1;
	}

	return 0;
}

void init_hound_adc(void)
{
	// Configure ADC to convert on triggers from Timer 1 Channel 0
	REG(SOC_ADC_ADCCON1) |=		SOC_ADC_ADCCON1_STSEL_TIM1;

	// Reference AVDD
	// 128 Decimation
	// AIN 3 Ends Sequence
	REG(SOC_ADC_ADCCON2) |= 	(SOC_ADC_ADCCON2_SREF_AVDD
								| SOC_ADC_ADCCON2_SDIV_128
								| SOC_ADC_ADCCON2_SCH_AIN2);

	// Configure DMA for ADC Operation
	udma_set_channel_assignment(14, 0);
	udma_set_channel_assignment(15, 0);
	udma_set_channel_assignment(16, 0);

	// If extra is enabled, do this...
	//udma_set_channel_assignment(17, 0);

	udma_channel_mask_clr(14);		// Enable peripeheral interrupts
	udma_set_channel_control_word(14, 	(UDMA_CHCTL_DSTINC_16 			// 12 Bit DAC (so 16 bit data size) and increment on the destination
										| UDMA_CHCTL_SRCINC_NONE 
										| UDMA_CHCTL_SRCSIZE_16 
										| UDMA_CHCTL_DSTSIZE_16
										| UDMA_CHCTL_ARBSIZE_1
										| UDMA_CHCTL_XFERMODE_BASIC		// Do 1 transfer without interrupt
										| udma_xfer_size(200)));		// Do 200 transfers (hopefully)
	udma_set_channel_src(14, SOC_ADC_ADCL_ADC);
	udma_set_channel_dst(14, hound_raw_data.voltage + hound_raw_data.data_size);

	udma_channel_mask_clr(15);		// Enable peripeheral interrupts
	udma_set_channel_control_word(15, 	(UDMA_CHCTL_DSTINC_16 			// 12 Bit DAC (so 16 bit data size) and increment on the destination
										| UDMA_CHCTL_SRCINC_NONE 
										| UDMA_CHCTL_SRCSIZE_16 
										| UDMA_CHCTL_DSTSIZE_16
										| UDMA_CHCTL_ARBSIZE_1
										| UDMA_CHCTL_XFERMODE_BASIC		// Do 1 transfer without interrupt
										| udma_xfer_size(200)));		// Do 200 transfers (hopefully)
	udma_set_channel_src(15, SOC_ADC_ADCL_ADC);
	udma_set_channel_dst(15, hound_raw_data.current_socket_1 + hound_raw_data.data_size);

	udma_channel_mask_clr(16);		// Enable peripeheral interrupts
	udma_set_channel_control_word(16, 	(UDMA_CHCTL_DSTINC_16 			// 12 Bit DAC (so 16 bit data size) and increment on the destination
										| UDMA_CHCTL_SRCINC_NONE 
										| UDMA_CHCTL_SRCSIZE_16 
										| UDMA_CHCTL_DSTSIZE_16
										| UDMA_CHCTL_ARBSIZE_1
										| UDMA_CHCTL_XFERMODE_BASIC		// Do 1 transfer without interrupt
										| udma_xfer_size(200)));		// Do 200 transfers (hopefully)
	udma_set_channel_src(16, SOC_ADC_ADCL_ADC);
	udma_set_channel_dst(16, hound_raw_data.current_socket_2 + hound_raw_data.data_size);

	printf("Timer Configured\r\n");

	// Configure Timer
	REG(GPT_0_BASE + GPTIMER_CTL) 	&= 	~(GPTIMER_CTL_TBEN);		// Disable GPT0-TimerB
	REG(GPT_0_BASE + GPTIMER_CFG) 	= 	0x04;
	REG(GPT_0_BASE + GPTIMER_TBMR) 	|= 	(GPTIMER_TBMR_TBMR_PERIODIC		// Periodic with Interrupt?
										| GPTIMER_TBMR_TBMIE);
	REG(GPT_0_BASE + GPTIMER_TBILR) = 	0xFFF;						// Load Initial Value
	REG(GPT_0_BASE + GPTIMER_IMR) 	|= 	GPTIMER_IMR_TBMIM;			// Enable "match" interrupt
	REG(GPT_0_BASE + GPTIMER_TBMATCHR)	= 0xF;
	REG(GPT_0_BASE + GPTIMER_CTL)	|= 	GPTIMER_CTL_TBEN;			// Enable Timer

	GPIO_SET_OUTPUT(GPIO_C_BASE, 1 << 7);
	GPIO_SET_PIN(GPIO_C_BASE, 1 << 7);
	GPIO_CLR_PIN(GPIO_C_BASE, 1 << 7);

	nvic_interrupt_enable(NVIC_INT_GPTIMER_0B);
}


int hound_test(void)
{
	return hound_raw_data.voltage[10];
}
