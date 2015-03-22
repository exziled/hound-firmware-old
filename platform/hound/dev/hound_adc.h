/*
 * TODO: Copyright
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
 * \addtogroup hound-adc-sensors
 * @{
 *
 * \defgroup hound-adc-sensors cc2538 ADC Driver
 *
 * Driver for HOUND ADC Sensors using CC2538 Platform
 * @{
 *
 * \file
 * Header file for the cc2538dk ADC Driver
 */
#ifndef __HOUND_ADC_H
#define __HOUND_ADC_H

#include "lib/sensors.h"

/*---------------------------------------------------------------------------*/
/** \name ADC sensors
 * @{
 */

#define SOC_ADC_ADCCON1_STSEL_FULL 	(0x1 << 4)
#define SOC_ADC_ADCCON1_STSEL_TIM1	(0x2 << 4)
#define SOC_ADC_ADCCON1_STSEL_ST	(0x3 << 4)


#define SOC_ADC_ADCCON2_SREF_INT	(0x0 << 6)
#define SOC_ADC_ADCCON2_SREF_AIN7	(0x1 << 6)
#define SOC_ADC_ADCCON2_SREF_AVDD	(0x2 << 6)
#define SOC_ADC_ADCCON2_SREF_DIFF	(0x3 << 6)

#define SOC_ADC_ADCCON2_SDIV_64		(0x0 << 4)
#define SOC_ADC_ADCCON2_SDIV_128	(0x1 << 4)
#define SOC_ADC_ADCCON2_SDIV_256	(0x2 << 4)
#define SOC_ADC_ADCCON2_SDIV_512	(0x3 << 4)

#define SOC_ADC_ADCCON2_SCH_AIN0	(0x0 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN1	(0x1 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN2	(0x2 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN3	(0x3 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN4	(0x4 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN5	(0x5 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN6	(0x6 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN7	(0x7 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN01	(0x8 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN23	(0x9 << 0)
#define SOC_ADC_ADCCON2_SCH_AIN45	(0xA << 0)
#define SOC_ADC_ADCCON2_SCH_AIN67	(0xB << 0)
#define SOC_ADC_ADCCON2_SCH_GND		(0xC << 0)
#define SOC_ADC_ADCCON2_SCH_RES		(0xD << 0)
#define SOC_ADC_ADCCON2_SCH_TMP		(0xE << 0)
#define SOC_ADC_ADCCON2_SCH_VDD3	(0xF << 0)

typedef struct hound_raw_data {
	uint8_t data_size;
	uint16_t * current_socket_1;
	uint16_t * current_socket_2;
	uint16_t * voltage;
} HOUND_DATA;

extern HOUND_DATA hound_data;

/* Internal CC2538 Sensors */

#define SENSOR_TEMP			SOC_ADC_ADCCON_CH_TEMP
#define SENSOR_VDD_3		SOC_ADC_ADCCON_CH_VDD_3


/* HOUND Specific Sensors */

#define SENSOR_ISOCKA 		SOC_ADC_ADCCON_CH_AIN0
#define SENSOR_ISOCKB		SOC_ADC_ADCCON_CH_AIN1
#define SENSOR_VOLT			SOC_ADC_ADCCON_CH_AIN2


#define IS_HOUND_SENSOR(SENSOR)    (((SENSOR) == SENSOR_TEMP) || \
 									((SENSOR) == SENSOR_VDD_3) || \
 									((SENSOR) == SENSOR_ISOCKA) || \
 									((SENSOR) == SENSOR_ISOCKB) || \
 									((SENSOR) == SENSOR_VOLT))

int init_hound_data(uint8_t data_size);
void init_hound_adc(void);
int hound_test(void);

#endif // __HOUND_ADC_H

/**
 * @}
 * @}
 */
