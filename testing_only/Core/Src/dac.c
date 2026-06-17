#include "dac.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>

#define DAC0_ADDR  0x22
#define DAC1_ADDR  0x20
#define dac_vol_default 0x40  // 0dB


static const uint8_t dac_config[] =
{
    0b00101011, // 0xab, ACKS/TDM0/DIF1/PW1/RSTN
	//0b00101011,
	//0b11101011, // 0xe3, ACKS/TDM1/TDM0/DIF1/PW1/RSTN
    0x03, // DEM0/SMUTE (come up muted)
    0xe2, // PW4/PW3/PW2/PW1
    dac_vol_default,
    dac_vol_default,
    dac_vol_default,
    dac_vol_default,
    dac_vol_default,
    dac_vol_default,
    dac_vol_default,
    dac_vol_default,
    0x00,
    0x00,
    0x00,
    0x00
};

void mute_amps(bool mute)
{
	// Amp mute goes high when unmuted.
	HAL_GPIO_WritePin(GPIO_AmpMute_GPIO_Port, GPIO_AmpMute_Pin, mute ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_AmpMute2_GPIO_Port, GPIO_AmpMute2_Pin, mute ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void init_dacs()
{
	const uint16_t dacs[] = { DAC0_ADDR, DAC1_ADDR};

	printf("Initializing DACs. Only errors will be reported.\r\n");
	for (int j = 0; j < sizeof(dacs) /  sizeof(dacs[0]); ++j)
	{
		const int count = sizeof(dac_config) / sizeof(dac_config[0]);
		int i;
		for (i = 0; i < count; ++i)
		{
			uint8_t data[] = {i, dac_config[i]};
			int status = HAL_I2C_Master_Transmit(&hi2c2,
											dacs[j],          // slave address (shifted)
											data,             // pointer to data buffer
											2,                  // number of bytes to send
											100);
			if (status)
				printf("*** Status (0x%x): %d, 0x%x status %d\r\n", dacs[j], i, dac_config[i], status);
		}
	}
}

void reset_dacs()
{
	  HAL_GPIO_WritePin(GPIO_Reset_GPIO_Port, GPIO_Reset_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIO_Reset_GPIO_Port, GPIO_Reset_Pin, GPIO_PIN_SET);
}

void mute_dacs(bool mute)
{
	uint8_t data[2];
	data[0] = 1; // Control register 1
	data[1] = dac_config[1];
	if (!mute)
		data[1] &= ~1; // Turn off MUTE bit
	int status = HAL_I2C_Master_Transmit(&hi2c2, DAC0_ADDR, data, 2, 100);
	if (status)
		printf("Mute status for DAC 0 0x%x: %d\r\n", data[1], status);
	status = HAL_I2C_Master_Transmit(&hi2c2, DAC1_ADDR, data, 2, 100);
	if (status)
		printf("Mute status for DAC 1 0x%x: %d\r\n", data[1], status);
}


