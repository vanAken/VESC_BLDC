/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se
	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

// V2

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include <math.h>
#include "mc_interface.h"
#include "luna_display_serial.h"
#include "conf_general.h"
// (volker, 07.10.2026) Verdrahtung je Controller, siehe hw_75_100_V2.h.
// Gemessen und nach Funktion geprueft, nie nach Farbe oder Plan.
// (volker, 11.10.2026) RECHTSRAHMEN UND KONFIGURATIONSSPERRE HAENGEN AN DER UUID.
// Nur Controller mit s_pedelec = true (eigene, zugelassene S-Pedelecs) laufen
// mit 45 km/h +5 % und ohne Konfigurationssperre. JEDER andere Controller ist
// ein PEDELEC 25 (25 km/h, Schiebehilfe 6 km/h, Akkustrom max. 10 A) und
// unterstuetzt erst, wenn eine Motorkonfiguration gespeichert ist (Wizard).
// Wer selbst kompiliert und ein zugelassenes S-Pedelec hat, traegt hier seine
// UUID ein (luna_pins zeigt sie) - die Verantwortung liegt dann bei ihm.
static const luna_profil_t luna_profile[] = {
	// Eigene Controller stehen in luna_profile_lokal.h (nicht im Repository,
	// siehe .gitignore). Vorlage: luna_profile_lokal.h.beispiel
#if __has_include("luna_profile_lokal.h")
#include "luna_profile_lokal.h"
#endif
	// Endmarke: UUID 0 kommt bei echten Chips nicht vor
	{"(Ende)", {0}, false, false, false, 11, false, false, 0.0f, 0.0f},
};
static const luna_profil_t luna_profil_standard =
	{"Standard (unbekannte UUID)", {0}, false, false, true, 11, false, false, 0.0f, 0.0f};
// (12.10.2026) Bremse an ADC1 wie im Plan und README; PC10 bleibt frei.

// PC10 weder Bremse noch Licht noch Display-Senden -> stilllegen (weisses Rad)
bool luna_pc10_unbenutzt(void) {
	const luna_profil_t *p = luna_profil();
	bool licht_pc10 = !p->licht_an_pb && p->licht_pin == 10;
	return p->bremse_adc1 && !licht_pc10 && !p->display_tx_pc10;
}

const luna_profil_t *luna_profil(void) {
	static const luna_profil_t *p = 0;
	if (p) {
		return p;
	}
	p = &luna_profil_standard;
	for (unsigned i = 0; i < sizeof(luna_profile) / sizeof(luna_profile[0]); i++) {
		bool gleich = true;
		for (int k = 0; k < 12; k++) {
			if (STM32_UUID_8[k] != luna_profile[i].uuid[k]) {
				gleich = false;
				break;
			}
		}
		if (gleich) {
			p = &luna_profile[i];
			break;
		}
	}
	return p;
}

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// external Speedsensor
#ifdef HW_HAS_WHEEL_SPEED_SENSOR
	palSetPadMode(HW_SPEED_SENSOR_PORT, HW_SPEED_SENSOR_PIN, PAL_MODE_INPUT_PULLUP);
#elif defined(LUNA_LIGHT_GPIO)
	// (volker) Lichtausgang (PC11, Testcontroller PC10). Langsame Flanke genuegt, das Signal
	// schaltet nur. OSPEED_LOWEST haelt die Stoerabstrahlung klein.
	palSetPadMode(LUNA_LIGHT_GPIO, LUNA_LIGHT_PIN,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
	palClearPad(LUNA_LIGHT_GPIO, LUNA_LIGHT_PIN);
#else
	// (volker) ohne Radsensor bleibt PC11 passiver Eingang wie zuvor
	palSetPadMode(GPIOC, 11, PAL_MODE_INPUT_PULLUP);
#endif
	// external Brakesensor (PC10) - nur im Standardprofil. Ist PC10 der
	// Lichtausgang, wurde er oben schon gesetzt; sonst (weisses Rad,
	// PC10 defekt) ohne Pull-up stilllegen.
	if (!luna_profil()->bremse_adc1) {
		palSetPadMode(HW_BRAKE_SENSOR_PORT, HW_BRAKE_SENSOR_PIN, PAL_MODE_INPUT_PULLUP);
	} else if (luna_pc10_unbenutzt()) {
		palSetPadMode(HW_BRAKE_SENSOR_PORT, HW_BRAKE_SENSOR_PIN, PAL_MODE_INPUT_ANALOG);
	}

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Phase filters (jaykup - disabled)
	//palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
	//		PAL_MODE_OUTPUT_PUSHPULL |
	//		PAL_STM32_OSPEED_HIGHEST);
	//PHASE_FILTER_OFF();

		// Current filter (jaykup - disabled)
	//palSetPadMode(GPIOD, 2,
	//		PAL_MODE_OUTPUT_PUSHPULL |
	//		PAL_STM32_OSPEED_HIGHEST);
	//
	//CURRENT_FILTER_OFF(); (jaykup - disabled)

	// AUX pin
	//AUX_OFF();
	//palSetPadMode(AUX_GPIO, AUX_PIN,
	//		PAL_MODE_OUTPUT_PUSHPULL |
	//		PAL_STM32_OSPEED_HIGHEST);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);
	// (volker, 26.09.) 6, nicht 3: die Codes laufen in Zweier-Schritten,
	// Display 3 ist PAS_LEVEL_6. Vorher stand hier die 3 und damit eine
	// Stufe zwischen Display 1 und 2. Gilt ohne Display und bei jedem
	// Display, das neu dazukommt.
	luna_display_serial_start(6);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 6, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 6, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 6, ADC_SampleTime_15Cycles);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}
// (volker) Der Radsensor-Block wird nur gebaut, wenn HW_HAS_WHEEL_SPEED_SENSOR
// gesetzt ist. Ohne ihn liefert mc_interface_get_speed() das Tempo aus der
// Motordrehzahl.
#ifdef HW_HAS_WHEEL_SPEED_SENSOR
volatile float wheel_rpm_filtered = 0;
static float trip_odometer = 0;
void hw_update_speed_sensor(void) {
	static float wheel_rpm = 0;
	static uint8_t sensor_state = 0;
	static uint8_t sensor_state_old = 0;
	static float last_sensor_event_time = 0;
	float current_time = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

	sensor_state = palReadPad(HW_SPEED_SENSOR_PORT, HW_SPEED_SENSOR_PIN);

	if(sensor_state == 0 && sensor_state_old == 1 ) {
		float revolution_duration = current_time - last_sensor_event_time;

		if (revolution_duration > 0.05) {	//ignore periods <50ms
			last_sensor_event_time = current_time;
			wheel_rpm = 60.0 / revolution_duration;
			UTILS_LP_FAST(wheel_rpm_filtered, (float)wheel_rpm, 0.5);


			const volatile mc_configuration *conf = mc_interface_get_configuration();
			trip_odometer += conf->si_wheel_diameter * M_PI;
			//trip_odometer += mc_interface_get_configuration()->si_wheel_diameter * M_PI; test this
		}
	} else {
		// After 3 seconds without sensor signal, set RPM as zero
		if ( (current_time - last_sensor_event_time) > 3.0) {
			wheel_rpm_filtered = 0.0;
		}
	}
	sensor_state_old = sensor_state;
}

/* Get speed in m/s */
float hw_get_speed(void) {
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	float speed = wheel_rpm_filtered * conf->si_wheel_diameter * M_PI / 60.0;
	return speed;
}

/* Get trip distance in meters */
float hw_get_distance(void) {
	return trip_odometer;
}

float hw_get_distance_abs(void) {
	return trip_odometer;
}
#endif // HW_HAS_WHEEL_SPEED_SENSOR
