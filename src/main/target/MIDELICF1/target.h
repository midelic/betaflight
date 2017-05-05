/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once


//#define USE_HARDWARE_REVISION_DETECTION
#define TARGET_BUS_INIT

#if defined (MIDELICF1)
#define TARGET_BOARD_IDENTIFIER "MID"
#else
#define TARGET_BOARD_IDENTIFIER "MBR"
#endif

//#define LED0                    PC14
//#define LED1                    PC13
#define LED0                    PB4
#define LED1                    PB3
//#define LED2                    PC15

#undef BEEPER

#define GYRO
#define USE_GYRO_MPU6050

#define ACC
#define USE_ACC_MPU6050

//#define BARO
//#define USE_BARO_MS5611
//#define USE_BARO_BMP085
//#define USE_BARO_BMP280


#define USE_UART1//PA9;PA10(30;31)
#define USE_UART2//PA2;PA3(12;13)

#define SERIAL_PORT_COUNT       2

//#define INVERTER                PB2 // PB2 (BOOT1) abused as inverter select GPIO
//#define INVERTER_USART          USART2

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_2)//PB10;PB11(21;22)
#define USE_SPI
#define USE_SPI_DEVICE_2
//#define USE_RX_SOFTSPI

#define USE_RX_CC2500

#if defined USE_RX_CC2500//START USE_RX_CC2500
	
//
#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI2

#define FRSKY_BIND//bind via CLI serial interface
/*
Bind CLI procedure
Power-up FC normallymFrslyLed blinking slowly .Connect to configurator
On cli pannel type "bind" and press enter pbserve the Frsky led will come solid.
start Tx in bind  mode,the RX led will fash slowly -bind complete.
*/
//#define USE_EXTI
#define FRSKY_TELEMETRY
#define HUB
//PINOUT
#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOB
#define RX_NSS_PIN              PB12//25
#define RX_SCK_PIN              PB13//26
#define RX_MISO_PIN             PB14//27
#define RX_MOSI_PIN             PB15//28
//
#define BIND_PIN                   PA3//13//USART2 RX
#define GDO_0_PIN                 PB0//18
#define FRSKY_LED_PIN           PB1//19 used maple board led for test
//
#define SPI2_NSS_PIN            RX_NSS_PIN
#define SPI2_SCK_PIN            RX_SCK_PIN
#define SPI2_MISO_PIN           RX_MISO_PIN
#define SPI2_MOSI_PIN           RX_MOSI_PIN
//
#define PA_LNA//If there is CC2500 with amplifier chip
#define DIVERSITY

#if defined PA_LNA
//#define SWAMPING	
#define TX_EN_PIN              PA0//10
#define RX_EN_PIN              PA1//11
#endif
#if defined DIVERSITY
#define ANT_SEL_PIN            PB5//41
#endif

//#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define VBAT_ADC_PIN                     PA4//14 A1/ Frsky
#define EXTERNAL1_ADC_PIN            PA5//15 A2 /SCK1
#define RSSI_ADC_PIN                     PA6//16 MISO1
#define CURRENT_METER_ADC_PIN    PA7//17 MOSI1

//
#define USE_RX_FRSKYD
#define RX_SPI_DEFAULT_PROTOCOL FRSKYD
//#define USE_RX_FRSKYX
//#define RX_SPI_DEFAULT_PROTOCOL FRSKYX
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI

#if defined MBRAIN	
#define TARGET_CONFIG
#define BRUSHED_MOTORS
#define DEFAULT_FEATURES FEATURE_MOTOR_STOP
#define SKIP_SERIAL_PASSTHROUGH
#define USE_QUAD_MIXER_ONLY
#if defined PA_LNA
#undef 	PA_LNA//No PA_LNA
#undef DIVERSITY
#undef SWAMPING
#endif
#endif

#undef USE_SERVOS//has errors
//#undef USE_CLI//good
#undef BLACKBOX
//#undef TELEMETRY
//#undef BARO
//
#ifdef USE_PWM
#undef USE_PWM
#endif
#ifdef USE_PPM
#undef USE_PPM
#endif
#ifdef SERIAL_RX
#undef SERIAL_RX
#endif
#if (FLASH_SIZE <= 64)
#undef BLACKBOX
#undef TELEMETRY
#undef USE_SERVOS//has errors
#undef USE_CLI//
#endif
//
#if !defined(BRUSHED_MOTORS)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif
#undef TELEMETRY_HOTT
#undef TELEMETRY_LTM
#undef TELEMETRY_SMARTPORT

//MOTORS
//PA8;PA11;PB6;PB7;PB8;PB9
#else
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define USE_RX_MSP
#define SPEKTRUM_BIND
#define BIND_PIN                PA3 

#endif//END USE_RX_CC2500

#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP | FEATURE_TELEMETRY //| FEATURE_VBAT

// Number of available PWM outputs
//#define MAX_PWM_OUTPUT_PORTS    4
// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))




