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

#define TARGET_BOARD_IDENTIFIER "MIDF3"
//#define USE_HARDWARE_REVISION_DETECTION
//#define TARGET_BUS_INIT

#define LED0                    PB3
//#define LED1                    PF0
//#define LED2                    PC15

//#undef BEEPER
#define BEEPER                  PC14
//#define BEEPER_INVERTED

#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW270_DEG
#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW270_DEG

//#define BARO
//#define USE_BARO_MS5611

#define USE_UART1//PA9;PA10(30;31)
#define USE_UART2//PA2;PA3(12;13)
//#define USE_UART3//PA10;PA11(12;13)

#define SERIAL_PORT_COUNT       2

#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15

//#define UART3_TX_PIN            PB10 
//#define UART3_RX_PIN            PB11 

#define USE_I2C
#define I2C_DEVICE             (I2CDEV_1)//PB6;PB7(42;43)
//
#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_RX_CC2500

#if defined USE_RX_CC2500//START USE_RX_CC2500
#if defined MIDELICF3V2
#define FRSKY_LED_PIN         PB4	
#elif defined MIDELICF3
#define FRSKY_LED_PIN         PA8
#endif
	
#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1

#define FRSKY_BIND//bind via CLI serial interface
/*
Bind CLI procedure
Power-up FC normallymFrslyLed blinking slowly .Connect to configurator
On cli pannel type "bind" and press enter and observe the Frsky led will come solid .
start Tx in bind  mode,the RX led will fash slowly -bind complete.
*/
//#define USE_EXTI
#define FRSKY_TELEMETRY
#define HUB
//PINOUT
#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define RX_NSS_PIN              PA4 
#define RX_SCK_PIN              PA5
#define RX_MISO_PIN             PA6
#define RX_MOSI_PIN             PA7
#define BIND_PIN                  PC13
#define GDO_0_PIN               PB0
//
#define SPI1_NSS_PIN            RX_NSS_PIN
#define SPI1_SCK_PIN            RX_SCK_PIN
#define SPI1_MISO_PIN           RX_MISO_PIN
#define SPI1_MOSI_PIN           RX_MOSI_PIN
//
#define PA_LNA//if there is amplifier chip
#define DIVERSITY//antenna switch 
//#define SWAMPING
#if defined PA_LNA
#define TX_EN_PIN              PB1
#define RX_EN_PIN              PB2
#endif
#if defined DIVERSITY
#define ANT_SEL_PIN            PB11
#endif

//#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0// ADC1
//#define EXTERNAL1_ADC_PIN       PA1//A2
#define CURRENT_METER_ADC_PIN   PA1
//
#define USE_RX_FRSKYD
#define RX_SPI_DEFAULT_PROTOCOL FRSKYD
//#define USE_RX_FRSKYX
//#define RX_SPI_DEFAULT_PROTOCOL FRSKYX
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
//#define TELEMETRY

//
#undef USE_SERVOS//
#undef TELEMETRY // to remove comment  if using HUB
#undef TELEMETRY_CRSF
#undef TELEMETRY_SRXL
#undef TELEMETRY_JETIEXBUS
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
//onboard flash
//#define USE_FLASHFS
//#define USE_FLASH_M25P16
//#define USE_SPI
//#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5
//
//#define M25P16_CS_PIN           PB12
//#define M25P16_SPI_INSTANCE     SPI2
//#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
//#define DEFAULT_FEATURES        FEATURE_BLACKBOX
//
//MOTORS
//PA2;PA3;PA12;PA11;PB8;PB9
#else
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define USE_RX_MSP
#define SPEKTRUM_BIND
#define BIND_PIN                PA3 

#endif//END USE_RX_CC2500

#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP
#define USE_QUAD_MIXER_ONLY
//#define SKIP_SERIAL_PASSTHROUGH
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
//#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))
//#define USABLE_TIMER_CHANNEL_COUNT 14
//#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))

#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) | TIM_N(17) )


