/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#if defined(ASGARD32F7)
#define TARGET_BOARD_IDENTIFIER "ASF7"
#endif

#if defined(ASGARD32F4)
#define USBD_PRODUCT_STRING "Asgard32"
#endif

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000" // Remove this at the next major release (?)
#endif

#define LED0_PIN                PC13
//#define LED1_PIN                PB4 // Remove this at the next major release
#define BEEPER                  PC13
//#define BEEPER_INVERTED

#define CAMERA_CONTROL_PIN	PA3

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define MPU6000_CS_PIN          PA4
#define MPU6500_CS_PIN          PC15
#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6500_SPI_INSTANCE    SPI1

#define GYRO_1_SPI_INSTANCE     MPU6000_SPI_INSTANCE
#define GYRO_2_SPI_INSTANCE     MPU6500_SPI_INSTANCE

#define USE_DUAL_GYRO
#define GYRO_0_CS_PIN           MPU6000_CS_PIN
#define GYRO_1_CS_PIN 		    MPU6500_CS_PIN

// MPU6000 interrupts
//#define USE_EXTI
//#define MPU_INT_EXTI            PC4
//#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_MPU6000_ALIGN      CW270_DEG
#define ACC_MPU6000_ALIGN       CW270_DEG
#define GYRO_MPU6500_ALIGN      CW90_DEG
#define ACC_MPU6500_ALIGN 		CW90_DEG

#define ACC_1_ALIGN				CW270_DEG
#define ACC_2_ALIGN				CW90_DEG

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI2
#define BMP280_CS_PIN 		    PB9

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN           SPI2_NSS_PIN
#define M25P16_SPI_INSTANCE     SPI2
#define USE_FLASHFS
#define USE_FLASH_M25P16


#define USE_VCP
#define VBUS_SENSING_PIN 	    PB2

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            NONE
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            NONE

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       9 //VCP, USART1, USART3, USART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8  // (Hardware=0)

#define USE_SPI
#define USE_SPI_DEVICE_1

//#if defined(OMNIBUSF4SD) || defined(LUXF4OSD)
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3
//#endif

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10 // PB10, shared with UART3TX
#define I2C2_SDA                PB11 // PB11, shared with UART3RX

#define I2C_DEVICE              (I2CDEV_2)

#define USE_ADC
#define ADC_INSTANCE            ADC2
#define CURRENT_METER_ADC_PIN   PC1  // Direct from CRNT pad (part of onboard sensor for Pro)
#define VBAT_ADC_PIN            PC0  // 11:1 (10K + 1K) divider
#define RSSI_ADC_PIN            PC4  // Direct from RSSI pad

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART1
#define SERIALRX_PROVIDER 	    SERIALRX_SBUS
#define DEFAULT_FEATURES        (FEATURE_OSD)

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff)
#define TARGET_IO_PORTD BIT(2)


#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12))
