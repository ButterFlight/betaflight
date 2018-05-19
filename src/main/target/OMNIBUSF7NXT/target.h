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

#define TARGET_BOARD_IDENTIFIER "OBF7"

#define USBD_PRODUCT_STRING "OmnibusF7NXT"

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000" // Remove this at the next major release (?)
#endif

#define LED0_PIN                PB2
#define BEEPER                  PC13
//#define BEEPER_INVERTED

#define CAMERA_CONTROL_PIN  PA3

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define MPU6000_CS_PIN          PB12
#define MPU6500_CS_PIN          PA8
#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6500_SPI_INSTANCE    SPI1

// MPU6000 interrupts
//#define USE_EXTI
//#define MPU_INT_EXTI            PC4
//#define USE_MPU_DATA_READY_SIGNAL

// blue board
// MPU 6000 on SPI3, cs=PB12, ICM20608 on SPI3, CS=PA8
// --> change spi from spi1 to spi3

// OMNIBUS F7 NEXT has two IMUs - MPU6000 onboard and ICM20608 (MPU6500) in the vibration dampened box
#define GYRO_MPU6000_ALIGN      CW90_DEG
#define ACC_MPU6000_ALIGN       CW90_DEG

#define GYRO_MPU6500_ALIGN      CW90_DEG
#define ACC_MPU6500_ALIGN       CW90_DEG

#define GYRO_1_SPI_INSTANCE     MPU6000_SPI_INSTANCE
#define GYRO_2_SPI_INSTANCE     MPU6500_SPI_INSTANCE

#define USE_DUAL_GYRO
#define GYRO_0_CS_PIN           MPU6000_CS_PIN
#define GYRO_1_CS_PIN           MPU6500_CS_PIN


#define USE_BARO
#define USE_BARO_SPI_LPS
#define LPS_SPI_INSTANCE     SPI2
#define LPS_CS_PIN           PA10

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define M25P16_CS_PIN           PC14
#define M25P16_SPI_INSTANCE     SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16


#define USE_VCP
#define VBUS_SENSING_PIN        PC15

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            NONE
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA2

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            NONE

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART2, USART3, USART4, USART5, USART6

#define USE_ESCSERIAL           // RX5
#define ESCSERIAL_TIMER_TX_PIN  PD2

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7


#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PC3


#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define ADC_INSTANCE            ADC2
#define CURRENT_METER_ADC_PIN   PC1  // Direct from CRNT pad (part of onboard sensor for Pro)
#define VBAT_ADC_PIN            PC0  // 11:1 (10K + 1K) divider
#define RSSI_ADC_PIN            PC4  // Direct from RSSI pad

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART1
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define DEFAULT_FEATURES        (FEATURE_OSD)

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA (0xffff)
#define TARGET_IO_PORTB (0xffff)
#define TARGET_IO_PORTC (0xffff)
#define TARGET_IO_PORTD (0xffff)


#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12))
