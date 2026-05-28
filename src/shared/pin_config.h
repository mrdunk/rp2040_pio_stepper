#pragma once

/* Fixed RP2040 GPIO assignments for on-board peripherals.
 * Shared between firmware and driver so both can check for pin conflicts. */

#define MODBUS_TX_PIN   8
#define MODBUS_RX_PIN   9
#define MODBUS_DIR_PIN  10

#define I2C_RESET_PIN   22
#define I2C_SDA_PIN     26
#define I2C_SCL_PIN     27

/* W5500 SPI pins (mirror of port/ioLibrary_Driver/inc/w5x00_spi.h and w5x00_gpio_irq.h).
 * Duplicated here so the driver (Linux-side) can check for conflicts without
 * pulling in RP2040-specific headers. */
#define SPI_MISO_PIN    16
#define SPI_CS_PIN      17
#define SPI_SCK_PIN     18
#define SPI_MOSI_PIN    19
#define SPI_RST_PIN     20
#define SPI_INT_PIN     21

/* Other board-reserved pins (W5500-EVB-Pico).
 * GP24 is a hardware VBUS-sense input (not firmware-controlled; no conflict to detect).
 * GP25 is driven by the firmware heartbeat LED. */
#define ONBOARD_LED_PIN 25
