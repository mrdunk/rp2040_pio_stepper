#pragma once

/* Fixed RP2040 GPIO assignments for on-board peripherals.
 * Shared between firmware and driver so both can check for pin conflicts. */

#define MODBUS_TX_PIN   8
#define MODBUS_RX_PIN   9
#define MODBUS_DIR_PIN  10

#define I2C_RESET_PIN   22
#define I2C_SDA_PIN     26
#define I2C_SCL_PIN     27
