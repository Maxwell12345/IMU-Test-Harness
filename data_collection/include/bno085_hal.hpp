#pragma once

extern "C" {
#include <sh2.h>
#include "sh2_err.h"
}


/**
 * @brief Constructs a Linux I2C HAL for the BNO085 on /dev/i2c-1 at address 0x4A.
 *
 * The returned sh2_Hal_t is populated with function pointers that implement
 * the SHTP-over-I2C transport required by the CEVA SH2 driver.  Pass the
 * result directly to sh2_open().
 *
 * @return sh2_Hal_t A fully-initialised HAL struct ready for use with sh2_open().
 * @throws Nothing — errors surface as SH2_ERR return codes inside the HAL callbacks.
 */
sh2_Hal_t bno085_hal_create();
