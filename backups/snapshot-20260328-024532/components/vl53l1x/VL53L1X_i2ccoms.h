/**
 * @file  vl53l1_i2ccoms.h
 * @brief Contains i2c implementation of the platform
 */

#ifndef _VL53L1X_I2CCOMS_H_
#define _VL53L1X_I2CCOMS_H_

#include <stdint.h>

#include "vl53l1_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Configure I2C pins and bus frequency used by the ULD transport layer. */
void vl53l1x_configure_i2c(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency_hz);

/** @brief i2c_init() definition. */
int8_t i2c_init();

/** @brief i2c_write_multi() definition.
 * To be implemented by the developer
 */
int8_t i2c_write_multi(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint8_t      *pdata,
        uint32_t      count);
/** @brief i2c_read_multi() definition.
 * To be implemented by the developer
 */
int8_t i2c_read_multi(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint8_t      *pdata,
        uint32_t      count);
/** @brief i2c_write_byte() definition.
 * To be implemented by the developer
 */
int8_t i2c_write_byte(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint8_t       data);
/** @brief i2c_write_word() definition.
 * To be implemented by the developer
 */
int8_t i2c_write_word(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint16_t      data);
/** @brief i2c_write_Dword() definition.
 * To be implemented by the developer
 */
int8_t i2c_write_Dword(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint32_t      data);
/** @brief i2c_read_byte() definition.
 * To be implemented by the developer
 */
int8_t i2c_read_byte(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint8_t      *pdata);
/** @brief i2c_read_word() definition.
 * To be implemented by the developer
 */
int8_t i2c_read_word(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint16_t     *pdata);
/** @brief i2c_read_Dword() definition.
 * To be implemented by the developer
 */
int8_t i2c_read_Dword(
        uint8_t       deviceAddress,
        uint16_t      registerAddress,
        uint32_t     *pdata);

#ifdef __cplusplus
}
#endif

#endif