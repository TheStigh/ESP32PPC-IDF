#include "VL53L1X_i2ccoms.h"

#include <cstring>

#include "vl53l1_error_codes.h"

#if defined(USE_ARDUINO)
#include <Wire.h>
#elif defined(USE_ESP_IDF)
#include "driver/i2c_master.h"
#endif

namespace {
constexpr int kI2CTimeoutMs = 100;

struct VL53L1XI2CConfig {
  uint8_t sda_pin{21};
  uint8_t scl_pin{22};
  uint32_t frequency_hz{400000};
  bool initialized{false};
};

VL53L1XI2CConfig g_i2c_config;

inline uint8_t to_7bit_address(uint8_t address) {
  return (address >> 1) & 0x7F;
}

#if defined(USE_ESP_IDF)
constexpr i2c_port_num_t kI2CPort = I2C_NUM_0;
i2c_master_bus_handle_t g_bus_handle = nullptr;
i2c_master_dev_handle_t g_device_handles[128] = {nullptr};

esp_err_t get_device_handle(uint8_t address_7bit, i2c_master_dev_handle_t *out_handle) {
  if (address_7bit >= 128 || g_bus_handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  auto &cached = g_device_handles[address_7bit];
  if (cached == nullptr) {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address_7bit;
    dev_cfg.scl_speed_hz = g_i2c_config.frequency_hz;
    dev_cfg.scl_wait_us = 0;
    dev_cfg.flags.disable_ack_check = 0;

    esp_err_t err = i2c_master_bus_add_device(g_bus_handle, &dev_cfg, &cached);
    if (err != ESP_OK) {
      return err;
    }
  }

  *out_handle = cached;
  return ESP_OK;
}
#endif

}  // namespace

void vl53l1x_configure_i2c(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency_hz) {
  bool changed = (g_i2c_config.sda_pin != sda_pin) || (g_i2c_config.scl_pin != scl_pin) ||
                 (g_i2c_config.frequency_hz != frequency_hz);
  g_i2c_config.sda_pin = sda_pin;
  g_i2c_config.scl_pin = scl_pin;
  g_i2c_config.frequency_hz = frequency_hz;
  if (changed) {
    g_i2c_config.initialized = false;
#if defined(USE_ESP_IDF)
    g_bus_handle = nullptr;
    for (auto &handle : g_device_handles) {
      handle = nullptr;
    }
#endif
  }
}

int8_t i2c_init() {
  if (g_i2c_config.initialized) {
    return VL53L1_ERROR_NONE;
  }

#if defined(USE_ARDUINO)
  if (!Wire.begin(g_i2c_config.sda_pin, g_i2c_config.scl_pin, g_i2c_config.frequency_hz)) {
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }
  g_i2c_config.initialized = true;
  return VL53L1_ERROR_NONE;
#elif defined(USE_ESP_IDF)
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = kI2CPort;
  bus_cfg.sda_io_num = static_cast<gpio_num_t>(g_i2c_config.sda_pin);
  bus_cfg.scl_io_num = static_cast<gpio_num_t>(g_i2c_config.scl_pin);
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.intr_priority = 0;
  bus_cfg.trans_queue_depth = 0;
  bus_cfg.flags.enable_internal_pullup = 1;
  bus_cfg.flags.allow_pd = 0;

  esp_err_t err = i2c_new_master_bus(&bus_cfg, &g_bus_handle);
  if (err == ESP_ERR_INVALID_STATE) {
    err = i2c_master_get_bus_handle(kI2CPort, &g_bus_handle);
  }
  if (err != ESP_OK || g_bus_handle == nullptr) {
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  g_i2c_config.initialized = true;
  return VL53L1_ERROR_NONE;
#else
  return VL53L1_ERROR_CONTROL_INTERFACE;
#endif
}

int8_t i2c_write_multi(uint8_t deviceAddress, uint16_t registerAddress, uint8_t *pdata, uint32_t count) {
  if (count > 64) {
    return VL53L1_ERROR_INVALID_PARAMS;
  }
#if defined(USE_ARDUINO)
  const uint8_t address = to_7bit_address(deviceAddress);

  Wire.beginTransmission(address);
  const uint8_t reg[2] = {
      static_cast<uint8_t>(registerAddress >> 8),
      static_cast<uint8_t>(registerAddress & 0xFF),
  };
  if (Wire.write(reg, sizeof(reg)) != sizeof(reg)) {
    Wire.endTransmission();
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  while (count-- > 0) {
    if (Wire.write(*pdata++) != 1) {
      Wire.endTransmission();
      return VL53L1_ERROR_CONTROL_INTERFACE;
    }
  }

  return (Wire.endTransmission() == 0) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
#elif defined(USE_ESP_IDF)
  i2c_master_dev_handle_t dev = nullptr;
  if (get_device_handle(to_7bit_address(deviceAddress), &dev) != ESP_OK) {
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  uint8_t tx_buffer[66] = {
      static_cast<uint8_t>(registerAddress >> 8),
      static_cast<uint8_t>(registerAddress & 0xFF),
  };

  if (count > 0) {
    std::memcpy(&tx_buffer[2], pdata, count);
  }

  esp_err_t err = i2c_master_transmit(dev, tx_buffer, count + 2, kI2CTimeoutMs);
  return (err == ESP_OK) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
#else
  return VL53L1_ERROR_CONTROL_INTERFACE;
#endif
}

int8_t i2c_read_multi(uint8_t deviceAddress, uint16_t registerAddress, uint8_t *pdata, uint32_t count) {
  if (count > 64) {
    return VL53L1_ERROR_INVALID_PARAMS;
  }
#if defined(USE_ARDUINO)
  const uint8_t address = to_7bit_address(deviceAddress);

  Wire.beginTransmission(address);
  const uint8_t reg[2] = {
      static_cast<uint8_t>(registerAddress >> 8),
      static_cast<uint8_t>(registerAddress & 0xFF),
  };
  if (Wire.write(reg, sizeof(reg)) != sizeof(reg)) {
    Wire.endTransmission();
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  if (Wire.endTransmission(false) != 0) {
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  const uint8_t requested = static_cast<uint8_t>(count);
  if (Wire.requestFrom(address, requested, true) != requested) {
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  while (count-- > 0) {
    if (!Wire.available()) {
      return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    *pdata++ = static_cast<uint8_t>(Wire.read());
  }

  return VL53L1_ERROR_NONE;
#elif defined(USE_ESP_IDF)
  i2c_master_dev_handle_t dev = nullptr;
  if (get_device_handle(to_7bit_address(deviceAddress), &dev) != ESP_OK) {
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  uint8_t reg[2] = {
      static_cast<uint8_t>(registerAddress >> 8),
      static_cast<uint8_t>(registerAddress & 0xFF),
  };

  esp_err_t err = i2c_master_transmit_receive(
      dev, reg, sizeof(reg), pdata, count, kI2CTimeoutMs);
  return (err == ESP_OK) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
#else
  return VL53L1_ERROR_CONTROL_INTERFACE;
#endif
}

int8_t i2c_write_byte(uint8_t deviceAddress, uint16_t registerAddress, uint8_t data) {
  return i2c_write_multi(deviceAddress, registerAddress, &data, 1);
}

int8_t i2c_write_word(uint8_t deviceAddress, uint16_t registerAddress, uint16_t data) {
  uint8_t buff[2];
  buff[1] = data & 0xFF;
  buff[0] = data >> 8;
  return i2c_write_multi(deviceAddress, registerAddress, buff, 2);
}

int8_t i2c_write_Dword(uint8_t deviceAddress, uint16_t registerAddress, uint32_t data) {
  uint8_t buff[4];

  buff[3] = data & 0xFF;
  buff[2] = data >> 8;
  buff[1] = data >> 16;
  buff[0] = data >> 24;

  return i2c_write_multi(deviceAddress, registerAddress, buff, 4);
}

int8_t i2c_read_byte(uint8_t deviceAddress, uint16_t registerAddress, uint8_t *data) {
  return i2c_read_multi(deviceAddress, registerAddress, data, 1);
}

int8_t i2c_read_word(uint8_t deviceAddress, uint16_t registerAddress, uint16_t *data) {
  uint8_t buff[2];
  int8_t r = i2c_read_multi(deviceAddress, registerAddress, buff, 2);

  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  *data = tmp;

  return r;
}

int8_t i2c_read_Dword(uint8_t deviceAddress, uint16_t registerAddress, uint32_t *data) {
  uint8_t buff[4];
  int8_t r = i2c_read_multi(deviceAddress, registerAddress, buff, 4);

  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];

  *data = tmp;

  return r;
}