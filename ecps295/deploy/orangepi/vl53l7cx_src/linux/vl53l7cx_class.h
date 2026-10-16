/* Minimal C++ class around the unchanged ST ULD sources (vl53l7cx_api.cpp from stm32duino/VL53L7CX, BSD-3). */
#ifndef __VL53L7CX_CLASS_H
#define __VL53L7CX_CLASS_H
#include <string.h>
#include "vl53l7cx_platform.h"
#include "vl53l7cx_api.h"

class VL53L7CX {
 public:
  VL53L7CX(int fd, uint16_t addr8) {
    memset((void *)&_dev, 0, sizeof(VL53L7CX_Configuration));
    _dev.platform.fd = fd;
    _dev.platform.address = addr8;
    p_dev = &_dev;
  }
  uint8_t vl53l7cx_is_alive(uint8_t *p_is_alive);
  uint8_t vl53l7cx_init();
  uint8_t vl53l7cx_set_i2c_address(uint16_t i2c_address);
  uint8_t vl53l7cx_get_power_mode(uint8_t *p_power_mode);
  uint8_t vl53l7cx_set_power_mode(uint8_t power_mode);
  uint8_t vl53l7cx_start_ranging();
  uint8_t vl53l7cx_stop_ranging();
  uint8_t vl53l7cx_check_data_ready(uint8_t *p_isReady);
  uint8_t vl53l7cx_get_ranging_data(VL53L7CX_ResultsData *p_results);
  uint8_t vl53l7cx_get_resolution(uint8_t *p_resolution);
  uint8_t vl53l7cx_set_resolution(uint8_t resolution);
  uint8_t vl53l7cx_get_ranging_frequency_hz(uint8_t *p_frequency_hz);
  uint8_t vl53l7cx_set_ranging_frequency_hz(uint8_t frequency_hz);
  uint8_t vl53l7cx_get_integration_time_ms(uint32_t *p_time_ms);
  uint8_t vl53l7cx_set_integration_time_ms(uint32_t integration_time_ms);
  uint8_t vl53l7cx_get_sharpener_percent(uint8_t *p_sharpener_percent);
  uint8_t vl53l7cx_set_sharpener_percent(uint8_t sharpener_percent);
  uint8_t vl53l7cx_get_target_order(uint8_t *p_target_order);
  uint8_t vl53l7cx_set_target_order(uint8_t target_order);
  uint8_t vl53l7cx_get_ranging_mode(uint8_t *p_ranging_mode);
  uint8_t vl53l7cx_set_ranging_mode(uint8_t ranging_mode);
  uint8_t vl53l7cx_dci_read_data(uint8_t *data, uint32_t index, uint16_t data_size);
  uint8_t vl53l7cx_dci_write_data(uint8_t *data, uint32_t index, uint16_t data_size);
  uint8_t vl53l7cx_dci_replace_data(uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data,
                                    uint16_t new_data_size, uint16_t new_data_pos);
  void SwapBuffer(uint8_t *buffer, uint16_t size);
  uint8_t get_stream_count(void) { return _dev.streamcount; }

 protected:
  uint8_t _vl53l7cx_poll_for_answer(uint8_t size, uint8_t pos, uint16_t address, uint8_t mask, uint8_t expected_value);
  uint8_t _vl53l7cx_poll_for_mcu_boot();
  uint8_t _vl53l7cx_send_offset_data(uint8_t resolution);
  uint8_t _vl53l7cx_send_xtalk_data(uint8_t resolution);
  uint8_t RdByte(VL53L7CX_Platform *p, uint16_t reg, uint8_t *v);
  uint8_t WrByte(VL53L7CX_Platform *p, uint16_t reg, uint8_t v);
  uint8_t RdMulti(VL53L7CX_Platform *p, uint16_t reg, uint8_t *v, uint32_t size);
  uint8_t WrMulti(VL53L7CX_Platform *p, uint16_t reg, uint8_t *v, uint32_t size);
  uint8_t WaitMs(VL53L7CX_Platform *p, uint32_t ms);
  VL53L7CX_Configuration _dev;
  VL53L7CX_Configuration *p_dev;
};
#endif
