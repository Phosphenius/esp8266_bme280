/*
 * esp8266_bme280  - a bme280 driver for the esp8266
 * Copyright (C) 2020 Luca Kredel
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "bme280.h"

#define CHIP_ID 0x60
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ

static const char *TAG = "bme280";

struct bme280_calib {
  uint16_t T1;
  int16_t T2;
  int16_t T3;
  uint16_t P1;
  int16_t P2;
  int16_t P3;
  int16_t P4;
  int16_t P5;
  int16_t P6;
  int16_t P7;
  int16_t P8;
  int16_t P9;
  uint8_t H1;
  int16_t H2;
  uint8_t H3;
  int16_t H4;
  int16_t H5;
  int8_t H6;
};

static esp_err_t bme280_i2c_read(struct bme280_dev *dev, uint8_t reg_addr,
                                 uint8_t *data, size_t data_len);
static esp_err_t bme280_i2c_write_byte(struct bme280_dev *dev, uint8_t reg_addr,
                                       uint8_t data);
static int32_t bme280_compensate_t(struct bme280_calib *calib, int32_t adc_t);
static uint32_t bme280_compensate_p(struct bme280_calib *calib, int32_t adc_p);
static uint32_t bme280_compensate_h(struct bme280_calib *calib, int32_t adc_h);

struct bme280_calib calib;
int32_t t_fine;

esp_err_t bme280_init(struct bme280_dev *dev) {
  int i2c_master_port = dev->i2c_port;

  i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                       .sda_io_num = dev->i2c_sda_io,
                       .sda_pullup_en = 0,
                       .scl_io_num = dev->i2c_scl_io,
                       .scl_pullup_en = 0,
                       .clk_stretch_tick = 300};

  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));

  uint8_t id = 0;
  int timeout = 0;

  while (bme280_i2c_read(dev, 0xD0, &id, 1) != ESP_OK) {
    vTaskDelay(100 / portTICK_RATE_MS);
    if (++timeout >= 5) {
      ESP_LOGE(TAG, "Unable to connect to sensor\n");
      return ESP_FAIL;
    }
  }

  if (id != 0x60) {
    ESP_LOGE(TAG, "No sensor found\n");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Sensor found at address 0x%x\n", dev->addr);

  uint8_t calib_buf[24];
  ESP_ERROR_CHECK(bme280_i2c_read(dev, 0x88, calib_buf, 24));

  calib.T1 = (uint16_t)(calib_buf[1] << 8) | calib_buf[0];
  calib.T2 = (int16_t)((calib_buf[3] << 8) | (calib_buf[2]));
  calib.T3 = (int16_t)((calib_buf[5] << 8) | (calib_buf[4]));

  calib.P1 = (uint16_t)(calib_buf[7] << 8) | calib_buf[6];
  calib.P2 = (int16_t)(calib_buf[9] << 8) | calib_buf[8];
  calib.P3 = (int16_t)(calib_buf[11] << 8) | calib_buf[10];
  calib.P4 = (int16_t)(calib_buf[13] << 8) | calib_buf[12];
  calib.P5 = (int16_t)(calib_buf[15] << 8) | calib_buf[14];
  calib.P6 = (int16_t)(calib_buf[17] << 9) | calib_buf[16];
  calib.P7 = (int16_t)(calib_buf[19] << 8) | calib_buf[18];
  calib.P8 = (int16_t)(calib_buf[21] << 8) | calib_buf[20];
  calib.P9 = (int16_t)(calib_buf[23] << 8) | calib_buf[22];

  uint8_t h1;
  ESP_ERROR_CHECK(bme280_i2c_read(dev, 0xA1, &h1, 1));
  calib.H1 = h1;

  uint8_t h_buf[2];
  ESP_ERROR_CHECK(bme280_i2c_read(dev, 0xE1, h_buf, 2));
  calib.H2 = (int16_t)(h_buf[1] << 8) | h_buf[0];

  uint8_t h3;
  ESP_ERROR_CHECK(bme280_i2c_read(dev, 0xE3, &h3, 1));
  calib.H3 = h3;

  uint8_t h45_buf[3];
  ESP_ERROR_CHECK(bme280_i2c_read(dev, 0xE4, h45_buf, 3));

  calib.H4 = (h45_buf[0] << 4) | (h45_buf[1] & 0x0F);
  calib.H5 = (h45_buf[2] << 4) | (h45_buf[1] & 0xF0);

  uint8_t h6;
  ESP_ERROR_CHECK(bme280_i2c_read(dev, 0xE7, &h6, 1));
  calib.H6 = h6;

  ESP_ERROR_CHECK(bme280_i2c_write_byte(dev, 0xF2, 0x1));
  ESP_ERROR_CHECK(bme280_i2c_write_byte(dev, 0xF4, 0x27));

  return ESP_OK;
}

esp_err_t bme280_read(struct bme280_dev *dev, float *temp, float *press,
                      float *hum) {
  uint8_t buf[8];
  uint32_t t = 0;
  uint32_t p = 0;
  uint32_t h = 0;
  ESP_ERROR_CHECK(bme280_i2c_read(dev, 0xF7, buf, 8));

  t |= (buf[3] << 16) | (buf[4] << 8) | buf[5];
  t >>= 4;

  p |= (buf[0] << 16) | (buf[1] << 8) | buf[2];
  p >>= 4;

  h |= (buf[6] << 8) | buf[7];

  *temp = bme280_compensate_t(&calib, t) / 100.0f;
  *press = (bme280_compensate_p(&calib, p) / 256000.0f);
  *hum = bme280_compensate_h(&calib, h) / 1024.0f;

  return ESP_OK;
}

static esp_err_t bme280_i2c_read(struct bme280_dev *dev, uint8_t reg_addr,
                                 uint8_t *data, size_t data_len) {
  int ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, dev->addr << 1 | WRITE_BIT, 1);
  i2c_master_write_byte(cmd, reg_addr, 1);
  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, dev->addr << 1 | READ_BIT, 1);
  i2c_master_read(cmd, data, data_len, 2);
  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

static esp_err_t bme280_i2c_write_byte(struct bme280_dev *dev, uint8_t reg_addr,
                                       uint8_t data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, dev->addr << 1 | WRITE_BIT, 1);
  i2c_master_write_byte(cmd, reg_addr, 1);
  i2c_master_write_byte(cmd, data, 1);
  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

static int32_t bme280_compensate_t(struct bme280_calib *calib, int32_t adc_t) {
  int32_t var1;
  int32_t var2;
  int32_t t;

  var1 =
      ((((adc_t >> 3) - ((int32_t)calib->T1 << 1))) * ((int32_t)calib->T2)) >>
      11;
  var2 = (((((adc_t >> 4) - ((int32_t)calib->T1)) *
            ((adc_t >> 4) - ((int32_t)calib->T1))) >>
           12) *
          ((int32_t)calib->T3)) >>
         14;
  t_fine = var1 + var2;
  t = (t_fine * 5 + 128) >> 8;

  return t;
}

static uint32_t bme280_compensate_p(struct bme280_calib *calib, int32_t adc_p) {
  int64_t var1;
  int64_t var2;
  int64_t p;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib->P6;
  var2 = var2 + ((var1 * (int64_t)calib->P5) << 17);
  var2 = var2 + (((int64_t)calib->P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib->P3) >> 8) +
         ((var1 * (int64_t)calib->P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->P1) >> 33;

  if (var1 == 0) {
    return 0;
  }

  p = 1048576 - adc_p;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib->P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)calib->P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib->P7) << 4);

  return (uint32_t)p;
}

static uint32_t bme280_compensate_h(struct bme280_calib *calib, int32_t adc_h) {
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r =
      (((((adc_h << 14) - (((int32_t)calib->H4) << 20) -
          (((int32_t)calib->H5) * v_x1_u32r)) +
         ((int32_t)16384)) >>
        15) *
       (((((((v_x1_u32r * ((int32_t)calib->H6)) >> 10) *
            (((v_x1_u32r * ((int32_t)calib->H3)) >> 11) + ((int32_t)32786))) >>
           10) +
          ((int32_t)2097152)) *
             ((int32_t)calib->H2) +
         8192) >>
        14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)calib->H1)) >>
                            4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r >> 12);
}
