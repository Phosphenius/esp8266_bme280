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

#include <stdlib.h>

#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

struct bme280_dev {
  uint8_t addr;
  i2c_port_t i2c_port;
  gpio_num_t i2c_sda_io;
  gpio_num_t i2c_scl_io;
};

esp_err_t bme280_init(struct bme280_dev *dev);
esp_err_t bme280_read(struct bme280_dev *dev, float *temp, float *press, float *hum);
