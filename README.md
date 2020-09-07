# ESP8266 driver for the BME280 sensor

Works only with BME280. Similar models like BMP280/BMP180 are not supported.

## Example usage
For this example to work, SDA has to be connected to GIPO13 (D7) and SCL to GPIO12 (D6).
```c
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bme280.h"

void app_main() {
  struct bme280_dev dev = {
      .addr = 0x76, .i2c_port = I2C_NUM_0, .i2c_sda_io = 13, .i2c_scl_io = 12};

  ESP_ERROR_CHECK(bme280_init(&dev));

  while (1) {
    float temp;
    float press;
    float hum;
    bme280_read(&dev, &temp, &press, &hum);
    printf("Temp: %f °C, Pressure %f hPa, Humidity: %f\n", temp, press, hum);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
```


