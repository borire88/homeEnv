#include <stdint.h>
#include "esp_err.h"
typedef enum {
    BME280_PRIMARY,
    BME280_SECONDARY,
    BME280_NUM_OF_DEVICES
}BME280_DEVICE;

esp_err_t bme280__intitInterface(uint8_t i2c, uint8_t sdaPin, uint8_t sclPin);
esp_err_t bme280__init(BME280_DEVICE dev);
void bme280__configure(_Bool temp, _Bool pressure, _Bool humidity);
void bme280__requestMesurements(BME280_DEVICE dev);
int32_t bme280__getTemperature(BME280_DEVICE device);
uint32_t bme280__getPressure(BME280_DEVICE device);
uint32_t bme280__getHumidity(BME280_DEVICE device);