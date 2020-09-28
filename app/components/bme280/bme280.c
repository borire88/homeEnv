#include "bme280.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <string.h>

#define MAX_NUM_OF_BME280                        (uint32_t)(2)

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM                      UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC                       UINT8_C(0x77)

#define BME280_ADDRESS                            (uint8_t)(0x76)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID                            UINT8_C(0x60)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR                       UINT8_C(0xD0)
#define BME280_RESET_ADDR                         UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR         UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR           UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR                      UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR                      UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR                     UINT8_C(0xF4)
#define BME280_CONFIG_ADDR                        UINT8_C(0xF5)
#define BME280_DATA_ADDR                          UINT8_C(0xF7)


// Private types
/**
 * @
 */
typedef struct {
    uint8_t press_msb;
    uint8_t press_lsb;
    uint8_t press_xlsb;
    uint8_t temp_msb;
    uint8_t temp_lsb;
    uint8_t temp_xlsb;
    uint8_t hum_msb;
    uint8_t hum_lsb;
}__packed BME280_MEAS_REGS;

typedef struct {
    uint8_t mode : 2;
    uint8_t osamp_p : 3;
    uint8_t osamp_t : 3;
    uint8_t spi3w : 1;
    uint8_t reserved : 1;
    uint8_t filter : 3;
    uint8_t t_sb : 3;
}__packed BME280_CTRL_REGS;

typedef struct 
{
    uint8_t osamp_h : 3;
    uint8_t reserved : 5;
}__packed BME280_HUM_CTRL;

typedef uint8_t RESET_REG;

typedef struct {
    uint16_t dig_T1;    
    int16_t dig_T2;     
    int16_t dig_T3;     
    uint16_t dig_P1;    
    int16_t dig_P2;     
    int16_t dig_P3;     
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} __packed BME280_CALIB;

typedef struct {
    uint32_t pressure;
    int32_t temperature;
    uint32_t humidity;
}BME_COMPENSTATED_MEASUREMENTS;

/* Private variables */
static int32_t T_Fine;
static BME280_CALIB CalibrationData[BME280_NUM_OF_DEVICES]; 
static BME_COMPENSTATED_MEASUREMENTS CompensatedMeasurements[BME280_NUM_OF_DEVICES];
static uint8_t CalibRead[33];
static const uint8_t Bme280Addresses[BME280_NUM_OF_DEVICES] = {0x76, 0x77};
static uint8_t I2C_MODULE = 0xFF;

/* Private functions declaration */
static int32_t compensateTemperature(int32_t notCompesatedTemp, BME280_CALIB* calibData);
static uint32_t compensatePressure(int32_t notCompesatedPressure,  BME280_CALIB* calibData);
static uint32_t compensateHumidity(int32_t notCompesatedHumidity, BME280_CALIB* calibData);
static void setRegister(uint8_t devAddress, uint8_t registerAddress);
static void writeRegister(uint8_t devAddress, uint8_t registerAddress, uint8_t value);
static void readRegister(uint8_t devAddress, uint8_t* destination, uint8_t length);

/* Public functions definition */

esp_err_t bme280__intitInterface(uint8_t i2c, uint8_t sdaPin, uint8_t sclPin) {
    I2C_MODULE = i2c;
    esp_err_t error;
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .master.clk_speed = 100000,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = sclPin,
        .sda_io_num = sdaPin
    };
    error = i2c_param_config(i2c, &config);
    error |= i2c_driver_install(i2c, I2C_MODE_MASTER, 0, 0, 0);
    return error;
}

esp_err_t bme280__init(BME280_DEVICE dev) {
    if(dev < BME280_NUM_OF_DEVICES) {
        memset(CalibRead, 0x00, sizeof(CalibRead));

        //read first part of comensation parameters.
        setRegister(Bme280Addresses[dev], 0x88);
        readRegister(Bme280Addresses[dev], CalibRead, 24);
        //read second part of comensation parameters.
        setRegister(Bme280Addresses[dev], 0xA1);
        readRegister(Bme280Addresses[dev], CalibRead + 24, 1);
        //read third part of
        setRegister(Bme280Addresses[dev], 0xE1);
        readRegister(Bme280Addresses[dev], CalibRead + 25, 7);
        //set config register
        writeRegister(Bme280Addresses[dev], 0xF5, 0xA0);
        // Set humidity control register
        writeRegister(Bme280Addresses[dev], 0xF2, 0x01);
        // Set temperature and pressure control register
        writeRegister(Bme280Addresses[dev], 0xF4, 0x27);

        CalibrationData[dev].dig_T1 = (((uint16_t)CalibRead[1]) << 8) | (uint16_t)CalibRead[0];    
        CalibrationData[dev].dig_T2 = (int16_t)((((uint16_t)CalibRead[3]) << 8) | (uint16_t)CalibRead[2]);     
        CalibrationData[dev].dig_T3 = (int16_t)((((uint16_t)CalibRead[5]) << 8) | (uint16_t)CalibRead[4]);     
        CalibrationData[dev].dig_P1 = (uint16_t)((((uint16_t)CalibRead[7]) << 8) | (uint16_t)CalibRead[6]);    
        CalibrationData[dev].dig_P2 = (int16_t)((((uint16_t)CalibRead[9]) << 8) | (uint16_t)CalibRead[8]);     
        CalibrationData[dev].dig_P3 = (int16_t)((((uint16_t)CalibRead[11]) << 8) | (uint16_t)CalibRead[10]);     
        CalibrationData[dev].dig_P4 = (int16_t)((((uint16_t)CalibRead[13]) << 8) | (uint16_t)CalibRead[12]);
        CalibrationData[dev].dig_P5 = (int16_t)((((uint16_t)CalibRead[15]) << 8) | (uint16_t)CalibRead[14]);
        CalibrationData[dev].dig_P6 = (int16_t)((((uint16_t)CalibRead[17]) << 8) | (uint16_t)CalibRead[16]);
        CalibrationData[dev].dig_P7 = (int16_t)((((uint16_t)CalibRead[19]) << 8) | (uint16_t)CalibRead[18]);
        CalibrationData[dev].dig_P8 = (int16_t)((((uint16_t)CalibRead[21]) << 8) | (uint16_t)CalibRead[20]);
        CalibrationData[dev].dig_P9 = (int16_t)((((uint16_t)CalibRead[23]) << 8) | (uint16_t)CalibRead[22]);
        CalibrationData[dev].dig_H1 = CalibRead[24];
        CalibrationData[dev].dig_H2 = (int16_t)((((uint16_t)CalibRead[26]) << 8) | (uint16_t)CalibRead[25]);
        CalibrationData[dev].dig_H3 = CalibRead[27];
        CalibrationData[dev].dig_H4 = (int16_t)((((int16_t)CalibRead[28]) << 4) | (((uint16_t)CalibRead[29]) & 0x000F));
        CalibrationData[dev].dig_H5 = (int16_t)((((uint16_t)CalibRead[30]) << 4) | ((((uint16_t)CalibRead[29]) >> 4) & 0x000F));
        CalibrationData[dev].dig_H6 = (int8_t)(CalibRead[31]);
    }
    return 0;
}

int32_t bme280__getTemperature(BME280_DEVICE device) {
    return CompensatedMeasurements[device].temperature;
}
uint32_t bme280__getPressure(BME280_DEVICE device) {
    return CompensatedMeasurements[device].pressure;
}
uint32_t bme280__getHumidity(BME280_DEVICE device) {
    return CompensatedMeasurements[device].humidity;
}

void bme280__requestMesurements(BME280_DEVICE dev) {
    uint8_t measurements[8];
    setRegister(Bme280Addresses[dev], 0xF7);
    readRegister(Bme280Addresses[dev], measurements, 8);
    
    int32_t convertedTemp = (((uint32_t)measurements[3]) << 16) | (((uint32_t)measurements[4]) << 8) | ((uint32_t)measurements[5]);
    convertedTemp >>= 4;
    int32_t convertedPress = (((uint32_t)measurements[0]) << 16) | (((uint32_t)measurements[1]) << 8) | ((uint32_t)measurements[2]);
    convertedPress >>= 4;
    int32_t convertedHumidity = (((uint32_t)measurements[6]) << 8) | ((uint32_t)measurements[7]);

    CompensatedMeasurements[dev].temperature = compensateTemperature(convertedTemp, &(CalibrationData[dev]));
    CompensatedMeasurements[dev].pressure = compensatePressure(convertedPress, &(CalibrationData[dev]));
    CompensatedMeasurements[dev].humidity = compensateHumidity(convertedHumidity, &(CalibrationData[dev]));
}

/* Private functions definitions */
static int32_t compensateTemperature(int32_t notCompesatedTemp, BME280_CALIB* calibData) {
    int32_t var1, var2, var3, T;
    var1 = notCompesatedTemp >> 3;
    var1 -= ((int32_t)(calibData->dig_T1)) << 1;
    var1 *= (int32_t)(calibData->dig_T2);
    var1 >>= 11;
    var2 = notCompesatedTemp >> 4;
    var2 -= (int32_t)(calibData->dig_T1);
    var3 = notCompesatedTemp >> 4;
    var3 -= (int32_t)(calibData->dig_T1);
    var2 *= var3;
    var2 >>= 12;
    var2 *= (int32_t)(calibData->dig_T3);
    var2 >>= 14;
    T_Fine = var1 + var2;
    T = (T_Fine * 5 + 128) >> 8;
    return T;
}
static uint32_t compensatePressure(int32_t notCompesatedPressure,  BME280_CALIB* calibData) {
    int64_t var1, var2, p;
    var1 = ((int64_t)T_Fine) - 128000;
    var2 = var1 * var1 * (int64_t)(calibData->dig_P6);
    var2 += (var1 * ((int64_t)(calibData->dig_P5))) << 17;
    var2 += ((int64_t)(calibData->dig_P4)) << 35;
    var1 = ((var1 * var1 * ((int64_t)(calibData->dig_P3))) >> 8) + ((var1 * ((int64_t)(calibData->dig_P2))) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)(calibData->dig_P1))) >> 33;
    if(var1 == 0) 
        return 0;
    p = 1048576 - notCompesatedPressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)(calibData->dig_P9)) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)(calibData->dig_P8)) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)(calibData->dig_P7)) << 4);
    return (uint32_t)p;
}
static uint32_t compensateHumidity(int32_t notCompesatedHumidity, BME280_CALIB* calibData) {
    int32_t var1;
    var1 = T_Fine - (int32_t)76800;
    var1 = (((((notCompesatedHumidity << 14) - ((((int32_t)(calibData->dig_H4))) << 20) - ((((int32_t)(calibData->dig_H5))) * var1)) + ((int32_t)16384)) >> 15) * (((((((var1 * (((int32_t)(calibData->dig_H6)))) >> 10) * (((var1 * (((int32_t)(calibData->dig_H3)))) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * (((int32_t)(calibData->dig_H2))) + 8192) >> 14));
    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * (((int32_t)(calibData->dig_H1)))) >> 4));
    var1 = (var1 < 0 ? 0 : var1);
    var1 = (var1 > 419430400 ? 419430400 : var1);
    return(uint32_t)(var1 >> 12);
}

static void setRegister(uint8_t devAddress, uint8_t registerAddress) {
    i2c_cmd_handle_t cmdContainer = i2c_cmd_link_create();
    i2c_master_start(cmdContainer);
    i2c_master_write_byte(cmdContainer, devAddress << 1 | 0, 1);
    i2c_master_write_byte(cmdContainer, registerAddress, 1);
    i2c_master_stop(cmdContainer);
    i2c_master_cmd_begin(I2C_MODULE, cmdContainer, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmdContainer);
}

static void writeRegister(uint8_t devAddress, uint8_t registerAddress, uint8_t value) {
    i2c_cmd_handle_t cmdContainer = i2c_cmd_link_create();
    i2c_master_start(cmdContainer);
    i2c_master_write_byte(cmdContainer, devAddress << 1 | 0, 1);
    i2c_master_write_byte(cmdContainer, registerAddress, 1);
    i2c_master_write_byte(cmdContainer, value, 1);
    i2c_master_stop(cmdContainer);
    i2c_master_cmd_begin(I2C_MODULE, cmdContainer, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmdContainer);
}

static void readRegister(uint8_t devAddress, uint8_t* destination, uint8_t length) {
    i2c_cmd_handle_t cmdContainer = i2c_cmd_link_create();
    i2c_master_start(cmdContainer);
    i2c_master_write_byte(cmdContainer, devAddress << 1 | 1, 1);
    if(length > 1) 
        i2c_master_read(cmdContainer, destination, length - 1, I2C_MASTER_ACK);
    i2c_master_read(cmdContainer, destination + length - 1, 1, I2C_MASTER_NACK);
    i2c_master_stop(cmdContainer);
    i2c_master_cmd_begin(I2C_MODULE, cmdContainer, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmdContainer);
}