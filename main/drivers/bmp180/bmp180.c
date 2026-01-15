/**
 * @file bmp180.c
 * @brief Implementación del driver BMP180
 */

#include "bmp180.h"
#include "../i2c_bus.h"
#include "../../config/hardware_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "BMP180";

/*==============================================================================
 * REGISTROS DEL BMP180
 *============================================================================*/
#define BMP180_REG_CHIP_ID          0xD0
#define BMP180_REG_SOFT_RESET       0xE0
#define BMP180_REG_CONTROL          0xF4
#define BMP180_REG_DATA_MSB         0xF6
#define BMP180_REG_DATA_LSB         0xF7
#define BMP180_REG_DATA_XLSB        0xF8

// Registros de calibración
#define BMP180_REG_CAL_AC1_MSB      0xAA
#define BMP180_REG_CAL_AC1_LSB      0xAB
#define BMP180_REG_CAL_AC2_MSB      0xAC
#define BMP180_REG_CAL_AC2_LSB      0xAD
#define BMP180_REG_CAL_AC3_MSB      0xAE
#define BMP180_REG_CAL_AC3_LSB      0xAF
#define BMP180_REG_CAL_AC4_MSB      0xB0
#define BMP180_REG_CAL_AC4_LSB      0xB1
#define BMP180_REG_CAL_AC5_MSB      0xB2
#define BMP180_REG_CAL_AC5_LSB      0xB3
#define BMP180_REG_CAL_AC6_MSB      0xB4
#define BMP180_REG_CAL_AC6_LSB      0xB5
#define BMP180_REG_CAL_B1_MSB       0xB6
#define BMP180_REG_CAL_B1_LSB       0xB7
#define BMP180_REG_CAL_B2_MSB       0xB8
#define BMP180_REG_CAL_B2_LSB       0xB9
#define BMP180_REG_CAL_MB_MSB       0xBA
#define BMP180_REG_CAL_MB_LSB       0xBB
#define BMP180_REG_CAL_MC_MSB       0xBC
#define BMP180_REG_CAL_MC_LSB       0xBD
#define BMP180_REG_CAL_MD_MSB       0xBE
#define BMP180_REG_CAL_MD_LSB       0xBF

// Comandos
#define BMP180_CMD_READ_TEMP        0x2E
#define BMP180_CMD_READ_PRESSURE    0x34

// ID del chip
#define BMP180_CHIP_ID              0x55

// Constantes
#define BMP180_SEA_LEVEL_PRESSURE   1013.25f

/*==============================================================================
 * ESTRUCTURAS PRIVADAS
 *============================================================================*/

/**
 * @brief Coeficientes de calibración del BMP180
 */
typedef struct {
    int16_t  ac1;
    int16_t  ac2;
    int16_t  ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t  b1;
    int16_t  b2;
    int16_t  mb;
    int16_t  mc;
    int16_t  md;
} bmp180_calib_data_t;

/**
 * @brief Estructura del dispositivo BMP180
 */
struct bmp180_dev_s {
    i2c_device_t i2c_dev;
    bmp180_calib_data_t calib;
    bmp180_mode_t mode;
    int32_t b5;  // Variable intermedia de cálculo
};

/*==============================================================================
 * FUNCIONES PRIVADAS
 *============================================================================*/

/**
 * @brief Lee los coeficientes de calibración del sensor
 */
static esp_err_t bmp180_read_calibration(bmp180_handle_t handle)
{
    uint8_t calib_data[22];
    
    esp_err_t ret = i2c_bus_read_reg(&handle->i2c_dev, BMP180_REG_CAL_AC1_MSB, 
                                     calib_data, sizeof(calib_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo datos de calibración");
        return ret;
    }

    // Parsear coeficientes (big-endian)
    handle->calib.ac1 = (int16_t)((calib_data[0] << 8) | calib_data[1]);
    handle->calib.ac2 = (int16_t)((calib_data[2] << 8) | calib_data[3]);
    handle->calib.ac3 = (int16_t)((calib_data[4] << 8) | calib_data[5]);
    handle->calib.ac4 = (uint16_t)((calib_data[6] << 8) | calib_data[7]);
    handle->calib.ac5 = (uint16_t)((calib_data[8] << 8) | calib_data[9]);
    handle->calib.ac6 = (uint16_t)((calib_data[10] << 8) | calib_data[11]);
    handle->calib.b1 = (int16_t)((calib_data[12] << 8) | calib_data[13]);
    handle->calib.b2 = (int16_t)((calib_data[14] << 8) | calib_data[15]);
    handle->calib.mb = (int16_t)((calib_data[16] << 8) | calib_data[17]);
    handle->calib.mc = (int16_t)((calib_data[18] << 8) | calib_data[19]);
    handle->calib.md = (int16_t)((calib_data[20] << 8) | calib_data[21]);

    ESP_LOGI(TAG, "Calibración leída: AC1=%d, AC2=%d, AC3=%d, AC4=%u, AC5=%u, AC6=%u",
             handle->calib.ac1, handle->calib.ac2, handle->calib.ac3,
             handle->calib.ac4, handle->calib.ac5, handle->calib.ac6);

    return ESP_OK;
}

/**
 * @brief Lee temperatura sin compensar del sensor
 */
static esp_err_t bmp180_read_raw_temperature(bmp180_handle_t handle, int32_t *raw_temp)
{
    // Iniciar lectura de temperatura
    esp_err_t ret = i2c_bus_write_byte(&handle->i2c_dev, BMP180_REG_CONTROL, 
                                       BMP180_CMD_READ_TEMP);
    if (ret != ESP_OK) {
        return ret;
    }

    // Esperar tiempo de conversión (4.5ms)
    vTaskDelay(pdMS_TO_TICKS(5));

    // Leer resultado
    uint8_t data[2];
    ret = i2c_bus_read_reg(&handle->i2c_dev, BMP180_REG_DATA_MSB, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    *raw_temp = (int32_t)((data[0] << 8) | data[1]);
    return ESP_OK;
}

/**
 * @brief Lee presión sin compensar del sensor
 */
static esp_err_t bmp180_read_raw_pressure(bmp180_handle_t handle, int32_t *raw_press)
{
    // Comando de lectura según el modo
    uint8_t cmd = BMP180_CMD_READ_PRESSURE + (handle->mode << 6);
    
    esp_err_t ret = i2c_bus_write_byte(&handle->i2c_dev, BMP180_REG_CONTROL, cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    // Tiempo de espera según el modo
    const uint8_t delays_ms[] = {5, 8, 14, 26};
    vTaskDelay(pdMS_TO_TICKS(delays_ms[handle->mode]));

    // Leer resultado (3 bytes)
    uint8_t data[3];
    ret = i2c_bus_read_reg(&handle->i2c_dev, BMP180_REG_DATA_MSB, data, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    *raw_press = (((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2]) 
                 >> (8 - handle->mode);
    
    return ESP_OK;
}

/**
 * @brief Calcula temperatura compensada
 */
static float bmp180_compensate_temperature(bmp180_handle_t handle, int32_t raw_temp)
{
    int32_t x1 = ((raw_temp - handle->calib.ac6) * handle->calib.ac5) >> 15;
    int32_t x2 = (handle->calib.mc << 11) / (x1 + handle->calib.md);
    handle->b5 = x1 + x2;
    
    float temp = ((handle->b5 + 8) >> 4) / 10.0f;
    return temp;
}

/**
 * @brief Calcula presión compensada
 */
static float bmp180_compensate_pressure(bmp180_handle_t handle, int32_t raw_press)
{
    int32_t b6 = handle->b5 - 4000;
    int32_t x1 = (handle->calib.b2 * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (handle->calib.ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((handle->calib.ac1 * 4 + x3) << handle->mode) + 2) >> 2;
    
    x1 = (handle->calib.ac3 * b6) >> 13;
    x2 = (handle->calib.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (handle->calib.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)raw_press - b3) * (50000 >> handle->mode);
    
    int32_t p;
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    
    return p / 100.0f;  // Convertir a hPa
}

/*==============================================================================
 * FUNCIONES PÚBLICAS
 *============================================================================*/

esp_err_t bmp180_init(bmp180_mode_t mode, bmp180_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Alocar memoria para el handle
    bmp180_handle_t dev = calloc(1, sizeof(struct bmp180_dev_s));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Error alocando memoria para BMP180");
        return ESP_ERR_NO_MEM;
    }

    dev->mode = mode;

    // Agregar dispositivo al bus I2C
    esp_err_t ret = i2c_bus_add_device(&dev->i2c_dev, BMP180_I2C_ADDR, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error agregando BMP180 al bus I2C");
        free(dev);
        return ret;
    }

    // Verificar ID del chip
    uint8_t chip_id;
    ret = i2c_bus_read_byte(&dev->i2c_dev, BMP180_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK || chip_id != BMP180_CHIP_ID) {
        ESP_LOGE(TAG, "BMP180 no detectado (ID: 0x%02X, esperado: 0x%02X)", 
                 chip_id, BMP180_CHIP_ID);
        free(dev);
        return ESP_FAIL;
    }

    // Leer coeficientes de calibración
    ret = bmp180_read_calibration(dev);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    *handle = dev;
    ESP_LOGI(TAG, "BMP180 inicializado correctamente en modo %d", mode);
    
    return ESP_OK;
}

esp_err_t bmp180_read_temperature(bmp180_handle_t handle, float *temperature)
{
    if (handle == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t raw_temp;
    esp_err_t ret = bmp180_read_raw_temperature(handle, &raw_temp);
    if (ret != ESP_OK) {
        return ret;
    }

    *temperature = bmp180_compensate_temperature(handle, raw_temp);
    return ESP_OK;
}

esp_err_t bmp180_read_pressure(bmp180_handle_t handle, float *pressure)
{
    if (handle == NULL || pressure == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Primero leer temperatura (necesaria para compensar presión)
    int32_t raw_temp;
    esp_err_t ret = bmp180_read_raw_temperature(handle, &raw_temp);
    if (ret != ESP_OK) {
        return ret;
    }
    bmp180_compensate_temperature(handle, raw_temp);

    // Ahora leer presión
    int32_t raw_press;
    ret = bmp180_read_raw_pressure(handle, &raw_press);
    if (ret != ESP_OK) {
        return ret;
    }

    *pressure = bmp180_compensate_pressure(handle, raw_press);
    return ESP_OK;
}

esp_err_t bmp180_read_all(bmp180_handle_t handle, bmp180_data_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(data, 0, sizeof(bmp180_data_t));

    // Leer temperatura
    int32_t raw_temp;
    esp_err_t ret = bmp180_read_raw_temperature(handle, &raw_temp);
    if (ret != ESP_OK) {
        return ret;
    }
    data->temperature = bmp180_compensate_temperature(handle, raw_temp);

    // Leer presión
    int32_t raw_press;
    ret = bmp180_read_raw_pressure(handle, &raw_press);
    if (ret != ESP_OK) {
        return ret;
    }
    data->pressure = bmp180_compensate_pressure(handle, raw_press);

    // Calcular altitud
    data->altitude = bmp180_calculate_altitude(data->pressure, BMP180_SEA_LEVEL_PRESSURE);
    data->valid = true;

    return ESP_OK;
}

float bmp180_calculate_altitude(float pressure, float sea_level_pressure)
{
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

float bmp180_calculate_sea_level_pressure(float pressure, float altitude)
{
    return pressure / powf(1.0f - (altitude / 44330.0f), 5.255f);
}

esp_err_t bmp180_set_mode(bmp180_handle_t handle, bmp180_mode_t mode)
{
    if (handle == NULL || mode > BMP180_MODE_ULTRA_HIGH_RES) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->mode = mode;
    ESP_LOGI(TAG, "Modo cambiado a %d", mode);
    return ESP_OK;
}

bool bmp180_is_available(bmp180_handle_t handle)
{
    if (handle == NULL) {
        return false;
    }

    uint8_t chip_id;
    esp_err_t ret = i2c_bus_read_byte(&handle->i2c_dev, BMP180_REG_CHIP_ID, &chip_id);
    
    return (ret == ESP_OK && chip_id == BMP180_CHIP_ID);
}

esp_err_t bmp180_deinit(bmp180_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    free(handle);
    ESP_LOGI(TAG, "BMP180 desinicializado");
    
    return ESP_OK;
}