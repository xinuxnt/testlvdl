/**
 * @file i2c_bus.c
 * @brief Implementación del driver de bus I2C compartido
 */

#include "i2c_bus.h"
#include "../config/hardware_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "I2C_BUS";

/*==============================================================================
 * VARIABLES PRIVADAS
 *============================================================================*/
static i2c_master_bus_handle_t bus_handle = NULL;
static SemaphoreHandle_t i2c_mutex = NULL;
static bool is_bus_initialized = false;

/*==============================================================================
 * FUNCIONES PÚBLICAS
 *============================================================================*/

esp_err_t i2c_bus_init(void)
{
    if (is_bus_initialized) {
        ESP_LOGW(TAG, "Bus I2C ya inicializado");
        return ESP_OK;
    }

    // Crear mutex para acceso thread-safe
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Error creando mutex I2C");
        return ESP_ERR_NO_MEM;
    }

    // Configuración del bus I2C maestro
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando bus I2C: %s", esp_err_to_name(ret));
        vSemaphoreDelete(i2c_mutex);
        return ret;
    }

    is_bus_initialized = true;
    ESP_LOGI(TAG, "Bus I2C inicializado - SDA: GPIO%d, SCL: GPIO%d", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    return ESP_OK;
}

esp_err_t i2c_bus_add_device(i2c_device_t *device, uint8_t dev_addr, uint32_t speed_hz)
{
    if (!is_bus_initialized) {
        ESP_LOGE(TAG, "Bus I2C no inicializado");
        return ESP_ERR_INVALID_STATE;
    }

    if (device == NULL) {
        ESP_LOGE(TAG, "Puntero a dispositivo NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Usar velocidad del bus por defecto si no se especifica
    if (speed_hz == 0) {
        speed_hz = I2C_MASTER_FREQ_HZ;
    }

    // Configuración del dispositivo
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = speed_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &device->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error agregando dispositivo 0x%02X: %s", 
                 dev_addr, esp_err_to_name(ret));
        return ret;
    }

    device->device_addr = dev_addr;
    device->is_initialized = true;

    ESP_LOGI(TAG, "Dispositivo I2C 0x%02X agregado correctamente", dev_addr);
    return ESP_OK;
}

esp_err_t i2c_bus_write_reg(i2c_device_t *device, uint8_t reg_addr, 
                            const uint8_t *data, size_t len)
{
    if (device == NULL || !device->is_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Tomar mutex para acceso exclusivo
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout esperando mutex I2C");
        return ESP_ERR_TIMEOUT;
    }

    // Crear buffer con dirección de registro + datos
    uint8_t *write_buf = malloc(len + 1);
    if (write_buf == NULL) {
        xSemaphoreGive(i2c_mutex);
        return ESP_ERR_NO_MEM;
    }

    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);

    // Realizar escritura
    esp_err_t ret = i2c_master_transmit(device->handle, write_buf, len + 1, 
                                        I2C_MASTER_TIMEOUT_MS);

    free(write_buf);
    xSemaphoreGive(i2c_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error escribiendo en 0x%02X reg 0x%02X: %s", 
                 device->device_addr, reg_addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_bus_write_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t data)
{
    return i2c_bus_write_reg(device, reg_addr, &data, 1);
}

esp_err_t i2c_bus_read_reg(i2c_device_t *device, uint8_t reg_addr, 
                          uint8_t *data, size_t len)
{
    if (device == NULL || !device->is_initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Tomar mutex
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout esperando mutex I2C");
        return ESP_ERR_TIMEOUT;
    }

    // Escribir dirección de registro y luego leer
    esp_err_t ret = i2c_master_transmit_receive(device->handle, 
                                                &reg_addr, 1,
                                                data, len,
                                                I2C_MASTER_TIMEOUT_MS);

    xSemaphoreGive(i2c_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo de 0x%02X reg 0x%02X: %s", 
                 device->device_addr, reg_addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_bus_read_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t *data)
{
    return i2c_bus_read_reg(device, reg_addr, data, 1);
}

bool i2c_bus_detect_device(uint8_t dev_addr)
{
    if (!is_bus_initialized) {
        return false;
    }

    // Crear dispositivo temporal para probar
    i2c_device_t temp_device = {0};
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &temp_device.handle);
    if (ret != ESP_OK) {
        return false;
    }

    // Intentar probe del dispositivo
    ret = i2c_master_probe(bus_handle, dev_addr, I2C_MASTER_TIMEOUT_MS);
    
    // Limpiar dispositivo temporal
    i2c_master_bus_rm_device(temp_device.handle);

    return (ret == ESP_OK);
}

void i2c_bus_scan(void)
{
    if (!is_bus_initialized) {
        ESP_LOGE(TAG, "Bus I2C no inicializado");
        return;
    }

    ESP_LOGI(TAG, "Escaneando bus I2C...");
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");

    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            uint8_t addr = i + j;
            
            // Saltar direcciones reservadas
            if (addr < 0x03 || addr > 0x77) {
                printf("   ");
                continue;
            }

            if (i2c_bus_detect_device(addr)) {
                printf("%02x ", addr);
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
}

esp_err_t i2c_bus_deinit(void)
{
    if (!is_bus_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_del_master_bus(bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error desinicializando bus I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    if (i2c_mutex != NULL) {
        vSemaphoreDelete(i2c_mutex);
        i2c_mutex = NULL;
    }

    is_bus_initialized = false;
    bus_handle = NULL;

    ESP_LOGI(TAG, "Bus I2C desinicializado");
    return ESP_OK;
}