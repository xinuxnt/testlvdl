/**
 * @file i2c_bus.h
 * @brief Driver de bus I2C compartido para múltiples dispositivos
 * 
 * Este módulo gestiona el bus I2C compartido por todos los dispositivos
 * (BMP180, ADS1115, PCA9685, etc.) garantizando acceso seguro y thread-safe.
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * TIPOS Y ESTRUCTURAS
 *============================================================================*/

/**
 * @brief Handle de dispositivo I2C
 */
typedef struct {
    i2c_master_dev_handle_t handle;
    uint8_t device_addr;
    bool is_initialized;
} i2c_device_t;

/*==============================================================================
 * FUNCIONES PÚBLICAS
 *============================================================================*/

/**
 * @brief Inicializa el bus I2C maestro
 * 
 * Debe llamarse una vez al inicio del programa antes de usar cualquier
 * dispositivo I2C.
 * 
 * @return 
 *      - ESP_OK: Inicialización exitosa
 *      - ESP_ERR_INVALID_ARG: Parámetros inválidos
 *      - ESP_FAIL: Error al inicializar el bus
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Agrega un dispositivo al bus I2C
 * 
 * @param device Puntero a la estructura del dispositivo
 * @param dev_addr Dirección I2C del dispositivo (7 bits)
 * @param speed_hz Velocidad del dispositivo en Hz (0 = usar velocidad del bus)
 * 
 * @return 
 *      - ESP_OK: Dispositivo agregado correctamente
 *      - ESP_ERR_INVALID_ARG: Argumentos inválidos
 *      - ESP_ERR_NO_MEM: Sin memoria disponible
 */
esp_err_t i2c_bus_add_device(i2c_device_t *device, uint8_t dev_addr, uint32_t speed_hz);

/**
 * @brief Escribe datos a un dispositivo I2C
 * 
 * @param device Puntero al dispositivo
 * @param reg_addr Dirección del registro
 * @param data Puntero a los datos a escribir
 * @param len Longitud de los datos
 * 
 * @return 
 *      - ESP_OK: Escritura exitosa
 *      - ESP_ERR_INVALID_ARG: Argumentos inválidos
 *      - ESP_FAIL: Error de comunicación
 */
esp_err_t i2c_bus_write_reg(i2c_device_t *device, uint8_t reg_addr, 
                            const uint8_t *data, size_t len);

/**
 * @brief Escribe un byte a un registro de dispositivo I2C
 * 
 * @param device Puntero al dispositivo
 * @param reg_addr Dirección del registro
 * @param data Byte a escribir
 * 
 * @return 
 *      - ESP_OK: Escritura exitosa
 *      - ESP_ERR_INVALID_ARG: Argumentos inválidos
 *      - ESP_FAIL: Error de comunicación
 */
esp_err_t i2c_bus_write_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t data);

/**
 * @brief Lee datos de un dispositivo I2C
 * 
 * @param device Puntero al dispositivo
 * @param reg_addr Dirección del registro
 * @param data Puntero al buffer donde guardar los datos
 * @param len Longitud de datos a leer
 * 
 * @return 
 *      - ESP_OK: Lectura exitosa
 *      - ESP_ERR_INVALID_ARG: Argumentos inválidos
 *      - ESP_FAIL: Error de comunicación
 */
esp_err_t i2c_bus_read_reg(i2c_device_t *device, uint8_t reg_addr, 
                          uint8_t *data, size_t len);

/**
 * @brief Lee un byte de un registro de dispositivo I2C
 * 
 * @param device Puntero al dispositivo
 * @param reg_addr Dirección del registro
 * @param data Puntero donde guardar el byte leído
 * 
 * @return 
 *      - ESP_OK: Lectura exitosa
 *      - ESP_ERR_INVALID_ARG: Argumentos inválidos
 *      - ESP_FAIL: Error de comunicación
 */
esp_err_t i2c_bus_read_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Detecta si un dispositivo responde en el bus I2C
 * 
 * @param dev_addr Dirección del dispositivo a detectar
 * 
 * @return 
 *      - true: Dispositivo detectado
 *      - false: Dispositivo no responde
 */
bool i2c_bus_detect_device(uint8_t dev_addr);

/**
 * @brief Escanea el bus I2C y lista todos los dispositivos encontrados
 * 
 * Útil para debug. Imprime en consola las direcciones de dispositivos detectados.
 */
void i2c_bus_scan(void);

/**
 * @brief Desinicializa el bus I2C y libera recursos
 * 
 * @return 
 *      - ESP_OK: Bus desinicializado correctamente
 *      - ESP_FAIL: Error al desinicializar
 */
esp_err_t i2c_bus_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_H */