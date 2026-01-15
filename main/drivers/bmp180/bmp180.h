/**
 * @file bmp180.h
 * @brief Driver para sensor de presión y temperatura BMP180
 * 
 * Este driver implementa la comunicación con el sensor BMP180 de Bosch
 * para lectura de temperatura, presión atmosférica y cálculo de altitud.
 * 
 * Características:
 * - Sensor de presión barométrica
 * - Sensor de temperatura integrado
 * - Rango de presión: 300 a 1100 hPa
 * - Precisión: ±1 hPa
 * - Interfaz I2C
 */

#ifndef BMP180_H
#define BMP180_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * TIPOS Y ESTRUCTURAS
 *============================================================================*/

/**
 * @brief Modos de medición de presión (oversample settings)
 * 
 * Modos con mayor oversampling son más precisos pero tardan más tiempo.
 */
typedef enum {
    BMP180_MODE_ULTRA_LOW_POWER = 0,    ///< 4.5ms, baja precisión
    BMP180_MODE_STANDARD = 1,           ///< 7.5ms, precisión estándar
    BMP180_MODE_HIGH_RES = 2,           ///< 13.5ms, alta precisión
    BMP180_MODE_ULTRA_HIGH_RES = 3      ///< 25.5ms, máxima precisión
} bmp180_mode_t;

/**
 * @brief Estructura de datos de medición del BMP180
 */
typedef struct {
    float temperature;      ///< Temperatura en grados Celsius
    float pressure;         ///< Presión en hPa (hectopascales)
    float altitude;         ///< Altitud en metros (calculada)
    bool valid;            ///< true si los datos son válidos
} bmp180_data_t;

/**
 * @brief Handle del sensor BMP180
 */
typedef struct bmp180_dev_s *bmp180_handle_t;

/*==============================================================================
 * FUNCIONES PÚBLICAS
 *============================================================================*/

/**
 * @brief Inicializa el sensor BMP180
 * 
 * Esta función debe llamarse antes de usar cualquier otra función del driver.
 * Inicializa la comunicación I2C y lee los coeficientes de calibración.
 * 
 * @param mode Modo de medición (precisión vs velocidad)
 * @param handle Puntero donde se guardará el handle del sensor
 * 
 * @return 
 *      - ESP_OK: Inicialización exitosa
 *      - ESP_ERR_INVALID_ARG: Argumento inválido
 *      - ESP_ERR_NO_MEM: Sin memoria disponible
 *      - ESP_FAIL: Error de comunicación con el sensor
 */
esp_err_t bmp180_init(bmp180_mode_t mode, bmp180_handle_t *handle);

/**
 * @brief Lee la temperatura del sensor
 * 
 * @param handle Handle del sensor
 * @param temperature Puntero donde guardar la temperatura en °C
 * 
 * @return 
 *      - ESP_OK: Lectura exitosa
 *      - ESP_ERR_INVALID_ARG: Handle inválido
 *      - ESP_FAIL: Error de comunicación
 */
esp_err_t bmp180_read_temperature(bmp180_handle_t handle, float *temperature);

/**
 * @brief Lee la presión atmosférica del sensor
 * 
 * @param handle Handle del sensor
 * @param pressure Puntero donde guardar la presión en hPa
 * 
 * @return 
 *      - ESP_OK: Lectura exitosa
 *      - ESP_ERR_INVALID_ARG: Handle inválido
 *      - ESP_FAIL: Error de comunicación
 */
esp_err_t bmp180_read_pressure(bmp180_handle_t handle, float *pressure);

/**
 * @brief Lee temperatura y presión en una sola llamada
 * 
 * Más eficiente que llamar ambas funciones por separado.
 * 
 * @param handle Handle del sensor
 * @param data Puntero a estructura donde guardar los datos
 * 
 * @return 
 *      - ESP_OK: Lectura exitosa
 *      - ESP_ERR_INVALID_ARG: Handle inválido
 *      - ESP_FAIL: Error de comunicación
 */
esp_err_t bmp180_read_all(bmp180_handle_t handle, bmp180_data_t *data);

/**
 * @brief Calcula la altitud basándose en la presión medida
 * 
 * Usa la fórmula internacional de atmósfera estándar.
 * 
 * @param pressure Presión en hPa
 * @param sea_level_pressure Presión al nivel del mar en hPa (por defecto 1013.25)
 * 
 * @return Altitud en metros
 */
float bmp180_calculate_altitude(float pressure, float sea_level_pressure);

/**
 * @brief Calcula la presión al nivel del mar basándose en la altitud conocida
 * 
 * Útil para calibrar el sensor si conoces tu altitud actual.
 * 
 * @param pressure Presión medida en hPa
 * @param altitude Altitud conocida en metros
 * 
 * @return Presión equivalente al nivel del mar en hPa
 */
float bmp180_calculate_sea_level_pressure(float pressure, float altitude);

/**
 * @brief Cambia el modo de medición del sensor
 * 
 * @param handle Handle del sensor
 * @param mode Nuevo modo de medición
 * 
 * @return 
 *      - ESP_OK: Modo cambiado correctamente
 *      - ESP_ERR_INVALID_ARG: Handle o modo inválido
 */
esp_err_t bmp180_set_mode(bmp180_handle_t handle, bmp180_mode_t mode);

/**
 * @brief Verifica si el sensor responde correctamente
 * 
 * @param handle Handle del sensor
 * 
 * @return 
 *      - true: Sensor responde correctamente
 *      - false: Sensor no responde o error
 */
bool bmp180_is_available(bmp180_handle_t handle);

/**
 * @brief Desinicializa el sensor y libera recursos
 * 
 * @param handle Handle del sensor a desinicializar
 * 
 * @return 
 *      - ESP_OK: Sensor desinicializado correctamente
 */
esp_err_t bmp180_deinit(bmp180_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_H */