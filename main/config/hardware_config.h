/**
 * @file hardware_config.h
 * @brief Configuración de hardware para JC1060P470
 * 
 * Este archivo centraliza toda la configuración de pines y periféricos
 * del proyecto. Facilita el mantenimiento y cambios de hardware.
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"

/*==============================================================================
 * CONFIGURACIÓN I2C (Conector CN4)
 *============================================================================*/
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000      // 100kHz (estándar para BMP180)
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// Pines I2C en CN4
#define I2C_MASTER_SDA_IO           GPIO_NUM_7
#define I2C_MASTER_SCL_IO           GPIO_NUM_8

/*==============================================================================
 * DIRECCIONES I2C DE DISPOSITIVOS
 *============================================================================*/
#define BMP180_I2C_ADDR             0x77        // Dirección fija del BMP180
#define ADS1115_I2C_ADDR            0x48        // Dirección por defecto (para futuro uso)
#define PCA9685_I2C_ADDR            0x40        // Dirección por defecto (para futuro uso)

/*==============================================================================
 * CONFIGURACIÓN DE TAREAS Y TIEMPOS
 *============================================================================*/
#define BMP180_UPDATE_PERIOD_MS     2000        // Actualizar sensor cada 2 segundos
#define UI_UPDATE_PERIOD_MS         100         // Actualizar UI cada 100ms

/*==============================================================================
 * CONFIGURACIÓN DE DISPLAY (Referencia - ya configurado en tu proyecto)
 *============================================================================*/
// Estos valores son de referencia, tu proyecto ya los tiene configurados
#define DISPLAY_WIDTH               1024
#define DISPLAY_HEIGHT              600
#define DISPLAY_BPP                 16

/*==============================================================================
 * CONFIGURACIÓN DE LVGL
 *============================================================================*/
#define LVGL_TICK_PERIOD_MS         2

#endif /* HARDWARE_CONFIG_H */