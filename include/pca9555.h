#pragma once

#include <esp_err.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define PCA9555_REG_INPUT_0    0
#define PCA9555_REG_INPUT_1    1
#define PCA9555_REG_OUTPUT_0   2
#define PCA9555_REG_OUTPUT_1   3
#define PCA9555_REG_POLARITY_0 4
#define PCA9555_REG_POLARITY_1 5
#define PCA9555_REG_CONFIG_0   6
#define PCA9555_REG_CONFIG_1   7

#define PCA9555_DIR_IN  0
#define PCA9555_DIR_OUT 1

#define PCA9555_POL_NORMAL   0
#define PCA9555_POL_INVERTED 1

typedef void (*pca9555_intr_t)(uint8_t, bool); // Interrupt handler type

typedef struct PCA9555 {
    int              i2c_bus;
    int              i2c_address;
    int              pin_interrupt;
    uint8_t          reg_direction[2];
    uint8_t          reg_polarity[2];
    uint8_t          reg_output[2];
    pca9555_intr_t   intr_handler[16];
    TaskHandle_t     intr_task_handle;
    xSemaphoreHandle intr_trigger;
    xSemaphoreHandle mux;
} PCA9555;

extern esp_err_t pca9555_init(PCA9555* device);
extern esp_err_t pca9555_destroy(PCA9555* device);

extern void pca9555_set_interrupt_handler(PCA9555* device, uint8_t pin, pca9555_intr_t handler);

extern esp_err_t pca9555_set_gpio_direction(PCA9555* device, int pin, bool direction);
extern esp_err_t pca9555_get_gpio_direction(PCA9555* device, int pin, bool* direction);

extern esp_err_t pca9555_set_gpio_polarity(PCA9555* device, int pin, bool polarity);
extern esp_err_t pca9555_get_gpio_polarity(PCA9555* device, int pin, bool* polarity);

extern esp_err_t pca9555_set_gpio_value(PCA9555* device, int pin, bool value);
extern esp_err_t pca9555_get_gpio_value(PCA9555* device, int pin, bool* value);
