/**
 * Copyright (c) 2021 Nicolai Electronics
 *
 * SPDX-License-Identifier: MIT
 */

#include <sdkconfig.h>
#include <driver/gpio.h>
#include "pca9555.h"
#include "managed_i2c.h"

static const char *TAG = "PCA9555";

void pca9555_intr_task(void *arg) {
    PCA9555* device = (PCA9555*) arg;
    uint8_t data[] = {0,0};
    
    while (1) {
        if (xSemaphoreTake(device->intr_trigger, portMAX_DELAY)) {
            esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, PCA9555_REG_INPUT_0, data, 2);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "failed to read input state");
                continue;
            }
            uint16_t current_state = data[0] + (data[1] << 8);
            for (uint8_t pin = 0; pin < 16; pin++) {
                if ((current_state & (1 << pin)) != (device->pin_state & (1 << pin))) {
                    bool value = (current_state & (1 << pin)) > 0;
                    xSemaphoreTake(device->mux, portMAX_DELAY);
                    pca9555_intr_t handler = device->intr_handler[pin];
                    xSemaphoreGive(device->mux);
                    if (handler != NULL) handler(pin, value);
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            device->pin_state = current_state;
        }
    }
}

void pca9555_intr_handler(void *arg) {
    /* in interrupt handler context */
    PCA9555* device = (PCA9555*) arg;
    xSemaphoreGiveFromISR(device->intr_trigger, NULL);
}

esp_err_t pca9555_init(PCA9555* device, int i2c_bus, int i2c_address, int pin_interrupt) {    
    esp_err_t res;
    
    device->i2c_bus = i2c_bus;
    device->i2c_address = i2c_address;
    device->pin_interrupt = pin_interrupt;
    
    // Write default values for the registers
    device->reg_direction[0] = 0xFF; // Set all pins to input
    device->reg_direction[1] = 0xFF;
    device->reg_polarity[0]  = 0x00; // Don't invert input polarity
    device->reg_polarity[1]  = 0x00;
    device->reg_output[0]    = 0x00; // Set value of output flipflops to LOW (0)
    device->reg_output[1]    = 0x00;

    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, PCA9555_REG_CONFIG_0, device->reg_direction, 2);
    if (res != ESP_OK) return res;

    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, PCA9555_REG_POLARITY_0, device->reg_polarity, 2);
    if (res != ESP_OK) return res;

    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, PCA9555_REG_OUTPUT_0, device->reg_output, 2);
    if (res != ESP_OK) return res;

    //Create mutex
    device->mux = xSemaphoreCreateMutex();
    if (device->mux == NULL) return ESP_ERR_NO_MEM;

    //Create interrupt trigger
    device->intr_trigger = xSemaphoreCreateBinary();
    if (device->intr_trigger == NULL) return ESP_ERR_NO_MEM;

    // Initialize the interrupt handler pointers
    for (uint8_t pin = 0; pin < 16; pin++) {
        device->intr_handler[pin] = NULL;
    }

    device->intr_task_handle = NULL;

    //Attach interrupt to interrupt pin
    if (device->pin_interrupt >= 0) {
        res = gpio_isr_handler_add(device->pin_interrupt, pca9555_intr_handler, (void*) device);
        if (res != ESP_OK) return res;

        gpio_config_t io_conf = {
            .intr_type    = GPIO_INTR_NEGEDGE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1LL << device->pin_interrupt,
            .pull_down_en = 0,
            .pull_up_en   = 1,
        };

        res = gpio_config(&io_conf);
        if (res != ESP_OK) return res;
        
        xTaskCreate(&pca9555_intr_task, "PCA9555 interrupt", 4096, (void*) device, 10, &device->intr_task_handle);
        xSemaphoreGive(device->intr_trigger);
    }
    return ESP_OK;
}

esp_err_t pca9555_destroy(PCA9555* device) {
    esp_err_t res;
    if (device->pin_interrupt >= 0) {
        vTaskDelete(device->intr_task_handle);
        
        gpio_config_t io_conf = {
            .intr_type    = GPIO_INTR_DISABLE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1LL << device->pin_interrupt,
            .pull_down_en = 0,
            .pull_up_en   = 1,
        };

        res = gpio_config(&io_conf);
        if (res != ESP_OK) return res;
        
        res = gpio_isr_handler_remove(device->pin_interrupt);
        if (res != ESP_OK) return res;
    }
    vSemaphoreDelete(device->mux);
    vSemaphoreDelete(device->intr_trigger);
    return ESP_OK;
}

esp_err_t pca9555_set_gpio_direction(PCA9555* device, uint8_t pin, bool direction) {
    if (pin > 15) return ESP_FAIL; //Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    if (direction) {
        device->reg_direction[port] &= ~(1 << bit); //Set the pin to output
    } else {
        device->reg_direction[port] |= (1 << bit); //Set the pin to input
    }
    return i2c_write_reg_n(device->i2c_bus, device->i2c_address, PCA9555_REG_CONFIG_0, device->reg_direction, 2);
}

esp_err_t pca9555_get_gpio_direction(PCA9555* device, uint8_t pin, bool* direction) {
    if (direction == NULL) return ESP_FAIL;
    if (pin > 15) return ESP_FAIL; //Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    *direction = ((device->reg_direction[port] >> bit) & 1) ? PCA9555_DIR_IN : PCA9555_DIR_OUT;
    return ESP_OK;
}

esp_err_t pca9555_set_gpio_polarity(PCA9555* device, uint8_t pin, bool polarity) {
    if (pin > 15) return ESP_FAIL; //Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    if (polarity) {
        device->reg_polarity[port] |= (1 << bit);
    } else {
        device->reg_polarity[port] &= ~(1 << bit);
    }
    return i2c_write_reg_n(device->i2c_bus, device->i2c_address, PCA9555_REG_POLARITY_0, device->reg_polarity, 2);
}

esp_err_t pca9555_get_gpio_polarity(PCA9555* device, uint8_t pin, bool* polarity) {
    if (polarity == NULL) return ESP_FAIL;
    if (pin > 15) return ESP_FAIL; //Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    *polarity = ((device->reg_polarity[port] >> bit) & 1) ? PCA9555_POL_NORMAL : PCA9555_POL_INVERTED;
    return ESP_OK;
}

esp_err_t pca9555_set_gpio_value(PCA9555* device, uint8_t pin, bool value) {
    if (pin > 15) return ESP_FAIL; //Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    bool direction;
    esp_err_t res = pca9555_get_gpio_direction(device, pin, &direction);
    if (res != ESP_OK) return res;
    if (direction != PCA9555_DIR_OUT) return ESP_FAIL; // Pin is an input
    if (value) {
        device->reg_output[port] |= (1 << bit);
    } else {
        device->reg_output[port] &= ~(1 << bit);
    }
    return i2c_write_reg_n(device->i2c_bus, device->i2c_address, port ? PCA9555_REG_OUTPUT_1 : PCA9555_REG_OUTPUT_0, &device->reg_output[port], 1);
}

esp_err_t pca9555_get_gpio_value(PCA9555* device, uint8_t pin, bool* value) {
    if (value == NULL) return ESP_FAIL;
    if (pin > 15) return ESP_FAIL; //Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    bool direction;
    esp_err_t res = pca9555_get_gpio_direction(device, pin, &direction);
    uint8_t reg;
    if (res != ESP_OK) return res;
    if (direction) {
        reg = port ? PCA9555_REG_OUTPUT_1 : PCA9555_REG_OUTPUT_0;
    } else {
        reg = port ? PCA9555_REG_INPUT_1 : PCA9555_REG_INPUT_0;
    }
    uint8_t reg_value;
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, reg, &reg_value, 1);
    if (res != ESP_OK) return ESP_FAIL;
    *value = (reg_value>>bit) & 1;
    return ESP_OK;
}

esp_err_t pca9555_set_interrupt_handler(PCA9555* device, uint8_t pin, pca9555_intr_t handler) {
    if (pin > 15) {
        return ESP_FAIL;
    }
    xSemaphoreTake(device->mux, portMAX_DELAY);
    device->intr_handler[pin] = handler;
    xSemaphoreGive(device->mux);
    return ESP_OK;
}
