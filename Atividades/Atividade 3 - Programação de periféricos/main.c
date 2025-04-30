#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Definições dos pinos dos LEDs
#define LED1_GPIO GPIO_NUM_2   // LED 1 conectado ao GPIO2
#define LED2_GPIO GPIO_NUM_4   // LED 2 conectado ao GPIO4

// Tarefa para piscar o LED 1 a cada 1000ms
void piscar_led1_task(void *pvParameter) {
    while (1) {
        gpio_set_level(LED1_GPIO, 1); // Liga o LED
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED1_GPIO, 0); // Desliga o LED
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Tarefa para piscar o LED 2 a cada 200ms
void piscar_led2_task(void *pvParameter) {
    while (1) {
        gpio_set_level(LED2_GPIO, 1); // Liga o LED
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED2_GPIO, 0); // Desliga o LED
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    // Configura os pinos dos LEDs como saída
    gpio_reset_pin(LED1_GPIO);
    gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED2_GPIO);
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);

    // Cria as tarefas para controlar os LEDs
    xTaskCreate(piscar_led1_task, "piscar_led1_task", 1024, NULL, 5, NULL);
    xTaskCreate(piscar_led2_task, "piscar_led2_task", 1024, NULL, 5, NULL);
}
