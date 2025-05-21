#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define LED1 GPIO_NUM_1
#define LED2 GPIO_NUM_2
#define LED3 GPIO_NUM_3
#define LED4 GPIO_NUM_4

#define BTN_INC GPIO_NUM_5
#define BTN_TOGGLE_INC GPIO_NUM_6

#define DEBOUNCE_DELAY_US 200000  // 200 ms

static const char *TAG = "ISR_BUTTON";

volatile int counter = 0;
volatile int step = 1;

volatile int64_t last_press_inc = 0;
volatile int64_t last_press_toggle = 0;

void update_leds(int value) {
    gpio_set_level(LED1, (value >> 0) & 1);
    gpio_set_level(LED2, (value >> 1) & 1);
    gpio_set_level(LED3, (value >> 2) & 1);
    gpio_set_level(LED4, (value >> 3) & 1);
}

static void IRAM_ATTR btn_isr_handler(void* arg) {
    int gpio_num = (int)(intptr_t)arg;
    int64_t now = esp_timer_get_time();

    if (gpio_num == BTN_INC) {
        if (now - last_press_inc > DEBOUNCE_DELAY_US) {
            last_press_inc = now;
            counter = (counter + step) & 0xF;
            update_leds(counter);
        }
    } else if (gpio_num == BTN_TOGGLE_INC) {
        if (now - last_press_toggle > DEBOUNCE_DELAY_US) {
            last_press_toggle = now;
            step = (step == 1) ? 2 : 1;
        }
    }
}

void app_main() {
    // Configura LEDs como saída
    gpio_config_t io_conf_leds = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3) | (1ULL << LED4),
    };
    gpio_config(&io_conf_leds);

    // Configura botões como entrada com resistores externos
    gpio_config_t io_conf_btns = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_TOGGLE_INC),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf_btns);

    // Instala o driver de interrupção
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, btn_isr_handler, (void*)(intptr_t)BTN_INC);
    gpio_isr_handler_add(BTN_TOGGLE_INC, btn_isr_handler, (void*)(intptr_t)BTN_TOGGLE_INC);

    update_leds(counter);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
