#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED1 GPIO_NUM_1
#define LED2 GPIO_NUM_2
#define LED3 GPIO_NUM_3
#define LED4 GPIO_NUM_4

#define BTN_INC GPIO_NUM_5
#define BTN_TOGGLE_INC GPIO_NUM_6

#define DEBOUNCE_DELAY_US 200000  // 200 ms
#define CONFIRM_DELAY_MS 5        // Verifica estado após 5 ms

volatile int counter = 0;
volatile int step = 1;

int last_state_inc = 1;
int last_state_toggle = 1;
int64_t last_press_inc = 0;
int64_t last_press_toggle = 0;

void update_leds(int value) {
    gpio_set_level(LED1, (value >> 0) & 1);
    gpio_set_level(LED2, (value >> 1) & 1);
    gpio_set_level(LED3, (value >> 2) & 1);
    gpio_set_level(LED4, (value >> 3) & 1);
}

void app_main() {
    // Configura LEDs como saída
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3) | (1ULL << LED4),
    };
    gpio_config(&io_conf);

    // Configura botões como entrada
    gpio_config_t btn_config = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_TOGGLE_INC),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&btn_config);

    update_leds(counter); // Inicializa LEDs com valor 0

    while (1) {
        int btn_inc = gpio_get_level(BTN_INC);
        int btn_toggle = gpio_get_level(BTN_TOGGLE_INC);
        int64_t now = esp_timer_get_time();

        // Botão de incremento
        if (btn_inc == 0 && last_state_inc == 1 && now - last_press_inc > DEBOUNCE_DELAY_US) {
            vTaskDelay(pdMS_TO_TICKS(CONFIRM_DELAY_MS)); // Delay curto para confirmar estabilidade
            if (gpio_get_level(BTN_INC) == 0) {
                counter = (counter + step) & 0xF; // contador circular 4 bits
                printf("Valor do contador: %d\n", counter);
                update_leds(counter);
                last_press_inc = now;
            }
        }
        last_state_inc = btn_inc;

        // Botão para alternar passo
        if (btn_toggle == 0 && last_state_toggle == 1 && now - last_press_toggle > DEBOUNCE_DELAY_US) {
            vTaskDelay(pdMS_TO_TICKS(CONFIRM_DELAY_MS));
            if (gpio_get_level(BTN_TOGGLE_INC) == 0) {
                step = (step == 1) ? 2 : 1;
                printf("Incremento agora é: %d\n", step);
                last_press_toggle = now;
            }
        }
        last_state_toggle = btn_toggle;

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeno delay para evitar loop intenso
    }
}
