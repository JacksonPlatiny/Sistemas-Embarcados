#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "int_i2c.h"

#define TAG "CONTADOR"

// GPIOs
#define GPIO_LED1 1
#define GPIO_LED2 2
#define GPIO_LED3 3
#define GPIO_LED4 4
#define GPIO_BTN_INC 5
#define GPIO_BTN_DEC 6
#define GPIO_PWM_LED 7
#define I2C_SDA 8
#define I2C_SCL 9

#define I2C_PORT I2C_NUM_0
#define LCD_ADDR 0x27

static uint8_t contador = 0;
static QueueHandle_t fila_eventos = NULL;

lcd_i2c_handle_t lcd = {
    .address = LCD_ADDR,
    .num = I2C_PORT,
    .backlight = 1,
    .size = DISPLAY_16X02
};

typedef enum {
    BTN_INC_EVT,
    BTN_DEC_EVT
} evento_t;

// ISR dos botões
static void IRAM_ATTR isr_btn_inc(void* arg) {
    evento_t evt = BTN_INC_EVT;
    xQueueSendFromISR(fila_eventos, &evt, NULL);
}

static void IRAM_ATTR isr_btn_dec(void* arg) {
    evento_t evt = BTN_DEC_EVT;
    xQueueSendFromISR(fila_eventos, &evt, NULL);
}

// Atualiza LEDs binários
void atualizar_leds(uint8_t valor) {
    gpio_set_level(GPIO_LED1, (valor >> 0) & 1);
    gpio_set_level(GPIO_LED2, (valor >> 1) & 1);
    gpio_set_level(GPIO_LED3, (valor >> 2) & 1);
    gpio_set_level(GPIO_LED4, (valor >> 3) & 1);
}

// Atualiza brilho do LED PWM
void atualizar_pwm(uint8_t valor) {
    uint32_t duty = (valor * 8191) / 15; // escala de 0 a 8191
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Atualiza conteúdo do LCD
void atualizar_lcd(uint8_t valor) {
  
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Hex: 0x%X", valor);

    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "Dec: %d", valor);
}

// Configura botões
void configurar_botoes() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_BTN_INC) | (1ULL << GPIO_BTN_DEC),
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_BTN_INC, isr_btn_inc, NULL);
    gpio_isr_handler_add(GPIO_BTN_DEC, isr_btn_dec, NULL);
}

// Inicializa PWM
void configurar_pwm() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = GPIO_PWM_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

// Inicializa I2C e LCD
void configurar_lcd() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    lcd_i2c_init(&lcd);
}

void app_main(void) {
    // LEDs binários
    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED4, GPIO_MODE_OUTPUT);

    configurar_botoes();
    configurar_pwm();
    configurar_lcd();

    fila_eventos = xQueueCreate(10, sizeof(evento_t));

    atualizar_leds(contador);
    atualizar_pwm(contador);
    atualizar_lcd(contador);

    while (1) {
        evento_t evt;
        if (xQueueReceive(fila_eventos, &evt, pdMS_TO_TICKS(20))) {
            static int64_t ultimo_evt = 0;
            int64_t agora = esp_timer_get_time();
            if (agora - ultimo_evt > 200000) {
                ultimo_evt = agora;

                if (evt == BTN_INC_EVT) {
                    contador = (contador + 1) & 0x0F;
                } else if (evt == BTN_DEC_EVT) {
                    contador = (contador - 1) & 0x0F;
                }

                atualizar_leds(contador);
                atualizar_pwm(contador);
                atualizar_lcd(contador);

                ESP_LOGI(TAG, "Valor contador: %02X (%d)", contador, contador);
            }
        }
    }
}
