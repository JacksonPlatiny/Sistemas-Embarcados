#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sys/stat.h"
#include "int_i2c.h"

#define TAG "TEMP_MONITOR"

#define GPIO_LED1 1
#define GPIO_LED2 2
#define GPIO_LED3 3
#define GPIO_LED4 4
#define GPIO_BTN_INC 5
#define GPIO_BTN_DEC 6
#define GPIO_BUZZER 7
#define I2C_SDA 8
#define I2C_SCL 9
#define NTC_GPIO 10
#define NTC_ADC_CHANNEL ADC_CHANNEL_9

#define I2C_PORT I2C_NUM_0
#define LCD_ADDR 0x27
#define TEMP_DEFAULT_ALARM 25
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

static QueueHandle_t fila_eventos = NULL;
static volatile int temp_alarme = TEMP_DEFAULT_ALARM;
static volatile int temperatura_atual = 0;
static int64_t ultimo_evento = 0;
static bool estado_pisca = false;
static int64_t ultimo_pisca_time = 0;
static sdmmc_card_t* card;

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

static void IRAM_ATTR isr_btn_inc(void* arg) {
    evento_t evt = BTN_INC_EVT;
    xQueueSendFromISR(fila_eventos, &evt, NULL);
}

static void IRAM_ATTR isr_btn_dec(void* arg) {
    evento_t evt = BTN_DEC_EVT;
    xQueueSendFromISR(fila_eventos, &evt, NULL);
}

int ler_temperatura_ntc() {
    const float BETA = 3950.0;
    int analogValue = adc1_get_raw(NTC_ADC_CHANNEL);
    float celsius = 1.0 / (log(1.0 / (4095.0 / analogValue - 1.0)) / BETA + 1.0 / 298.15) - 273.15;
    return (int)celsius;
}

void atualizar_leds(int temperatura, int limite) {
    int64_t agora = esp_timer_get_time();
    bool alarme = temperatura >= limite;
    if (alarme) {
        if (agora - ultimo_pisca_time > 500000) {
            ultimo_pisca_time = agora;
            estado_pisca = !estado_pisca;
        }
        gpio_set_level(GPIO_LED1, estado_pisca);
        gpio_set_level(GPIO_LED2, estado_pisca);
        gpio_set_level(GPIO_LED3, estado_pisca);
        gpio_set_level(GPIO_LED4, estado_pisca);
    } else {
        int diff = limite - temperatura;
        gpio_set_level(GPIO_LED1, diff <= 20);
        gpio_set_level(GPIO_LED2, diff <= 15);
        gpio_set_level(GPIO_LED3, diff <= 10);
        gpio_set_level(GPIO_LED4, diff <= 5);
    }
}

void configurar_buzzer() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = GPIO_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void ativar_buzzer(bool ligar) {
    uint32_t duty = ligar ? 128 : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void atualizar_lcd(int temp, int limite) {
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Temp: %2d C", temp);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "Limite: %2d C", limite);

}

void configurar_lcd() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    lcd_i2c_init(&lcd);
    lcd_i2c_clear(&lcd);
    vTaskDelay(pdMS_TO_TICKS(200));
}

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

void iniciar_sdcard() {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Falha ao inicializar SPI (%s)", esp_err_to_name(ret));
        return;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;
    slot_config.gpio_cd = -1;

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Falha ao montar o cartão SD (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI("SDCARD", "Cartão SD montado com sucesso");
    }
}

void salvar_leitura_sd(int temperatura, int limite) {
    static int ultima_temp = -999;
    static int ultimo_limite = -999;
    static int contador = 0;

    if (temperatura == ultima_temp && limite == ultimo_limite) return;

    FILE *f = fopen("/sdcard/leituras.txt", "a");
    if (f == NULL) {
        ESP_LOGE("SDCARD", "Erro ao abrir arquivo");
        return;
    }

    contador++;
    fprintf(f, "Leitura #%d | Temp: %d °C | Alarme: %d °C\n", contador, temperatura, limite);
    fclose(f);

    ultima_temp = temperatura;
    ultimo_limite = limite;
    ESP_LOGI("SDCARD", "Leitura salva no SD");
}

void tarefa_leitura_temp(void *pv) {
    while (1) {
        temperatura_atual = ler_temperatura_ntc();
        ESP_LOGI(TAG, "Temp: %d | Limite: %d", temperatura_atual, temp_alarme);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tarefa_botoes(void *pv) {
    evento_t evt;
    while (1) {
        if (xQueueReceive(fila_eventos, &evt, pdMS_TO_TICKS(10))) {
            int64_t agora = esp_timer_get_time();
            if (agora - ultimo_evento > 200000) {
                ultimo_evento = agora;
                if (evt == BTN_INC_EVT) temp_alarme += 5;
                else if (evt == BTN_DEC_EVT) temp_alarme -= 5;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void tarefa_buzzer(void *pv) {
    while (1) {
        ativar_buzzer(temperatura_atual >= temp_alarme);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void tarefa_lcd(void *pv) {
    static int last_temp = -999;
    static int last_limite = -999;

    while (1) {
        if (last_temp != temperatura_atual || last_limite != temp_alarme) {
            atualizar_lcd(temperatura_atual, temp_alarme);
            last_temp = temperatura_atual;
            last_limite = temp_alarme;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tarefa_leds(void *pv) {
    while (1) {
        atualizar_leds(temperatura_atual, temp_alarme);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void tarefa_sdcard(void *pv) {
    while (1) {
        salvar_leitura_sd(temperatura_atual, temp_alarme);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void mostrar_digito(char c) {
    bool seg[8] = {0};
    switch (c) {
        case '0': seg[0] = seg[1] = seg[2] = seg[3] = seg[4] = seg[5] = 1; break;
        case '3': seg[0] = seg[1] = seg[2] = seg[3] = seg[6] = 1; break;
        case '7': seg[0] = seg[1] = seg[2] = 1; break;
        case 'D': seg[1] = seg[2] = seg[3] = seg[4] = seg[6] = 1; break;
        case 'F': seg[0] = seg[4] = seg[5] = seg[6] = 1; break;
        default: for (int i = 0; i < 8; i++) seg[i] = 0; break;
    }
    gpio_set_level(16, !seg[0]);
    gpio_set_level(17, !seg[1]);
    gpio_set_level(18, !seg[2]);
    gpio_set_level(19, !seg[3]);
    gpio_set_level(20, !seg[4]);
    gpio_set_level(21, !seg[5]);
    gpio_set_level(11, !seg[6]);
    gpio_set_level(47, !seg[7]);
}

void tarefa_display7seg(void *pv) {
    while (1) {
        int diff = temp_alarme - temperatura_atual;
        static bool pisca = false;
        static int64_t anterior = 0;
        int64_t agora = esp_timer_get_time();

        if (temperatura_atual >= temp_alarme) {
            if (agora - anterior > 500000) {
                anterior = agora;
                pisca = !pisca;
                mostrar_digito(pisca ? 'F' : ' ');
            }
        } else if (diff >= 20) mostrar_digito('0');
        else if (diff >= 15) mostrar_digito('3');
        else if (diff >= 10) mostrar_digito('7');
        else if (diff >= 2) mostrar_digito('D');
        else mostrar_digito('F');

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED4, GPIO_MODE_OUTPUT);

    configurar_lcd();
    configurar_botoes();
    configurar_buzzer();
    gpio_set_direction(16, GPIO_MODE_OUTPUT);
    gpio_set_direction(17, GPIO_MODE_OUTPUT);
    gpio_set_direction(18, GPIO_MODE_OUTPUT);
    gpio_set_direction(19, GPIO_MODE_OUTPUT);
    gpio_set_direction(20, GPIO_MODE_OUTPUT);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);
    gpio_set_direction(11, GPIO_MODE_OUTPUT);
    gpio_set_direction(47, GPIO_MODE_OUTPUT);
    iniciar_sdcard();

    fila_eventos = xQueueCreate(10, sizeof(evento_t));

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_9, ADC_ATTEN_DB_11);

    xTaskCreate(tarefa_leitura_temp, "leitura_temp", 1024, NULL, 2, NULL);
    xTaskCreate(tarefa_botoes, "botoes", 1024, NULL, 2, NULL);
    xTaskCreate(tarefa_buzzer, "buzzer", 1280, NULL, 1, NULL);
    xTaskCreate(tarefa_lcd, "lcd", 2048, NULL, 1, NULL);
    xTaskCreate(tarefa_leds, "leds", 1024, NULL, 1, NULL);
    xTaskCreate(tarefa_sdcard, "sdcard", 2048, NULL, 1, NULL);
    xTaskCreate(tarefa_display7seg, "disp7seg", 1024, NULL, 1, NULL);
}
