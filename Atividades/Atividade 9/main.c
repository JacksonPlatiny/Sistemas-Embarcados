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
static int temp_alarme = TEMP_DEFAULT_ALARM;
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

typedef enum {
    ESTADO_INICIALIZAR,
    ESTADO_ESPERAR_EVENTO,
    ESTADO_PROCESSAR_EVENTO,
    ESTADO_LEITURA_SENSOR,
    ESTADO_ATUALIZAR_INTERFACE
} estado_t;

static estado_t estado_atual = ESTADO_INICIALIZAR;
static evento_t ultimo_evento_recebido;
static bool evento_disponivel = false;
static int temperatura = 0;

static void IRAM_ATTR isr_btn_inc(void* arg) {
    gpio_isr_handler_remove(GPIO_BTN_INC);
    evento_t evt = BTN_INC_EVT;
    xQueueSendFromISR(fila_eventos, &evt, NULL);
}

static void IRAM_ATTR isr_btn_dec(void* arg) {
    gpio_isr_handler_remove(GPIO_BTN_DEC);
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
    lcd_i2c_print(&lcd, "Temp: %2d C     ", temp);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "Limite: %2d C   ", limite);
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

void ler_arquivo_sd() {
    FILE *f = fopen("/sdcard/leituras.txt", "r");
    if (f == NULL) {
        ESP_LOGE("SDCARD", "Erro ao abrir arquivo para leitura");
        return;
    }

    char linha[128];
    ESP_LOGI("SDCARD", "--- Conteúdo do arquivo ---");
    while (fgets(linha, sizeof(linha), f)) {
        printf("%s", linha);
    }
    ESP_LOGI("SDCARD", "--- Fim do arquivo ---");
    fclose(f);
}

void app_main(void) {
    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED4, GPIO_MODE_OUTPUT);

    fila_eventos = xQueueCreate(10, sizeof(evento_t));

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_9, ADC_ATTEN_DB_11);

    while (1) {
        switch (estado_atual) {
            case ESTADO_INICIALIZAR:
                configurar_lcd();
                configurar_botoes();
                configurar_buzzer();
                iniciar_sdcard();
                estado_atual = ESTADO_ESPERAR_EVENTO;
                break;

            case ESTADO_ESPERAR_EVENTO:
                if (xQueueReceive(fila_eventos, &ultimo_evento_recebido, pdMS_TO_TICKS(10))) {
                    evento_disponivel = true;
                    estado_atual = ESTADO_PROCESSAR_EVENTO;
                } else {
                    estado_atual = ESTADO_LEITURA_SENSOR;
                }
                break;

            case ESTADO_PROCESSAR_EVENTO:
                if (evento_disponivel) {
                    int64_t agora = esp_timer_get_time();
                    if (agora - ultimo_evento > 200000) {
                        ultimo_evento = agora;
                        if (ultimo_evento_recebido == BTN_INC_EVT) temp_alarme += 5;
                        else if (ultimo_evento_recebido == BTN_DEC_EVT) temp_alarme -= 5;
                        gpio_isr_handler_add(GPIO_BTN_INC, isr_btn_inc, NULL);
                        gpio_isr_handler_add(GPIO_BTN_DEC, isr_btn_dec, NULL);
                    }
                    evento_disponivel = false;
                }
                estado_atual = ESTADO_LEITURA_SENSOR;
                break;

            case ESTADO_LEITURA_SENSOR:
                temperatura = ler_temperatura_ntc();
                ESP_LOGI(TAG, "Temp: %d | Limite: %d", temperatura, temp_alarme);
                salvar_leitura_sd(temperatura, temp_alarme);
                estado_atual = ESTADO_ATUALIZAR_INTERFACE;
                break;

            case ESTADO_ATUALIZAR_INTERFACE: {
                atualizar_leds(temperatura, temp_alarme);
                ativar_buzzer(temperatura >= temp_alarme);

                static int64_t ultima_att = 0;
                int64_t agora = esp_timer_get_time();
                if (agora - ultima_att > 1000000) {
                    atualizar_lcd(temperatura, temp_alarme);
                    ler_arquivo_sd();
                    ultima_att = agora;
                }

                estado_atual = ESTADO_ESPERAR_EVENTO;
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
            }

            default:
                estado_atual = ESTADO_INICIALIZAR;
                break;
        }
    }
}
