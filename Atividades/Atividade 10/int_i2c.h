#include "driver/i2c.h"
#include "rom/ets_sys.h"

//Biblioteca fornecida por luizfilipemr

// INÍCIO DA BIBLIOTECA DO LCD -------------------------------------------

// PINOS DO PCF8475
#define RS 0
#define RW 1
#define EN 2
#define BL 3

// MEDIDAS DOS DISPLAYS
#define DISPLAY_16X02 0
#define DISPLAY_20X04 1

// INSTRUÇÕES DO DISPLAY LCD
#define CLEAR_DISPLAY               0x01
#define RETURN_HOME_UNSHIFT         0x02
#define CURSOR_RIGHT_NO_SHIFT       0x04
#define CURSOR_RIGHT_SHIFT          0x05
#define CURSOR_RIGHT_NO_SHIFT_LEFT  0x06
#define CURSOR_RIGHT_SHIFT_LEFT     0x07
#define DISPLAY_OFF                 0x08
#define DISPLAY_ON_CURSOR_OFF       0x0C
#define DISPLAY_ON_CURSOR_ON_STEADY 0x0E
#define DISPLAY_ON_CURSOR_ON_BLINK  0x0F
#define RETURN_HOME                 0x80
#define SHIFT_CURSOR_LEFT           0x10
#define SHIFT_CURSOR_RIGHT          0x14
#define SHIFT_DISPLAY_LEFT          0x18
#define SHIFT_DISPLAY_RIGHT         0x1C
#define SET_4BIT_MODE               0x28

typedef struct {
  uint8_t address;
  uint8_t num;
  uint8_t backlight;
  uint8_t size;
} lcd_i2c_handle_t;

void i2c_write_byte(lcd_i2c_handle_t * lcd, uint8_t data) {
  i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
  i2c_master_start(i2c_cmd);
  i2c_master_write_byte(i2c_cmd, (lcd->address << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(i2c_cmd, data, 1);
  i2c_master_stop(i2c_cmd);
  i2c_master_cmd_begin(lcd->num, i2c_cmd, 10 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(i2c_cmd);
}

void lcd_i2c_write(lcd_i2c_handle_t * lcd, char rs_flag, char data_byte) {
  uint8_t buffer_byte = ((1 << RS) * rs_flag) | ((1 << BL) * lcd->backlight);

  buffer_byte |= (buffer_byte & 0x0F) | (0xF0 & data_byte);
  buffer_byte |= (1 << EN);
  i2c_write_byte(lcd, buffer_byte);
  ets_delay_us(10);
  buffer_byte &= ~(1 << EN);
  i2c_write_byte(lcd, buffer_byte);
  ets_delay_us(50);

  buffer_byte = (buffer_byte & 0x0F) | (data_byte << 4);
  buffer_byte |= (1 << EN);
  i2c_write_byte(lcd, buffer_byte);
  ets_delay_us(10);
  buffer_byte &= ~(1 << EN);
  i2c_write_byte(lcd, buffer_byte);
  ets_delay_us(50);

  if (data_byte == CLEAR_DISPLAY) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void lcd_i2c_init(lcd_i2c_handle_t * lcd) {
  vTaskDelay(10 / portTICK_PERIOD_MS);
  lcd_i2c_write(lcd, 0, RETURN_HOME_UNSHIFT);
  lcd_i2c_write(lcd, 0, SET_4BIT_MODE);
  lcd_i2c_write(lcd, 0, CLEAR_DISPLAY);
  lcd_i2c_write(lcd, 0, DISPLAY_ON_CURSOR_OFF);
  lcd_i2c_write(lcd, 0, CURSOR_RIGHT_NO_SHIFT_LEFT);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lcd_i2c_cursor_set(lcd_i2c_handle_t * lcd, uint8_t column, uint8_t row) {
  if (lcd->size == DISPLAY_16X02) {
    if (row) lcd_i2c_write(lcd, 0, 0x80 + 0x40 + column);
    else lcd_i2c_write(lcd, 0, 0x80 + column);
  }

  else if (lcd->size == DISPLAY_20X04) {
    switch (row) {
      case 0:
        lcd_i2c_write(lcd, 0, 0x80 + column);
        break;

      case 1:
        lcd_i2c_write(lcd, 0, 0x80 + 0x40 + column);
        break;

      case 2:
        lcd_i2c_write(lcd, 0, 0x80 + 0x14 + column);
        break;

      case 3:
        lcd_i2c_write(lcd, 0, 0x80 + 0x54 + column);
        break;

      default:
        break;
    }
  }
}

void lcd_i2c_custom_char(lcd_i2c_handle_t * lcd, char char_address, const char * pixels) {
  lcd_i2c_write(lcd, 0, 0x40 | (char_address << 3));

  for (uint8_t i = 0; i < 8; i++) {
    lcd_i2c_write(lcd, 1, pixels[i]);
  }

  lcd_i2c_write(lcd, 0, RETURN_HOME);
}

void lcd_i2c_print(lcd_i2c_handle_t * lcd, const char * format_string, ...) {
  uint16_t i = 0;
  char buffer_string[128];

  va_list arguments;
  va_start(arguments, format_string);
  vsnprintf(buffer_string, sizeof(buffer_string), format_string, arguments);
  va_end(arguments);

  while (buffer_string[i] != '\0') {
    lcd_i2c_write(lcd, 1, buffer_string[i]);
    i++;
  }
}

void lcd_i2c_clear(lcd_i2c_handle_t *lcd) {
    lcd_i2c_write(lcd, 0, CLEAR_DISPLAY); 
    vTaskDelay(pdMS_TO_TICKS(2));        
}

// FIM DA BIBLIOTECA DO LCD ----------------------------------------------