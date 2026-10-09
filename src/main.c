/*
 * ESP-IDF  |  DS18B20 + 16x2 I2C LCD + 4x4 keypad + Blue/Red LEDs
 *
 * Data flow:
 *   keypad column GPIO --(ISR)--> task notification --> keypad_task
 *   keypad_task --(queue of chars)--> ui_task --> LEDs + LCD
 *   sensor_task --(shared variable)--> ui_task (live temperature)
 *
 * Key '5' -> BLUE LED ON / OFF cycle     Key 'D' -> RED LED ON / OFF cycle
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "app";

/* ------------------------------ Pin map ---------------------------------- */
#define PIN_DS18B20   GPIO_NUM_4
#define PIN_SDA       GPIO_NUM_21
#define PIN_SCL       GPIO_NUM_22
#define PIN_LED_BLUE  GPIO_NUM_18
#define PIN_LED_RED   GPIO_NUM_19

static const gpio_num_t ROW_PINS[4] = { GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26 }; /* outputs */
static const gpio_num_t COL_PINS[4] = { GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_23 }; /* inputs + IRQ */
static const char KEYMAP[4][4] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '*', '0', '#', 'D' },
};

/* ------------------------------ Settings --------------------------------- */
#define I2C_PORT      I2C_NUM_0
#define LCD_ADDR      0x27
#define LED_ON_MS     3000
#define LED_OFF_MS    1500
#define TEMP_INVALID  INT32_MIN

/* ------------------------- Shared RTOS objects --------------------------- */
static TaskHandle_t   s_kp_task = NULL;          /* ISR notifies this task   */
static QueueHandle_t  s_key_q   = NULL;          /* keypad_task -> ui_task   */
static volatile int32_t s_temp_centi = TEMP_INVALID; /* temperature x100 (C) */

/* ========================================================================= */
/*  DS18B20 (bit-banged 1-Wire)                                              */
/* ========================================================================= */
static portMUX_TYPE s_ow_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void ow_low(void)     { gpio_set_level(PIN_DS18B20, 0); }
static inline void ow_release(void) { gpio_set_level(PIN_DS18B20, 1); }

static void ow_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_DS18B20,
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,   /* open-drain, readable */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    ow_release();
}

static bool ow_reset(void)
{
    ow_low();
    esp_rom_delay_us(480);
    portENTER_CRITICAL(&s_ow_mux);
    ow_release();
    esp_rom_delay_us(70);
    bool present = (gpio_get_level(PIN_DS18B20) == 0);
    portEXIT_CRITICAL(&s_ow_mux);
    esp_rom_delay_us(410);
    return present;
}

static void ow_write_bit(int bit)
{
    portENTER_CRITICAL(&s_ow_mux);
    ow_low();
    if (bit) { esp_rom_delay_us(6);  ow_release(); esp_rom_delay_us(64); }
    else     { esp_rom_delay_us(60); ow_release(); esp_rom_delay_us(10); }
    portEXIT_CRITICAL(&s_ow_mux);
}

static int ow_read_bit(void)
{
    portENTER_CRITICAL(&s_ow_mux);
    ow_low();
    esp_rom_delay_us(3);
    ow_release();
    esp_rom_delay_us(10);
    int b = gpio_get_level(PIN_DS18B20);
    esp_rom_delay_us(53);
    portEXIT_CRITICAL(&s_ow_mux);
    return b;
}

static void ow_write_byte(uint8_t v)
{
    for (int i = 0; i < 8; i++) { ow_write_bit(v & 1); v >>= 1; }
}

static uint8_t ow_read_byte(void)
{
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint8_t)(ow_read_bit() << i);
    return v;
}

static uint8_t ow_crc8(const uint8_t *d, int n)
{
    uint8_t crc = 0;
    while (n--) {
        uint8_t b = *d++;
        for (int i = 0; i < 8; i++) {
            uint8_t mix = (crc ^ b) & 1;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            b >>= 1;
        }
    }
    return crc;
}

/* Reads temperature in 1/100 degC. The 750 ms conversion wait uses
 * vTaskDelay(), so the CPU is free for other tasks meanwhile. */
static bool ds18b20_read(int32_t *centi)
{
    uint8_t sp[9];

    if (!ow_reset()) return false;
    ow_write_byte(0xCC);                    /* SKIP ROM        */
    ow_write_byte(0x44);                    /* CONVERT T       */
    vTaskDelay(pdMS_TO_TICKS(750));

    if (!ow_reset()) return false;
    ow_write_byte(0xCC);                    /* SKIP ROM        */
    ow_write_byte(0xBE);                    /* READ SCRATCHPAD */
    for (int i = 0; i < 9; i++) sp[i] = ow_read_byte();

    if (ow_crc8(sp, 8) != sp[8]) return false;

    int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);  /* 1/16 degC per LSB */
    *centi = ((int32_t)raw * 25) / 4;               /* raw*100/16        */
    return true;
}

static void sensor_task(void *arg)
{
    for (;;) {
        int32_t c;
        if (ds18b20_read(&c)) {
            s_temp_centi = c;
            ESP_LOGI(TAG, "Temperature: %.2f C", c / 100.0f);
        } else {
            s_temp_centi = TEMP_INVALID;
            ESP_LOGW(TAG, "DS18B20 read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ========================================================================= */
/*  16x2 LCD through PCF8574 I2C backpack (4-bit mode)                       */
/*  PCF8574 bits: P0=RS P1=RW P2=EN P3=Backlight P4..P7=D4..D7               */
/* ========================================================================= */
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BL 0x08

static void i2c_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_SDA,
        .scl_io_num       = PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0));
}

static void pcf_write(uint8_t v)
{
    i2c_master_write_to_device(I2C_PORT, LCD_ADDR, &v, 1, pdMS_TO_TICKS(50));
}

static void lcd_nibble(uint8_t nib, uint8_t rs)
{
    uint8_t d = (uint8_t)((nib << 4) | rs | LCD_BL);
    pcf_write(d | LCD_EN);          /* EN high */
    esp_rom_delay_us(2);
    pcf_write(d);                   /* EN low -> latch */
    esp_rom_delay_us(60);
}

static void lcd_send(uint8_t v, uint8_t rs)
{
    lcd_nibble(v >> 4, rs);
    lcd_nibble(v & 0x0F, rs);
}

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_nibble(0x3, 0); esp_rom_delay_us(5000);
    lcd_nibble(0x3, 0); esp_rom_delay_us(200);
    lcd_nibble(0x3, 0); esp_rom_delay_us(200);
    lcd_nibble(0x2, 0);                 /* switch to 4-bit        */
    lcd_send(0x28, 0);                  /* 2 lines, 5x8 font      */
    lcd_send(0x0C, 0);                  /* display on, no cursor  */
    lcd_send(0x01, 0);                  /* clear                  */
    esp_rom_delay_us(3000);
    lcd_send(0x06, 0);                  /* increment cursor       */
}

/* Writes exactly 16 characters (pads with spaces) so no clear is needed. */
static void lcd_print(uint8_t row, const char *s)
{
    lcd_send(0x80 | (row ? 0x40 : 0x00), 0);
    for (int i = 0; i < 16; i++) lcd_send(*s ? (uint8_t)*s++ : ' ', 1);
}

/* ========================================================================= */
/*  4x4 keypad - interrupt driven                                            */
/*  Idle: all ROWS driven LOW, COLUMNS pulled HIGH with falling-edge IRQ.    */
/*  Any key press pulls its column LOW -> ISR fires.                         */
/* ========================================================================= */
static void rows_all_low(void)
{
    for (int r = 0; r < 4; r++) gpio_set_level(ROW_PINS[r], 0);
}

static bool any_col_low(void)
{
    for (int c = 0; c < 4; c++) if (gpio_get_level(COL_PINS[c]) == 0) return true;
    return false;
}

static void cols_irq(bool enable)
{
    for (int c = 0; c < 4; c++) {
        if (enable) gpio_intr_enable(COL_PINS[c]);
        else        gpio_intr_disable(COL_PINS[c]);
    }
}

/* ISR: as short as possible - mask further IRQs and wake the keypad task. */
static void IRAM_ATTR col_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    cols_irq(false);
    vTaskNotifyGiveFromISR(s_kp_task, &hp);
    if (hp) portYIELD_FROM_ISR();
}

/* Row-by-row scan to identify which key is down. Returns 0 if none. */
static char scan_keypad(void)
{
    char key = 0;
    for (int r = 0; r < 4 && !key; r++) {
        for (int i = 0; i < 4; i++) gpio_set_level(ROW_PINS[i], 1);
        gpio_set_level(ROW_PINS[r], 0);
        esp_rom_delay_us(50);
        for (int c = 0; c < 4; c++) {
            if (gpio_get_level(COL_PINS[c]) == 0) { key = KEYMAP[r][c]; break; }
        }
    }
    rows_all_low();
    return key;
}

static void keypad_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);     /* sleep until ISR      */
        vTaskDelay(pdMS_TO_TICKS(20));               /* debounce             */

        char key = scan_keypad();
        if (key) {
            ESP_LOGI(TAG, "Key pressed: '%c'", key);
            xQueueSend(s_key_q, &key, 0);            /* hand over to UI task */
            while (any_col_low()) vTaskDelay(pdMS_TO_TICKS(20)); /* release  */
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        cols_irq(true);                              /* re-arm interrupts    */
    }
}

static void keypad_init(void)
{
    xTaskCreate(keypad_task, "keypad", 3072, NULL, 6, &s_kp_task);

    gpio_config_t rows = { .mode = GPIO_MODE_OUTPUT };
    for (int r = 0; r < 4; r++) rows.pin_bit_mask |= 1ULL << ROW_PINS[r];
    gpio_config(&rows);
    rows_all_low();

    gpio_config_t cols = {
        .mode       = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type  = GPIO_INTR_NEGEDGE,
    };
    for (int c = 0; c < 4; c++) cols.pin_bit_mask |= 1ULL << COL_PINS[c];
    gpio_config(&cols);

    gpio_install_isr_service(0);
    for (int c = 0; c < 4; c++) gpio_isr_handler_add(COL_PINS[c], col_isr, NULL);
}

/* ========================================================================= */
/*  UI task: owns the LCD and the LEDs                                       */
/* ========================================================================= */
static void show_temp(void)
{
    char buf[17];
    int32_t c = s_temp_centi;
    if (c == TEMP_INVALID) snprintf(buf, sizeof buf, "Temp: --.--%cC", 0xDF);
    else                   snprintf(buf, sizeof buf, "Temp:%7.2f%cC", c / 100.0f, 0xDF);
    lcd_print(1, buf);
}

/* Wait while still refreshing the live temperature on row 2. */
static void hold_ms(uint32_t ms)
{
    while (ms) {
        show_temp();
        uint32_t step = (ms > 250) ? 250 : ms;
        vTaskDelay(pdMS_TO_TICKS(step));
        ms -= step;
    }
}

static void led_cycle(gpio_num_t led, const char *on_msg, const char *off_msg)
{
    gpio_set_level(led, 1);  lcd_print(0, on_msg);   hold_ms(LED_ON_MS);
    gpio_set_level(led, 0);  lcd_print(0, off_msg);  hold_ms(LED_OFF_MS);
    lcd_print(0, "Ambient Temp");
}

static void ui_task(void *arg)
{
    lcd_print(0, "Ambient Temp");
    for (;;) {
        char key;
        if (xQueueReceive(s_key_q, &key, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (key == '5')      led_cycle(PIN_LED_BLUE, "BLUE LED ON", "BLUE LED OFF");
            else if (key == 'D') led_cycle(PIN_LED_RED,  "RED LED ON",  "RED LED OFF");
        }
        show_temp();             /* normal mode: refresh every 500 ms */
    }
}

/* ========================================================================= */
void app_main(void)
{
    s_key_q = xQueueCreate(8, sizeof(char));

    gpio_config_t led = {
        .pin_bit_mask = (1ULL << PIN_LED_BLUE) | (1ULL << PIN_LED_RED),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led);
    gpio_set_level(PIN_LED_BLUE, 0);
    gpio_set_level(PIN_LED_RED, 0);

    i2c_init();
    lcd_init();
    lcd_print(0, "Starting...");
    ow_init();

    xTaskCreate(sensor_task, "sensor", 3072, NULL, 5, NULL);
    xTaskCreate(ui_task,     "ui",     4096, NULL, 4, NULL);
    keypad_init();               /* last: IRQs go live only when tasks exist */
}
