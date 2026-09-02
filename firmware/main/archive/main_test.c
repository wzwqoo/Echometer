/**
 ******************************************************************************
 * main_test.c — minimal board bring-up test (ESP32-S3, ESP-IDF)
 *
 * IDLE    : LED off, ADC ignored.
 * ARMED   : BOOT button pressed -> sample ADC continuously, FFT every 2048
 *           samples, flash the RGB LED GREEN whenever any bin above 19 kHz
 *           exceeds the threshold. Press the button again to go back to IDLE.
 *
 * Also logs mean/min/max ADC counts and the peak bin once per second so you
 * can sanity-check the analog front end from the serial console.
 *
 * Wiring (same as main.c)
 *   ADC : ADCS7476 — CS<-GPIO4 (I2S WS), SCLK<-GPIO5 (I2S BCK), SDATA->GPIO6
 *   RGB : GPIO48 (WS2812)
 *   BTN : GPIO0  (BOOT, active-low)
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "led_strip.h"
#include "esp_dsp.h"

static const char *TAG = "brd-test";

/* ── Pin map ─────────────────────────────────────────────────────────────── */
#define PIN_ADC_CS      GPIO_NUM_4         /* I2S WS  -> ADCS7476 CS   */
#define PIN_ADC_CLK     GPIO_NUM_5         /* I2S BCK -> ADCS7476 SCLK */
#define PIN_ADC_SDATA   GPIO_NUM_6         /* I2S DIN <- ADCS7476 SDATA*/
#define ADC_I2S_PORT    I2S_NUM_0
#define PIN_RGB         GPIO_NUM_48
#define PIN_BTN         GPIO_NUM_0

/* ── Configuration (same numbers as main.c) ──────────────────────────────── */
#define SAMPLE_RATE_HZ      240000U
#define FFT_SIZE            2048U
#define ADC_SAMPLE_SHIFT    0

#define ULTRASONIC_BIN_MIN  ((uint32_t)((19000UL * FFT_SIZE + SAMPLE_RATE_HZ - 1U) \
                              / SAMPLE_RATE_HZ))   /* 163 */
#define ULTRASONIC_THRESH_COUNTS  50.0f
#define ULTRASONIC_THRESH_SQ      ((ULTRASONIC_THRESH_COUNTS * (FFT_SIZE / 2.0f)) \
                                 * (ULTRASONIC_THRESH_COUNTS * (FFT_SIZE / 2.0f)))

#define BTN_DEBOUNCE_US     50000
#define LED_BRIGHTNESS      40
#define GREEN_FLASH_MS      100U           /* min visible flash per detection */
#define STATS_PERIOD_US     1000000        /* console stats once per second   */

/* ── State / buffers ─────────────────────────────────────────────────────── */
static volatile uint8_t g_btn_event = 0;

static uint16_t sample_buf[FFT_SIZE];
static float    fft_buf[FFT_SIZE * 2];
static int16_t  i2s_raw[FFT_SIZE * 2];     /* stereo frames: keep left slot   */

static i2s_chan_handle_t  i2s_rx    = NULL;
static led_strip_handle_t led_strip = NULL;

/* ── LED ─────────────────────────────────────────────────────────────────── */
static void led_green(bool on)
{
    if (on) {
        led_strip_set_pixel(led_strip, 0, 0, LED_BRIGHTNESS, 0);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

/* ── Button ISR ──────────────────────────────────────────────────────────── */
static void IRAM_ATTR btn_isr(void *arg)
{
    (void)arg;
    static int64_t last_us = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_us < BTN_DEBOUNCE_US) return;
    last_us = now;
    g_btn_event = 1;
}

/* ── Peripheral init (copies of main.c) ──────────────────────────────────── */
static void adc_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(ADC_I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = 512;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &i2s_rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_ADC_CLK,
            .ws   = PIN_ADC_CS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_ADC_SDATA,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx));
}

static void led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = PIN_RGB,
        .max_leds       = 1,
        .led_model      = LED_MODEL_WS2812,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .flags = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags         = { .with_dma = false },
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led_strip));
    led_strip_clear(led_strip);
}

static void btn_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_BTN,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BTN, btn_isr, NULL));
}

/* ── One 2048-sample frame from the ADC ──────────────────────────────────── */
static void adc_read_frame(void)
{
    size_t need = sizeof(i2s_raw), got = 0;
    while (got < need) {
        size_t rd = 0;
        if (i2s_channel_read(i2s_rx, (uint8_t *)i2s_raw + got, need - got,
                             &rd, portMAX_DELAY) == ESP_OK)
            got += rd;
    }
    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        uint16_t w = (uint16_t)i2s_raw[2 * i];
        sample_buf[i] = (w >> ADC_SAMPLE_SHIFT) & 0x0FFF;
    }
}

/* ── FFT + peak search: returns 1 if any bin >19 kHz is over threshold ───── */
static uint8_t run_fft_detect(uint32_t *peak_bin, float *peak_mag_sq)
{
    float mean = 0.0f;
    for (uint32_t i = 0; i < FFT_SIZE; i++) mean += (float)sample_buf[i];
    mean /= (float)FFT_SIZE;

    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        fft_buf[2 * i]     = (float)sample_buf[i] - mean;
        fft_buf[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(fft_buf, FFT_SIZE);
    dsps_bit_rev_fc32(fft_buf, FFT_SIZE);

    uint8_t  det  = 0;
    uint32_t kmax = 0;
    float    mmax = 0.0f;
    for (uint32_t k = ULTRASONIC_BIN_MIN; k < FFT_SIZE / 2U; k++) {
        float re = fft_buf[2 * k];
        float im = fft_buf[2 * k + 1];
        float m  = re * re + im * im;
        if (m > mmax) { mmax = m; kmax = k; }
        if (m > ULTRASONIC_THRESH_SQ) det = 1U;
    }
    *peak_bin    = kmax;
    *peak_mag_sq = mmax;
    return det;
}

/* ── Main task ───────────────────────────────────────────────────────────── */
static void main_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "=== board test ===  Fs=%u  bin_min=%u  (press BOOT to arm)",
             (unsigned)SAMPLE_RATE_HZ, (unsigned)ULTRASONIC_BIN_MIN);

    bool    armed         = false;
    int64_t led_off_at_us = 0;
    int64_t next_stats_us = 0;

    for (;;) {
        if (g_btn_event) {
            g_btn_event = 0;
            armed = !armed;
            led_green(false);
            led_off_at_us = 0;
            ESP_LOGI(TAG, "%s", armed ? "ARMED — sampling + FFT" : "IDLE");
        }

        if (!armed) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        adc_read_frame();                          /* ~8.5 ms per buffer */

        uint32_t peak_bin;
        float    peak_sq;
        uint8_t  det = run_fft_detect(&peak_bin, &peak_sq);

        int64_t now = esp_timer_get_time();
        if (det) {
            led_green(true);
            led_off_at_us = now + (int64_t)GREEN_FLASH_MS * 1000;
        } else if (led_off_at_us && now > led_off_at_us) {
            led_green(false);
            led_off_at_us = 0;
        }

        if (now >= next_stats_us) {
            next_stats_us = now + STATS_PERIOD_US;
            uint16_t lo = 0xFFF, hi = 0;
            uint32_t sum = 0;
            for (uint32_t i = 0; i < FFT_SIZE; i++) {
                uint16_t s = sample_buf[i];
                if (s < lo) lo = s;
                if (s > hi) hi = s;
                sum += s;
            }
            /* mag_sq -> tone amplitude in ADC counts: A = 2*sqrt(m)/N */
            float amp = 2.0f * sqrtf(peak_sq) / (float)FFT_SIZE;
            ESP_LOGI(TAG, "adc min/avg/max=%u/%u/%u  peak %.1f kHz  amp=%.1f cnt%s",
                     (unsigned)lo, (unsigned)(sum / FFT_SIZE), (unsigned)hi,
                     (float)peak_bin * SAMPLE_RATE_HZ / FFT_SIZE / 1000.0f,
                     amp, det ? "  [DETECT]" : "");
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    led_init();
    btn_init();
    adc_i2s_init();
    xTaskCreatePinnedToCore(main_task, "main", 8192, NULL, 5, NULL, tskNO_AFFINITY);
}
