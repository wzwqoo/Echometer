/**
 ******************************************************************************
 * main.c — Ultrasonic detector + recorder  (ESP32-S3, ESP-IDF, FreeRTOS)
 *
 * Port of the STM32F446 "echometer" firmware (see main_stm.c) to the ESP32-S3.
 *
 * STATE_DETECT     : ADC stream -> FFT every 2048 samples -> any bin above
 *                    19 kHz over threshold -> RGB LED flashes GREEN.
 * STATE_RECORD_SD  : ADC -> FatFS file on SD card  -> RGB LED flashes BLUE.
 * STATE_RECORD_USB : ADC -> USB CDC serial stream  -> RGB LED flashes RED.
 *
 * BOOT button (GPIO0, active-low): first press starts recording — to SD if a
 * card mounts, otherwise streams over USB. Second press stops and returns to
 * detect.
 *
 * Wiring
 *   ADC   : external ADCS7476 (12-bit, 1 MSPS SAR) read via I2S "WS-as-CS":
 *             CS  <- GPIO4  (I2S WS)
 *             SCLK<- GPIO5  (I2S BCK)
 *             SDATA-> GPIO6 (I2S DIN)
 *   SD SPI: CS=GPIO10  CLK=GPIO12  MISO=GPIO13  MOSI=GPIO11   (SPI2_HOST)
 *   USB   : D+=GPIO19  D-=GPIO20   (native USB-OTG PHY, fixed function)
 *   RGB   : GPIO48    (addressable WS2812)
 *   BTN   : GPIO0     (BOOT button)
 *
 * ADC CLOCKING: the S3's built-in ADC-DMA tops out at ~83 ksps, too slow for
 * the 120 kHz ultrasonic band. We instead read an external ADCS7476 using the
 * I2S peripheral as a hardware frame generator: I2S is master, 16-bit stereo,
 * so WS is a square wave that toggles every 16 BCK. WS drives the ADC's CS
 * (low for 16 clocks = one conversion + readout, high for 16 = re-track), and
 * I2S RX DMA captures SDATA jitter-free with zero per-sample CPU. At Fs =
 * 240 ksps the BCK is 240000*2*16 = 7.68 MHz and usable Nyquist is 120 kHz.
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
#include "driver/spi_common.h"
#include "driver/i2s_std.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "led_strip.h"

#include "esp_dsp.h"

static const char *TAG = "echometer";

/* ── Pin map ─────────────────────────────────────────────────────────────── */
#define PIN_ADC_CS          GPIO_NUM_4        /* I2S WS  -> ADCS7476 CS       */
#define PIN_ADC_CLK         GPIO_NUM_5        /* I2S BCK -> ADCS7476 SCLK     */
#define PIN_ADC_SDATA       GPIO_NUM_6        /* I2S DIN <- ADCS7476 SDATA    */
#define ADC_I2S_PORT        I2S_NUM_0
#define PIN_SD_CS           GPIO_NUM_10
#define PIN_SD_CLK          GPIO_NUM_12
#define PIN_SD_MISO         GPIO_NUM_13
#define PIN_SD_MOSI         GPIO_NUM_11
#define PIN_RGB             GPIO_NUM_48
#define PIN_BTN             GPIO_NUM_0
#define SD_SPI_HOST         SPI2_HOST
#define SD_MOUNT_POINT      "/sdcard"

/* ── Configuration ───────────────────────────────────────────────────────── */
#define SAMPLE_RATE_HZ      240000U           /* external ADCS7476 via I2S    */
#define HALF_BUF_SAMPLES    2048U
#define FFT_SIZE            HALF_BUF_SAMPLES

/* ADCS7476 serial word = 4 leading zeros + 12 data bits over 16 SCLK, so a
 * raw 16-bit I2S slot masked with 0x0FFF is the sample. If a scope shows the
 * data is off by a bit (I2S MSB is delayed 1 BCK vs the ADC), bump this to 1. */
#define ADC_SAMPLE_SHIFT    0

/* First FFT bin with centre freq > 19 kHz:
 *   k_min = ceil(19000 * FFT_SIZE / SAMPLE_RATE)
 *   res   = SAMPLE_RATE / FFT_SIZE = 240000/2048 ~= 117.19 Hz/bin
 *   19000 * 2048 / 240000 = 162.1  ->  163                                   */
#define ULTRASONIC_BIN_MIN  ((uint32_t)((19000UL * FFT_SIZE + SAMPLE_RATE_HZ - 1U) \
                              / SAMPLE_RATE_HZ))   /* 163 */

/* Detection threshold as a per-bin tone amplitude in ADC counts. An unnormalised
 * FFT maps a real tone of amplitude A counts to a bin magnitude of ~A*N/2, so we
 * pre-scale and compare squared magnitudes (avoids a per-bin sqrt).
 * 12-bit ADC full-scale = 4095 counts; raise if false-triggers, lower if misses. */
#define ULTRASONIC_THRESH_COUNTS  50.0f
#define ULTRASONIC_THRESH_SQ      ((ULTRASONIC_THRESH_COUNTS * (FFT_SIZE / 2.0f)) \
                                 * (ULTRASONIC_THRESH_COUNTS * (FFT_SIZE / 2.0f)))

#define FATFS_SYNC_EVERY    100U              /* fsync every N writes (~2.5 s) */
#define BTN_DEBOUNCE_US     50000             /* 50 ms                         */

#define LED_FAST_MS         100U
#define LED_SLOW_MS         250U
#define LED_BRIGHTNESS      40                /* 0..255 per channel            */

/* ── Types ───────────────────────────────────────────────────────────────── */
typedef enum { STATE_DETECT, STATE_RECORD_SD, STATE_RECORD_USB } AppState;
typedef enum { LED_OFF, LED_GREEN, LED_BLUE, LED_RED } LedMode;

/* ── Application state ───────────────────────────────────────────────────── */
static volatile AppState g_state    = STATE_DETECT;
static volatile LedMode  g_led_mode = LED_OFF;
static volatile uint8_t  g_btn_event = 0;     /* set by ISR, cleared by task  */

/* ── Buffers (static — keep off task stacks) ─────────────────────────────── */
static uint16_t   sample_buf[HALF_BUF_SAMPLES];              /* raw ADC counts */
static float      fft_buf[FFT_SIZE * 2];                     /* interleaved re,im */

static i2s_chan_handle_t  i2s_rx    = NULL;
static led_strip_handle_t led_strip = NULL;

/* I2S delivers 16-bit stereo frames: [L, R] per conversion. The ADC readout
 * happens while WS is low (the "left" slot), so we keep the left samples and
 * discard the right (idle re-track) half. One buffer = HALF_BUF_SAMPLES frames. */
static int16_t i2s_raw[HALF_BUF_SAMPLES * 2];

/* ======================================================================
 * RGB LED task — non-blocking blink driven by g_led_mode
 * ==================================================================== */
static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

static void led_task(void *arg)
{
    (void)arg;
    bool on = false;
    for (;;) {
        LedMode mode = g_led_mode;
        if (mode == LED_OFF) {
            led_strip_clear(led_strip);
            on = false;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        on = !on;
        if (!on) {
            led_strip_clear(led_strip);
        } else if (mode == LED_GREEN) {
            led_set(0, LED_BRIGHTNESS, 0);
        } else if (mode == LED_BLUE) {
            led_set(0, 0, LED_BRIGHTNESS);
        } else { /* LED_RED */
            led_set(LED_BRIGHTNESS, 0, 0);
        }
        uint32_t ms = (mode == LED_GREEN) ? LED_FAST_MS : LED_SLOW_MS;
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

/* ======================================================================
 * Button ISR — GPIO0 falling edge (active-low BOOT button), debounced
 * ==================================================================== */
static void IRAM_ATTR btn_isr(void *arg)
{
    (void)arg;
    static int64_t last_us = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_us < BTN_DEBOUNCE_US) return;
    last_us = now;
    g_btn_event = 1;
}

/* ======================================================================
 * run_fft_detect — real FFT on 2048 samples, returns 1 if ultrasonic found.
 * Uses esp-dsp radix-2 complex FFT with the imaginary part zeroed.
 * ==================================================================== */
static uint8_t run_fft_detect(const uint16_t *samples)
{
    /* Remove DC offset, load interleaved complex (imag = 0). */
    float mean = 0.0f;
    for (uint32_t i = 0; i < FFT_SIZE; i++) mean += (float)samples[i];
    mean /= (float)FFT_SIZE;

    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        fft_buf[2 * i]     = (float)samples[i] - mean;
        fft_buf[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(fft_buf, FFT_SIZE);
    dsps_bit_rev_fc32(fft_buf, FFT_SIZE);

    for (uint32_t k = ULTRASONIC_BIN_MIN; k < FFT_SIZE / 2U; k++) {
        float re = fft_buf[2 * k];
        float im = fft_buf[2 * k + 1];
        if (re * re + im * im > ULTRASONIC_THRESH_SQ) return 1U;
    }
    return 0U;
}

/* ======================================================================
 * Peripheral init
 * ==================================================================== */
static void adc_i2s_init(void)
{
    /* I2S RX master: generates BCK (SCLK) + WS (CS) and clocks SDATA in.
     * 16-bit stereo @ Fs -> BCK = Fs * 2 slots * 16 bits = 7.68 MHz @ 240 ksps,
     * WS toggles every 16 BCK to frame each ADCS7476 conversion. */
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

static void usb_init(void)
{
    /* Native USB-OTG PHY on GPIO19/20; enumerates as a CDC-ACM serial port.
     * NOTE: this takes the USB pins away from the USB-Serial-JTAG console, so
     * the console must be routed to UART0 (see menuconfig notes). */
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor        = NULL,   /* use defaults */
        .string_descriptor        = NULL,
        .external_phy             = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    const tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev          = TINYUSB_USBDEV_0,
        .cdc_port         = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx      = NULL,
        .callback_rx_wanted_char        = NULL,
        .callback_line_state_changed    = NULL,
        .callback_line_coding_changed   = NULL,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
}

/* Stream one buffer over USB CDC in FIFO-sized chunks. */
static void usb_write(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    while (len) {
        size_t n = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, p, len);
        if (n == 0) {
            /* FIFO full — push it out and retry. */
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(50));
            continue;
        }
        p   += n;
        len -= n;
    }
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(50));
}

/* ======================================================================
 * SD helpers — SPI2 bus is initialised once; mount/unmount on demand.
 * ==================================================================== */
static sdmmc_card_t *s_card = NULL;

static void sd_bus_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = PIN_SD_MOSI,
        .miso_io_num     = PIN_SD_MISO,
        .sclk_io_num     = PIN_SD_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SD_SPI_HOST, &bus, SDSPI_DEFAULT_DMA));
}

/* Returns true if a card mounts at SD_MOUNT_POINT. */
static bool sd_mount(void)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = PIN_SD_CS;
    slot.host_id = SD_SPI_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 3,
        .allocation_unit_size   = 16 * 1024,
    };

    esp_err_t err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot,
                                            &mount_cfg, &s_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed (%s)", esp_err_to_name(err));
        s_card = NULL;
        return false;
    }
    return true;
}

static void sd_unmount(void)
{
    if (s_card) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        s_card = NULL;
    }
}

/* ======================================================================
 * Read exactly one HALF_BUF_SAMPLES frame from the ADC into sample_buf.
 * ==================================================================== */
static void adc_read_frame(void)
{
    size_t need = sizeof(i2s_raw), got = 0;
    while (got < need) {
        size_t rd = 0;
        if (i2s_channel_read(i2s_rx, (uint8_t *)i2s_raw + got, need - got,
                             &rd, portMAX_DELAY) == ESP_OK)
            got += rd;
    }
    /* Even index = left slot = WS-low window = valid ADCS7476 readout. */
    for (uint32_t i = 0; i < HALF_BUF_SAMPLES; i++) {
        uint16_t w = (uint16_t)i2s_raw[2 * i];
        sample_buf[i] = (w >> ADC_SAMPLE_SHIFT) & 0x0FFF;
    }
}

/* ======================================================================
 * Main task — DETECT <-> RECORD state machine.
 * ==================================================================== */
static void main_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "=== echometer boot (S3) ===  bin_min=%u  Fs=%u",
             (unsigned)ULTRASONIC_BIN_MIN, (unsigned)SAMPLE_RATE_HZ);

    FILE     *fp = NULL;
    uint32_t  write_count  = 0;
    uint32_t  file_counter = 0;
    char      path[48];

    for (;;) {
        adc_read_frame();

        if (g_btn_event) {
            g_btn_event = 0;
            ESP_LOGI(TAG, "BTN (state=%d)", (int)g_state);

            if (g_state == STATE_DETECT) {
                if (sd_mount()) {
                    /* Find next unused REC####.BIN */
                    struct stat st;
                    do {
                        snprintf(path, sizeof(path),
                                 SD_MOUNT_POINT "/REC%04u.BIN",
                                 (unsigned)++file_counter);
                    } while (stat(path, &st) == 0 && file_counter < 9999U);

                    fp = fopen(path, "wb");
                    if (fp) {
                        write_count = 0;
                        g_state     = STATE_RECORD_SD;
                        g_led_mode  = LED_BLUE;
                        ESP_LOGI(TAG, "REC->SD %s", path);
                    } else {
                        ESP_LOGW(TAG, "fopen failed");
                        sd_unmount();
                    }
                } else {
                    /* No SD — stream over USB instead. */
                    g_state    = STATE_RECORD_USB;
                    g_led_mode = LED_RED;
                    ESP_LOGI(TAG, "REC->USB");
                }
            } else if (g_state == STATE_RECORD_SD) {
                if (fp) { fflush(fp); fsync(fileno(fp)); fclose(fp); fp = NULL; }
                sd_unmount();
                g_state    = STATE_DETECT;
                g_led_mode = LED_OFF;
                ESP_LOGI(TAG, "REC STOP (%lu blocks)", (unsigned long)write_count);
            } else { /* STATE_RECORD_USB */
                g_state    = STATE_DETECT;
                g_led_mode = LED_OFF;
                ESP_LOGI(TAG, "USB STOP");
            }
        }

        if (g_state == STATE_DETECT) {
            uint8_t det = run_fft_detect(sample_buf);
            g_led_mode = det ? LED_GREEN : LED_OFF;
        } else if (g_state == STATE_RECORD_SD) {
            size_t n = fp ? fwrite(sample_buf, 1, sizeof(sample_buf), fp) : 0;
            if (n != sizeof(sample_buf)) {
                ESP_LOGW(TAG, "fwrite short (%u)", (unsigned)n);
                if (fp) { fclose(fp); fp = NULL; }
                sd_unmount();
                g_state    = STATE_DETECT;
                g_led_mode = LED_OFF;
            } else if (++write_count % FATFS_SYNC_EVERY == 0) {
                fflush(fp);
                fsync(fileno(fp));
            }
        } else { /* STATE_RECORD_USB */
            usb_write(sample_buf, sizeof(sample_buf));
        }
    }
}

void app_main(void)
{
    /* esp-dsp radix-2 FFT tables (reused for every transform). */
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));

    led_init();
    btn_init();
    sd_bus_init();
    usb_init();
    adc_i2s_init();

    xTaskCreatePinnedToCore(led_task,  "led",  2048, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(main_task, "main", 8192, NULL, 5, NULL, tskNO_AFFINITY);
}
