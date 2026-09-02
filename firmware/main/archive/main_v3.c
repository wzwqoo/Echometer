/**
 ******************************************************************************
 * main.c — ultrasonic chirp catcher, PSRAM + Wi-Fi edition (ESP32-S3)
 *
 * States
 *   IDLE   : LED off. Wi-Fi SoftAP + HTTP server up; captured chirps are
 *            listed at http://192.168.4.1 and downloadable as WAV in Safari.
 *   DETECT : after a BOOT-button press — ADC sampling + FFT, LED off while
 *            armed, flashing green while ultrasonic is being detected.
 *            Press again to cancel back to IDLE.
 *   RECORD : detection persists (see filters below) — LED solid blue while
 *            the clip streams into a PSRAM slot, then back to IDLE.
 *
 * Clips live in PSRAM only (lost at power-off): 3.84 s @ 256 ksps 16-bit =
 * ~1.97 MB each; an 8 MB (N16R8) module fits 4 slots, oldest is overwritten.
 * WAVs carry a GUANO metadata chunk and Wildlife-Acoustics-style filenames
 * (CHIRP_YYYYMMDD_HHMMSS.wav) so the Echo Meter iPhone app accepts them.
 * No RTC on board: the clock is seeded from firmware build time at boot.
 *
 * Wiring (same as before)
 *   ADC : ADCS7476 — CS<-GPIO4 (I2S WS), SCLK<-GPIO5 (I2S BCK), SDATA->GPIO6
 *   RGB : GPIO48 (WS2812)   BTN : GPIO0 (BOOT, active-low)
 ******************************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#include "led_strip.h"
#include "esp_dsp.h"

static const char *TAG = "chirp";

/* ── Pin map ─────────────────────────────────────────────────────────────── */
#define PIN_ADC_CS      GPIO_NUM_4
#define PIN_ADC_CLK     GPIO_NUM_5
#define PIN_ADC_SDATA   GPIO_NUM_6
#define ADC_I2S_PORT    I2S_NUM_0
#define PIN_RGB         GPIO_NUM_48
#define PIN_BTN         GPIO_NUM_0

/* ── Wi-Fi AP ────────────────────────────────────────────────────────────── */
#define AP_SSID         "echometer"        /* open network, no password */
#define AP_CHANNEL      1
#define AP_MAX_CONN     2

/* ── Sampling / FFT (same numbers as archive/main.c) ─────────────────────── */
#define SAMPLE_RATE_HZ      256000U
#define FFT_SIZE            2048U
#define ADC_SAMPLE_SHIFT    0

#define ULTRASONIC_BIN_MIN  ((uint32_t)((19000UL * FFT_SIZE + SAMPLE_RATE_HZ - 1U) \
                              / SAMPLE_RATE_HZ))   /* 152 */
#define ULTRASONIC_THRESH_COUNTS  50.0f
#define ULTRASONIC_THRESH_SQ      ((ULTRASONIC_THRESH_COUNTS * (FFT_SIZE / 2.0f)) \
                                 * (ULTRASONIC_THRESH_COUNTS * (FFT_SIZE / 2.0f)))

/* False-trigger rejection. A finger/contact transient is a broadband impulse:
 * it lifts the whole spectrum for a frame or two. A real chirp is narrowband
 * and sustained. So require both:
 *  - tonality: peak bin power >= TONALITY_MIN x the average power of the rest
 *    of the >19 kHz band (a flat broadband burst fails this), and
 *  - persistence: DETECT_CONSEC consecutive FFT frames (~8.5 ms each) must
 *    pass before recording starts. Lower to 1-2 if hunting very short clicks. */
#define CHIRP_TONALITY_MIN  20.0f
#define DETECT_CONSEC       3U

/* ── Clip storage ────────────────────────────────────────────────────────── */
/* 480 FFT buffers = 983 040 samples = 3.84 s @ 256 ksps = 1.97 MB per clip,
 * chosen so FOUR slots (7.5 MiB) fit in 8 MB PSRAM with headroom to spare. */
#define CLIP_BUFS           480U
#define CLIP_SAMPLES        (CLIP_BUFS * FFT_SIZE)
#define CLIP_BYTES          (CLIP_SAMPLES * sizeof(int16_t))
#define MAX_CLIP_SLOTS      8U
#define SPIRAM_RESERVE      (512U * 1024U) /* leave headroom for Wi-Fi/heap    */

#define BTN_DEBOUNCE_US     50000
#define LED_BRIGHTNESS      40

typedef enum { ST_IDLE, ST_DETECT, ST_RECORD } AppState;
typedef enum { LED_OFF, LED_GREEN_FLASH, LED_BLUE_FLASH, LED_ERR } LedMode;

typedef struct {
    int16_t      *data;                    /* CLIP_SAMPLES, in PSRAM          */
    volatile bool ready;                   /* false while being (re)written   */
    uint32_t      seq;                     /* global chirp number, 1..        */
    time_t        cap_time;                /* capture wall time               */
} clip_slot_t;

static clip_slot_t g_slots[MAX_CLIP_SLOTS];
static uint32_t    g_num_slots = 0;
static uint32_t    g_next_seq  = 1;

static volatile AppState g_state     = ST_IDLE;
static volatile LedMode  g_led_mode  = LED_OFF;
static volatile uint8_t  g_btn_event = 0;

static uint16_t sample_buf[FFT_SIZE];
static float    fft_buf[FFT_SIZE * 2];
static int16_t  i2s_raw[FFT_SIZE * 2];

static i2s_chan_handle_t  i2s_rx    = NULL;
static led_strip_handle_t led_strip = NULL;

/* ── 44-byte canonical WAV header (PCM mono 16-bit) ──────────────────────── */
typedef struct __attribute__((packed)) {
    char     riff[4];   uint32_t riff_size; char wave[4];
    char     fmt[4];    uint32_t fmt_size;
    uint16_t audio_fmt; uint16_t channels;
    uint32_t rate;      uint32_t byte_rate;
    uint16_t block_align; uint16_t bits;
    char     data[4];   uint32_t data_size;
} wav_header_t;

static void wav_header_fill(wav_header_t *h, uint32_t rate, uint32_t data_bytes)
{
    memcpy(h->riff, "RIFF", 4);  h->riff_size = 36 + data_bytes;
    memcpy(h->wave, "WAVE", 4);
    memcpy(h->fmt,  "fmt ", 4);  h->fmt_size = 16;
    h->audio_fmt = 1;  h->channels = 1;
    h->rate = rate;    h->byte_rate = rate * 2;
    h->block_align = 2; h->bits = 16;
    memcpy(h->data, "data", 4);  h->data_size = data_bytes;
}

/* No RTC/NTP on this board: seed the system clock from firmware build time
 * so capture timestamps and filenames are plausible and monotonic. */
static void clock_init(void)
{
    static const char mon[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char m[4] = { __DATE__[0], __DATE__[1], __DATE__[2], '\0' };
    struct tm tm = {
        .tm_mon  = (int)((strstr(mon, m) - mon) / 3),
        .tm_mday = atoi(__DATE__ + 4),
        .tm_year = atoi(__DATE__ + 7) - 1900,
        .tm_hour = atoi(__TIME__),
        .tm_min  = atoi(__TIME__ + 3),
        .tm_sec  = atoi(__TIME__ + 6),
    };
    struct timeval tv = { .tv_sec = mktime(&tm) };
    settimeofday(&tv, NULL);
}

/* Wildlife-Acoustics-style name: CHIRP_YYYYMMDD_HHMMSS.wav */
static void clip_filename(const clip_slot_t *s, char *out, size_t n)
{
    struct tm tm;
    localtime_r(&s->cap_time, &tm);
    snprintf(out, n, "CHIRP_%04d%02d%02d_%02d%02d%02d.wav",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/* GUANO metadata (https://guano-md.org) — Wildlife Acoustics apps use this
 * to recognise bat recordings. Returned length is always even (RIFF rule). */
static int guano_build(const clip_slot_t *s, char *out, size_t n)
{
    struct tm tm;
    localtime_r(&s->cap_time, &tm);
    /* The Echo Meter app refuses imports whose Model isn't a known WA module
     * ("invalid module type"), so identify as the module whose format we
     * match (256 kHz 16-bit full-spectrum). Real provenance is in Note. */
    int len = snprintf(out, n,
        "GUANO|Version: 1.0\n"
        "Make: Wildlife Acoustics, Inc.\n"
        "Model: Echo Meter Touch 2 Pro\n"
        "Note: recorded by DIY echometer-v3\n"
        "Samplerate: %u\n"
        "Length: %.2f\n"
        "Timestamp: %04d-%02d-%02dT%02d:%02d:%02d",
        (unsigned)SAMPLE_RATE_HZ, (float)CLIP_SAMPLES / SAMPLE_RATE_HZ,
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);
    if (len & 1) out[len++] = '\n';    /* pad to even length */
    out[len] = '\0';
    return len;
}

/* ── LED task ────────────────────────────────────────────────────────────────
 * The WS2812 is driven over RMT (GPIO48); every refresh is a burst of pulses
 * that can couple into the high-gain analog front end. So in LED_OFF we touch
 * the strip exactly ONCE (on entry) and then stay completely idle — this is
 * what keeps the LED silent through the RECORD window, where g_led_mode is set
 * to LED_OFF, so no periodic clicks are injected into the clip. Only the flash
 * modes refresh repeatedly, and those never overlap the sampling of a clip. */
static void led_task(void *arg)
{
    (void)arg;
    LedMode last = LED_ERR;      /* != LED_OFF, so the first OFF clears once */
    bool on = false;
    for (;;) {
        LedMode m = g_led_mode;
        if (m == LED_OFF) {
            if (m != last) { led_strip_clear(led_strip); on = false; }
            last = m;
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        /* LED_GREEN_FLASH / LED_BLUE_FLASH / LED_ERR — blink */
        on = !on;
        if (on) {
            uint8_t r = (m == LED_ERR)         ? LED_BRIGHTNESS : 0;
            uint8_t g = (m == LED_GREEN_FLASH) ? LED_BRIGHTNESS : 0;
            uint8_t b = (m == LED_BLUE_FLASH)  ? LED_BRIGHTNESS : 0;
            led_strip_set_pixel(led_strip, 0, r, g, b);
            led_strip_refresh(led_strip);
        } else {
            led_strip_clear(led_strip);
        }
        last = m;
        vTaskDelay(pdMS_TO_TICKS(m == LED_ERR ? 150 : 100));
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

/* ── Peripheral init ─────────────────────────────────────────────────────── */
static void adc_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(ADC_I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;            /* ~34 ms of DMA buffering          */
    chan_cfg.dma_frame_num = 1023;
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

/* ── PSRAM clip slots ────────────────────────────────────────────────────── */
static void slots_init(void)
{
    for (uint32_t i = 0; i < MAX_CLIP_SLOTS; i++) {
        if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < CLIP_BYTES + SPIRAM_RESERVE)
            break;
        int16_t *p = heap_caps_malloc(CLIP_BYTES, MALLOC_CAP_SPIRAM);
        if (!p) break;
        g_slots[i].data  = p;
        g_slots[i].ready = false;
        g_num_slots++;
    }
    ESP_LOGI(TAG, "PSRAM: %u clip slots of %u bytes (%.1f s each), %u KB free",
             (unsigned)g_num_slots, (unsigned)CLIP_BYTES,
             (float)CLIP_SAMPLES / SAMPLE_RATE_HZ,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
}

/* Pick the slot to record into: first empty, else the oldest (lowest seq). */
static clip_slot_t *slot_for_recording(void)
{
    if (g_num_slots == 0) return NULL;
    clip_slot_t *oldest = &g_slots[0];
    for (uint32_t i = 0; i < g_num_slots; i++) {
        if (!g_slots[i].ready && g_slots[i].seq == 0) return &g_slots[i];
        if (g_slots[i].seq < oldest->seq) oldest = &g_slots[i];
    }
    return oldest;
}

/* ── Wi-Fi SoftAP ────────────────────────────────────────────────────────── */
static void wifi_ap_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {
        .ap = {
            .ssid           = AP_SSID,
            .ssid_len       = strlen(AP_SSID),
            .channel        = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_OPEN,   /* no password */
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Full TX power (no cap): a weak link forces retransmits, which keep the
     * radio on longer and actually download slower. On a solid supply, run the
     * radio hot. If a marginal supply browns out, fix the power, not this. */
    ESP_LOGI(TAG, "SoftAP \"%s\" (open) -> http://192.168.4.1", AP_SSID);
}

/* ── HTTP server ─────────────────────────────────────────────────────────── */
static esp_err_t http_root(httpd_req_t *req)
{
    char line[256];
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><meta name=viewport "
        "content=\"width=device-width,initial-scale=1\">"
        "<title>echometer</title></head><body>"
        "<h2>echometer &mdash; captured chirps</h2>");

    snprintf(line, sizeof(line), "<p>state: %s &mdash; <a href=\"/\">refresh</a></p><ul>",
             g_state == ST_IDLE ? "idle (press button to arm)" :
             g_state == ST_DETECT ? "listening for ultrasonic..." : "recording");
    httpd_resp_sendstr_chunk(req, line);

    /* Newest first: descending-seq selection (n_slots <= 8, O(n^2) is fine). */
    uint32_t last_seq = UINT32_MAX;
    for (uint32_t pass = 0; pass < g_num_slots; pass++) {
        clip_slot_t *best = NULL;
        for (uint32_t i = 0; i < g_num_slots; i++) {
            clip_slot_t *s = &g_slots[i];
            if (!s->ready || s->seq >= last_seq) continue;
            if (!best || s->seq > best->seq) best = s;
        }
        if (!best) break;
        last_seq = best->seq;
        char fname[40];
        clip_filename(best, fname, sizeof(fname));
        snprintf(line, sizeof(line),
            "<li>%s &mdash; %.1f s @ %u kHz, %u KB &mdash; "
            "<a href=\"/dl?n=%u\">download</a> | "
            "<a href=\"/del?n=%u\" "
            "onclick=\"return confirm('Delete %s?')\">delete</a></li>",
            fname, (float)CLIP_SAMPLES / SAMPLE_RATE_HZ,
            (unsigned)(SAMPLE_RATE_HZ / 1000), (unsigned)(CLIP_BYTES / 1024),
            (unsigned)best->seq, (unsigned)best->seq, fname);
        httpd_resp_sendstr_chunk(req, line);
    }
    if (last_seq == UINT32_MAX)
        httpd_resp_sendstr_chunk(req, "<li>(no chirps captured yet)</li>");

    httpd_resp_sendstr_chunk(req, "</ul></body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static clip_slot_t *slot_from_query(httpd_req_t *req)
{
    char query[64], val[16];
    uint32_t seq = 0;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "n", val, sizeof(val)) == ESP_OK)
        seq = (uint32_t)strtoul(val, NULL, 10);

    for (uint32_t i = 0; i < g_num_slots; i++)
        if (g_slots[i].ready && g_slots[i].seq == seq) return &g_slots[i];
    return NULL;
}

static esp_err_t http_download(httpd_req_t *req)
{
    clip_slot_t *slot = slot_from_query(req);
    if (!slot) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such chirp");
        return ESP_FAIL;
    }

    char guano[192];
    int glen = guano_build(slot, guano, sizeof(guano));

    wav_header_t hdr;
    wav_header_fill(&hdr, SAMPLE_RATE_HZ, CLIP_BYTES);
    hdr.riff_size += 8 + (uint32_t)glen;   /* account for the guan chunk */

    char fname[40], disp[80];
    clip_filename(slot, fname, sizeof(fname));
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", fname);
    httpd_resp_set_type(req, "audio/wav");
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    esp_err_t err = httpd_resp_send_chunk(req, (const char *)&hdr, sizeof(hdr));
    const uint8_t *p = (const uint8_t *)slot->data;
    size_t left = CLIP_BYTES;
    while (err == ESP_OK && left) {
        size_t n = left > 8192 ? 8192 : left;
        err  = httpd_resp_send_chunk(req, (const char *)p, n);
        p    += n;
        left -= n;
    }
    if (err == ESP_OK) {                   /* trailing GUANO chunk */
        uint8_t chdr[8] = { 'g', 'u', 'a', 'n' };
        uint32_t gsz = (uint32_t)glen;
        memcpy(chdr + 4, &gsz, 4);
        err = httpd_resp_send_chunk(req, (const char *)chdr, sizeof(chdr));
    }
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, guano, glen);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, NULL, 0);
    return err;
}

static esp_err_t http_delete(httpd_req_t *req)
{
    clip_slot_t *slot = slot_from_query(req);
    if (slot) {
        ESP_LOGI(TAG, "deleted CHIRP seq %u via web", (unsigned)slot->seq);
        slot->ready = false;
        slot->seq   = 0;                   /* slot_for_recording reuses it first */
    }
    /* back to the list either way */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void http_init(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size       = 8192;
    cfg.lru_purge_enable = true;
    /* A 2 MB clip is a long transfer; if the link stalls briefly (a weak
     * moment, phone power-save), the default 5 s socket timeout aborts the
     * download mid-file. Give each send/recv much longer to ride out stalls. */
    cfg.send_wait_timeout = 30;
    cfg.recv_wait_timeout = 30;
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    static const httpd_uri_t root = { .uri = "/",    .method = HTTP_GET, .handler = http_root };
    static const httpd_uri_t dl   = { .uri = "/dl",  .method = HTTP_GET, .handler = http_download };
    static const httpd_uri_t del  = { .uri = "/del", .method = HTTP_GET, .handler = http_delete };
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &dl));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &del));
}

/* ── ADC frame + FFT detect ──────────────────────────────────────────────── */
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

static uint8_t run_fft_detect(void)
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

    /* Scan the >19 kHz band: track peak power and total band power. */
    float peak_sq = 0.0f, band_sum = 0.0f;
    for (uint32_t k = ULTRASONIC_BIN_MIN; k < FFT_SIZE / 2U; k++) {
        float re = fft_buf[2 * k];
        float im = fft_buf[2 * k + 1];
        float m  = re * re + im * im;
        band_sum += m;
        if (m > peak_sq) peak_sq = m;
    }

    if (peak_sq < ULTRASONIC_THRESH_SQ) return 0U;

    /* Tonality: peak must stand clear of the rest of the band. A broadband
     * impulse (finger on a floating pin) lifts every bin about equally, so
     * its peak/average ratio stays near 1 and it fails here. */
    uint32_t nbins = FFT_SIZE / 2U - ULTRASONIC_BIN_MIN;
    float avg_rest = (band_sum - peak_sq) / (float)(nbins - 1U);
    return (peak_sq >= CHIRP_TONALITY_MIN * avg_rest) ? 1U : 0U;
}

/* Convert one 12-bit unsigned frame to 16-bit signed PCM into the clip. */
static void frame_to_pcm(int16_t *dst)
{
    for (uint32_t i = 0; i < FFT_SIZE; i++)
        dst[i] = (int16_t)(((int32_t)sample_buf[i] - 2048) << 4);
}

/* ── Main task ───────────────────────────────────────────────────────────── */
static void main_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "=== chirp catcher ===  Fs=%u  bin_min=%u  clip=%u bufs",
             (unsigned)SAMPLE_RATE_HZ, (unsigned)ULTRASONIC_BIN_MIN,
             (unsigned)CLIP_BUFS);

    clip_slot_t *rec_slot   = NULL;
    uint32_t     rec_buf    = 0;
    uint32_t     det_streak = 0;
    int64_t      blue_until = 0;   /* post-record blue flash deadline (0 = off) */

    for (;;) {
        adc_read_frame();   /* always drain I2S DMA, ~8.5 ms per loop */

        /* Auto-end the post-record blue flash without blocking the ADC loop. */
        if (blue_until && esp_timer_get_time() > blue_until) {
            blue_until = 0;
            if (g_led_mode == LED_BLUE_FLASH) g_led_mode = LED_OFF;
        }

        if (g_btn_event) {
            g_btn_event = 0;
            if (g_state == ST_IDLE) {
                g_state    = ST_DETECT;
                g_led_mode = LED_OFF;      /* armed silently; LED only on detection */
                det_streak = 0;
                ESP_LOGI(TAG, "ARMED - listening for ultrasonic");
            } else if (g_state == ST_DETECT) {
                g_state    = ST_IDLE;
                g_led_mode = LED_OFF;
                ESP_LOGI(TAG, "disarmed");
            }
            /* button ignored during the 4 s RECORD */
        }

        if (g_state == ST_DETECT) {
            det_streak = run_fft_detect() ? det_streak + 1 : 0;
            g_led_mode = det_streak ? LED_GREEN_FLASH : LED_OFF;
            if (det_streak >= DETECT_CONSEC) {
                det_streak = 0;
                rec_slot = slot_for_recording();
                if (!rec_slot) {
                    ESP_LOGE(TAG, "no PSRAM slots!");
                    g_state    = ST_IDLE;
                    g_led_mode = LED_ERR;
                    continue;
                }
                rec_slot->ready = false;      /* hide from HTTP while writing */
                rec_slot->seq   = g_next_seq++;
                rec_slot->cap_time = time(NULL);
                frame_to_pcm(rec_slot->data); /* trigger buffer = clip start  */
                rec_buf    = 1;
                g_state    = ST_RECORD;
                g_led_mode = LED_OFF;          /* LED silent while sampling the clip */
                ESP_LOGI(TAG, "DETECT -> recording CHIRP%04u", (unsigned)rec_slot->seq);
            }
        } else if (g_state == ST_RECORD) {
            frame_to_pcm(rec_slot->data + (size_t)rec_buf * FFT_SIZE);
            if (++rec_buf >= CLIP_BUFS) {
                rec_slot->ready = true;
                ESP_LOGI(TAG, "CHIRP%04u done (%.2f s) -> idle, press button to re-arm",
                         (unsigned)rec_slot->seq,
                         (float)CLIP_SAMPLES / SAMPLE_RATE_HZ);
                rec_slot   = NULL;
                g_state    = ST_IDLE;
                g_led_mode = LED_BLUE_FLASH;   /* flash blue to confirm a saved clip */
                blue_until = esp_timer_get_time() + 1200000;   /* ~1.2 s */
            }
        }
        /* ST_IDLE: nothing — just keep draining the ADC and watching the button */
    }
}

void app_main(void)
{
    clock_init();
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    led_init();
    btn_init();
    slots_init();
    adc_i2s_init();

    xTaskCreatePinnedToCore(led_task,  "led",  2048, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(main_task, "main", 8192, NULL, 5, NULL, tskNO_AFFINITY);

    /* Bring the radio up last, well after the boot/PSRAM inrush has settled —
     * Wi-Fi RF calibration is the biggest current spike and marginal supplies
     * (USB filter dongles etc.) brown out when the spikes stack. */
    vTaskDelay(pdMS_TO_TICKS(2000));
    wifi_ap_init();
    http_init();
}
