/**
 ******************************************************************************
 * main.c — ultrasonic chirp catcher, v4 (ESP32-S3)
 *
 * v4 adds SWITCHED ANALOG GAIN on top of the v3 firmware. A 74HC4053 analog
 * mux picks the feedback resistors of both amplifier stages, giving six gain
 * steps from 22x to 782x (26.9 .. 57.9 dB, ~6 dB apart). The gain is settable
 * from the web page, or tracked automatically by an AGC with fast attack and
 * slow decay. See README.md for the resistor network and the SPL budget.
 *
 * States
 *   IDLE   : LED off. Wi-Fi SoftAP + HTTP server up; captured chirps are
 *            listed at http://192.168.4.1 and downloadable as WAV in Safari.
 *   DETECT : after a BOOT-button press — ADC sampling + FFT, LED off while
 *            armed, flashing green while ultrasonic is being detected.
 *            Press again to cancel back to IDLE.
 *   RECORD : detection persists (see filters below) — clip streams into a
 *            PSRAM slot, then back to IDLE. GAIN IS FROZEN for the whole clip.
 *
 * Clips live in PSRAM only (lost at power-off): 3.84 s @ 256 ksps 16-bit =
 * ~1.97 MB each; an 8 MB (N16R8) module fits 4 slots, oldest is overwritten.
 * WAVs carry a GUANO metadata chunk and Wildlife-Acoustics-style filenames
 * (CHIRP_YYYYMMDD_HHMMSS.wav) so the Echo Meter iPhone app accepts them.
 * No RTC on board: the clock is seeded from firmware build time at boot.
 *
 * Wiring
 *   ADC  : ADCS7476 — CS<-GPIO4 (I2S WS), SCLK<-GPIO5 (I2S BCK), SDATA->GPIO6
 *   GAIN : 74HC4053 — S1<-GPIO38, S2<-GPIO39, S3<-GPIO40
 *   RGB  : GPIO48 (WS2812)   BTN : GPIO0 (BOOT, active-low)
 *
 *   NOTE: GPIO35/36/37 are NOT usable on an N16R8 module — they belong to the
 *   octal PSRAM bus. The gain lines must stay clear of them.
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
#include "nvs.h"
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

/* 74HC4053 select lines. Keep off GPIO35/36/37 (octal PSRAM on N16R8). */
#define PIN_GAIN_S1     GPIO_NUM_38
#define PIN_GAIN_S2     GPIO_NUM_39
#define PIN_GAIN_S3     GPIO_NUM_40

/* ── Wi-Fi AP ────────────────────────────────────────────────────────────── */
#define AP_SSID         "echometer"        /* open network, no password */
#define AP_CHANNEL      1
#define AP_MAX_CONN     2

/* ── Sampling / FFT ──────────────────────────────────────────────────────── */
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
 *  - persistence: DETECT_CONSEC consecutive FFT frames (~8 ms each) must
 *    pass before recording starts. Lower to 1-2 if hunting very short clicks. */
#define CHIRP_TONALITY_MIN  20.0f
#define DETECT_CONSEC       3U

/* ── Variable gain (74HC4053) ────────────────────────────────────────────────
 * Stage 1 (OPA838): Rg = R6 = 1k, Rf = R13 = 33k, and R26 = 3k6 switched in
 *                   parallel by mux switch 1  ->  gain 34 or 4.25
 * Stage 2 (TL972) : Rg = R12 = 1k, Rf = R25 = 22k permanent, with R23 = 22k
 *                   (switch 2) and R24 = 6k8 (switch 3) switchable in parallel
 *                   ->  gain 23 / 12 / 6.19 / 5.20
 *
 * 4053 truth table: Sn LOW selects Yn0 = the resistor is CONNECTED = parallel
 * combination = LOWER gain. Sn HIGH selects Yn1, which is a no-connect.
 * R13 and R25 are permanent, so the loop is never open — and all-LOW (the
 * state the GPIOs sit in at reset) is minimum gain, which cannot clip.
 *
 * clip_spl is the input level at which the TL972 output hits its rail, for the
 * SPU0410 at -38 dBV/Pa and ~1.45 V peak of available swing. Use it to pick a
 * step for the environment; the AGC below does it from the samples instead. */
typedef struct {
    uint8_t  s1, s2, s3;    /* 4053 select levels                            */
    uint16_t gain;          /* total voltage gain, stage 1 x stage 2         */
    uint8_t  clip_spl;      /* approx. input that clips, dB SPL re 20 uPa    */
} gain_step_t;

static const gain_step_t GAIN_STEPS[] = {
    /* S1 S2 S3   gain  clip                                                 */
    {  0, 0, 0,     22, 105 },   /* cave / roost, bats at arm's length       */
    {  0, 0, 1,     51,  98 },
    {  0, 1, 1,     98,  92 },   /* general close work                       */
    {  1, 1, 0,    210,  86 },
    {  1, 0, 1,    408,  80 },
    {  1, 1, 1,    782,  74 },   /* max — distant passes, ~30 m              */
};
#define GAIN_NSTEPS   (sizeof(GAIN_STEPS) / sizeof(GAIN_STEPS[0]))
#define GAIN_DEFAULT  3U        /* 210x — a sane cold-start compromise       */

/* AGC. Asymmetric on purpose: drop the gain the instant a frame gets close to
 * full scale (a clipped call is unrecoverable — the clipping happens in the
 * analog domain, before the ADC), but only raise it after a sustained quiet
 * spell, otherwise the gain pumps on every gap between calls. */
#define AGC_FULLSCALE      2048.0f    /* 12-bit, centred                     */
#define AGC_ATTACK_LEVEL   (0.90f * AGC_FULLSCALE)
#define AGC_DECAY_LEVEL    (0.10f * AGC_FULLSCALE)
#define AGC_DECAY_FRAMES   120U       /* ~1 s of quiet before stepping up    */
#define GAIN_SETTLE_FRAMES 2U         /* frames to ignore after a gain change */

static volatile uint8_t  g_gain_idx = GAIN_DEFAULT;
static volatile bool     g_agc_on   = true;
static volatile uint16_t g_peak     = 0;   /* peak of the last frame, 0..2048 */

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
    uint16_t      gain;                    /* gain in force for the whole clip */
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
 * to recognise bat recordings. Returned length is always even (RIFF rule).
 *
 * The analog gain in force is recorded twice: in the free-text Note, and as a
 * namespaced "Echometer|Gain" field (GUANO readers must ignore namespaces they
 * don't know). Without it the absolute SPL of a clip can't be reconstructed
 * offline, because the same waveform amplitude means 31 dB more sound at 22x
 * than at 782x. If the Echo Meter import ever starts failing, drop the
 * namespaced line first — the Note is the safer of the two. */
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
        "Note: recorded by DIY echometer-v4, analog gain %ux\n"
        "Samplerate: %u\n"
        "Length: %.2f\n"
        "Timestamp: %04d-%02d-%02dT%02d:%02d:%02d\n"
        "Echometer|Gain: %u",
        (unsigned)s->gain,
        (unsigned)SAMPLE_RATE_HZ, (float)CLIP_SAMPLES / SAMPLE_RATE_HZ,
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec,
        (unsigned)s->gain);
    if (len & 1) out[len++] = '\n';    /* pad to even length */
    out[len] = '\0';
    return len;
}

/* ── Variable gain ───────────────────────────────────────────────────────── */
static void gain_apply(uint8_t idx)
{
    if (idx >= GAIN_NSTEPS) idx = GAIN_NSTEPS - 1U;
    const gain_step_t *s = &GAIN_STEPS[idx];
    gpio_set_level(PIN_GAIN_S1, s->s1);
    gpio_set_level(PIN_GAIN_S2, s->s2);
    gpio_set_level(PIN_GAIN_S3, s->s3);
    g_gain_idx = idx;
}

/* Persist only deliberate (web) changes, not every AGC step — the AGC can move
 * many times a minute and NVS is flash. */
static void gain_save(void)
{
    nvs_handle_t h;
    if (nvs_open("echometer", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "gain", g_gain_idx);
    nvs_set_u8(h, "agc",  g_agc_on ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

static void gain_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_GAIN_S1) | (1ULL << PIN_GAIN_S2) |
                        (1ULL << PIN_GAIN_S3),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* min gain if ever floated  */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    uint8_t idx = GAIN_DEFAULT, agc = 1;
    nvs_handle_t h;
    if (nvs_open("echometer", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v;
        if (nvs_get_u8(h, "gain", &v) == ESP_OK && v < GAIN_NSTEPS) idx = v;
        if (nvs_get_u8(h, "agc",  &v) == ESP_OK)                    agc = v;
        nvs_close(h);
    }
    g_agc_on = agc ? true : false;
    gain_apply(idx);
    ESP_LOGI(TAG, "gain %ux (step %u/%u), AGC %s",
             (unsigned)GAIN_STEPS[idx].gain, (unsigned)idx,
             (unsigned)GAIN_NSTEPS - 1U, g_agc_on ? "on" : "off");
}

/* Returns true if the gain moved, so the caller can skip a frame or two while
 * the mux charge injection settles out of the summing node. */
static bool agc_update(uint16_t peak)
{
    static uint32_t hold = AGC_DECAY_FRAMES;

    if (!g_agc_on) return false;

    if (peak > AGC_ATTACK_LEVEL) {          /* clipping imminent — act NOW    */
        hold = AGC_DECAY_FRAMES;
        if (g_gain_idx > 0) {
            gain_apply(g_gain_idx - 1U);
            ESP_LOGI(TAG, "AGC down -> %ux (peak %u)",
                     (unsigned)GAIN_STEPS[g_gain_idx].gain, (unsigned)peak);
            return true;
        }
    } else if (peak < AGC_DECAY_LEVEL) {    /* lots of headroom — creep up    */
        if (hold > 0) { hold--; }
        else {
            hold = AGC_DECAY_FRAMES;
            if (g_gain_idx + 1U < GAIN_NSTEPS) {
                gain_apply(g_gain_idx + 1U);
                ESP_LOGI(TAG, "AGC up -> %ux (peak %u)",
                         (unsigned)GAIN_STEPS[g_gain_idx].gain, (unsigned)peak);
                return true;
            }
        }
    } else {
        hold = AGC_DECAY_FRAMES;            /* in the sweet spot — sit still  */
    }
    return false;
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

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
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
    char line[320];
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><meta name=viewport "
        "content=\"width=device-width,initial-scale=1\">"
        "<title>echometer</title><style>"
        "body{font-family:system-ui,sans-serif;margin:1rem;line-height:1.5}"
        "a.btn{display:inline-block;padding:.4rem .7rem;margin:.15rem;"
        "border:1px solid #888;border-radius:.4rem;text-decoration:none}"
        "a.on{background:#2a6;color:#fff;border-color:#2a6}"
        "</style></head><body>"
        "<h2>echometer v4</h2>");

    snprintf(line, sizeof(line), "<p>state: %s &mdash; <a href=\"/\">refresh</a></p>",
             g_state == ST_IDLE ? "idle (press button to arm)" :
             g_state == ST_DETECT ? "listening for ultrasonic..." : "recording");
    httpd_resp_sendstr_chunk(req, line);

    /* ── gain panel ── */
    const gain_step_t *cur = &GAIN_STEPS[g_gain_idx];
    snprintf(line, sizeof(line),
        "<h3>Gain</h3><p>now <b>%ux</b> (clips near <b>%u dB SPL</b>) &mdash; "
        "last frame peak %u/2048</p><p>",
        (unsigned)cur->gain, (unsigned)cur->clip_spl, (unsigned)g_peak);
    httpd_resp_sendstr_chunk(req, line);

    for (uint32_t i = 0; i < GAIN_NSTEPS; i++) {
        snprintf(line, sizeof(line),
            "<a class=\"btn%s\" href=\"/gain?i=%u\">%ux</a>",
            (i == g_gain_idx && !g_agc_on) ? " on" : "",
            (unsigned)i, (unsigned)GAIN_STEPS[i].gain);
        httpd_resp_sendstr_chunk(req, line);
    }

    snprintf(line, sizeof(line),
        "</p><p><a class=\"btn%s\" href=\"/agc?on=%d\">AGC %s</a> "
        "&mdash; auto-tracks level; picking a gain above turns it off</p>",
        g_agc_on ? " on" : "", g_agc_on ? 0 : 1, g_agc_on ? "on" : "off");
    httpd_resp_sendstr_chunk(req, line);

    /* ── clip list, newest first (n_slots <= 8, O(n^2) is fine) ── */
    httpd_resp_sendstr_chunk(req, "<h3>Captured chirps</h3><ul>");
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
            "<li>%s &mdash; %.1f s @ %u kHz, %u KB, gain %ux &mdash; "
            "<a href=\"/dl?n=%u\">download</a> | "
            "<a href=\"/del?n=%u\" "
            "onclick=\"return confirm('Delete %s?')\">delete</a></li>",
            fname, (float)CLIP_SAMPLES / SAMPLE_RATE_HZ,
            (unsigned)(SAMPLE_RATE_HZ / 1000), (unsigned)(CLIP_BYTES / 1024),
            (unsigned)best->gain,
            (unsigned)best->seq, (unsigned)best->seq, fname);
        httpd_resp_sendstr_chunk(req, line);
    }
    if (last_seq == UINT32_MAX)
        httpd_resp_sendstr_chunk(req, "<li>(no chirps captured yet)</li>");

    httpd_resp_sendstr_chunk(req, "</ul></body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

/* Read one unsigned integer query parameter; returns def if absent/garbage. */
static uint32_t query_u32(httpd_req_t *req, const char *key, uint32_t def)
{
    char query[64], val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, key, val, sizeof(val)) == ESP_OK)
        return (uint32_t)strtoul(val, NULL, 10);
    return def;
}

static esp_err_t redirect_home(httpd_req_t *req)
{
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Picking a gain by hand implies you want that gain to stay put, so this also
 * turns the AGC off — otherwise the next quiet second would override you. */
static esp_err_t http_gain(httpd_req_t *req)
{
    uint32_t i = query_u32(req, "i", g_gain_idx);
    if (i < GAIN_NSTEPS) {
        g_agc_on = false;
        gain_apply((uint8_t)i);
        gain_save();
        ESP_LOGI(TAG, "gain set to %ux via web (AGC off)",
                 (unsigned)GAIN_STEPS[i].gain);
    }
    return redirect_home(req);
}

static esp_err_t http_agc(httpd_req_t *req)
{
    g_agc_on = query_u32(req, "on", 1) ? true : false;
    gain_save();
    ESP_LOGI(TAG, "AGC %s via web", g_agc_on ? "on" : "off");
    return redirect_home(req);
}

static clip_slot_t *slot_from_query(httpd_req_t *req)
{
    uint32_t seq = query_u32(req, "n", 0);
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

    char guano[256];
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
    return redirect_home(req);             /* back to the list either way */
}

static void http_init(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size       = 8192;
    cfg.lru_purge_enable = true;
    cfg.max_uri_handlers = 8;
    /* A 2 MB clip is a long transfer; if the link stalls briefly (a weak
     * moment, phone power-save), the default 5 s socket timeout aborts the
     * download mid-file. Give each send/recv much longer to ride out stalls. */
    cfg.send_wait_timeout = 30;
    cfg.recv_wait_timeout = 30;
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    static const httpd_uri_t root = { .uri = "/",     .method = HTTP_GET, .handler = http_root };
    static const httpd_uri_t dl   = { .uri = "/dl",   .method = HTTP_GET, .handler = http_download };
    static const httpd_uri_t del  = { .uri = "/del",  .method = HTTP_GET, .handler = http_delete };
    static const httpd_uri_t gain = { .uri = "/gain", .method = HTTP_GET, .handler = http_gain };
    static const httpd_uri_t agc  = { .uri = "/agc",  .method = HTTP_GET, .handler = http_agc };
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &dl));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &del));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &gain));
    ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &agc));
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
    uint16_t peak = 0;
    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        uint16_t w = (uint16_t)i2s_raw[2 * i];
        uint16_t s = (w >> ADC_SAMPLE_SHIFT) & 0x0FFF;
        sample_buf[i] = s;
        /* Excursion from midscale, for the AGC. Cheap enough to do inline. */
        uint16_t d = (s > 2048U) ? (uint16_t)(s - 2048U) : (uint16_t)(2048U - s);
        if (d > peak) peak = d;
    }
    g_peak = peak;
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
    ESP_LOGI(TAG, "=== chirp catcher v4 ===  Fs=%u  bin_min=%u  clip=%u bufs",
             (unsigned)SAMPLE_RATE_HZ, (unsigned)ULTRASONIC_BIN_MIN,
             (unsigned)CLIP_BUFS);

    clip_slot_t *rec_slot   = NULL;
    uint32_t     rec_buf    = 0;
    uint32_t     det_streak = 0;
    uint32_t     settle     = 0;   /* frames to ignore after a gain change     */
    int64_t      blue_until = 0;   /* post-record blue flash deadline (0 = off) */

    for (;;) {
        adc_read_frame();   /* always drain I2S DMA, ~8 ms per loop */

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
                ESP_LOGI(TAG, "ARMED - listening for ultrasonic (gain %ux)",
                         (unsigned)GAIN_STEPS[g_gain_idx].gain);
            } else if (g_state == ST_DETECT) {
                g_state    = ST_IDLE;
                g_led_mode = LED_OFF;
                ESP_LOGI(TAG, "disarmed");
            }
            /* button ignored during the 4 s RECORD */
        }

        /* AGC runs while idle and while armed, so the gain is already tracking
         * the environment when a call arrives. It is frozen during RECORD: a
         * gain step mid-clip puts a step change in the waveform and makes the
         * clip useless for absolute amplitude. */
        if (g_state != ST_RECORD && agc_update(g_peak)) {
            settle     = GAIN_SETTLE_FRAMES;
            det_streak = 0;                /* the switch transient is not a bat */
        }
        if (settle) { settle--; continue; }

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
                rec_slot->gain  = GAIN_STEPS[g_gain_idx].gain;  /* freeze it   */
                frame_to_pcm(rec_slot->data); /* trigger buffer = clip start  */
                rec_buf    = 1;
                g_state    = ST_RECORD;
                g_led_mode = LED_OFF;          /* LED silent while sampling the clip */
                ESP_LOGI(TAG, "DETECT -> recording CHIRP%04u at %ux",
                         (unsigned)rec_slot->seq, (unsigned)rec_slot->gain);
            }
        } else if (g_state == ST_RECORD) {
            frame_to_pcm(rec_slot->data + (size_t)rec_buf * FFT_SIZE);
            if (++rec_buf >= CLIP_BUFS) {
                rec_slot->ready = true;
                ESP_LOGI(TAG, "CHIRP%04u done (%.2f s, gain %ux) -> idle, "
                              "press button to re-arm",
                         (unsigned)rec_slot->seq,
                         (float)CLIP_SAMPLES / SAMPLE_RATE_HZ,
                         (unsigned)rec_slot->gain);
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
    nvs_init();              /* before gain_init — the gain choice lives here */
    gain_init();             /* before the analog front end sees any signal   */
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    led_init();
    btn_init();
    slots_init();
    adc_i2s_init();

    xTaskCreatePinnedToCore(led_task,  "led",  2048, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(main_task, "main", 8192, NULL, 5, NULL, tskNO_AFFINITY);

    /* Bring the radio up last, well after the boot/PSRAM inrush has settled —
     * Wi-Fi RF calibration is the biggest current spike and marginal supplies
     * brown out when the spikes stack. */
    vTaskDelay(pdMS_TO_TICKS(2000));
    wifi_ap_init();
    http_init();
}
