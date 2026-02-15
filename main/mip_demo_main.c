#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#define TAG "mip_demo"

#define MIP_HOST SPI2_HOST

#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_SCS  5
#define PIN_NUM_DISP 21
#define PIN_NUM_FRONTLIGHT 22

#define MIP_WIDTH  72
#define MIP_HEIGHT 144
#define MIP_LINE_BYTES (MIP_WIDTH / 2)

#define MIP_SPI_HZ (4 * 1000 * 1000)

#define MIP_CMD_NO_UPDATE          0x00
#define MIP_CMD_ALL_CLEAR          0x20
#define MIP_CMD_VCOM               0x40
#define MIP_CMD_UPDATE             0x90

#define COLOR_BLACK   0x00
#define COLOR_BLUE    0x02
#define COLOR_GREEN   0x04
#define COLOR_CYAN    0x06
#define COLOR_RED     0x08
#define COLOR_MAGENTA 0x0A
#define COLOR_YELLOW  0x0C
#define COLOR_WHITE   0x0E

static spi_device_handle_t s_spi;
static SemaphoreHandle_t s_spi_mutex;
static uint8_t s_vcom_bit;
static uint8_t s_fb[MIP_HEIGHT][MIP_LINE_BYTES];

static esp_err_t mip_spi_send(const uint8_t *data, size_t len)
{
    spi_transaction_t t = {
        .length = (uint32_t)(len * 8),
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t mip_send_two_byte_cmd(uint8_t cmd)
{
    uint8_t pkt[2] = { cmd, 0x00 };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = mip_spi_send(pkt, sizeof(pkt));
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static void fb_set_pixel(int x, int y, uint8_t color)
{
    if (x < 0 || x >= MIP_WIDTH || y < 0 || y >= MIP_HEIGHT) {
        return;
    }

    uint8_t *packed = &s_fb[y][x / 2];
    if ((x & 1) == 0) {
        *packed = (uint8_t)((*packed & 0x0F) | ((color & 0x0F) << 4));
    } else {
        *packed = (uint8_t)((*packed & 0xF0) | (color & 0x0F));
    }
}

static void fb_fill(uint8_t color)
{
    uint8_t packed = (uint8_t)((color & 0x0F) | ((color & 0x0F) << 4));
    for (int y = 0; y < MIP_HEIGHT; y++) {
        memset(s_fb[y], packed, MIP_LINE_BYTES);
    }
}

static void fb_fill_rect(int x, int y, int w, int h, uint8_t color)
{
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = x + w;
    int y1 = y + h;

    if (x1 > MIP_WIDTH) {
        x1 = MIP_WIDTH;
    }
    if (y1 > MIP_HEIGHT) {
        y1 = MIP_HEIGHT;
    }

    for (int yy = y0; yy < y1; yy++) {
        for (int xx = x0; xx < x1; xx++) {
            fb_set_pixel(xx, yy, color);
        }
    }
}

static esp_err_t mip_refresh(void)
{
    uint8_t line_pkt[MIP_LINE_BYTES + 4];

    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    for (int y = 0; y < MIP_HEIGHT; y++) {
        line_pkt[0] = (uint8_t)(MIP_CMD_UPDATE | s_vcom_bit);
        line_pkt[1] = (uint8_t)(y + 1);
        memcpy(&line_pkt[2], s_fb[y], MIP_LINE_BYTES);
        line_pkt[MIP_LINE_BYTES + 2] = 0x00;
        line_pkt[MIP_LINE_BYTES + 3] = 0x00;

        esp_err_t ret = mip_spi_send(line_pkt, sizeof(line_pkt));
        if (ret != ESP_OK) {
            xSemaphoreGive(s_spi_mutex);
            return ret;
        }
    }
    xSemaphoreGive(s_spi_mutex);

    return ESP_OK;
}

static void draw_demo_frame(int frame)
{
    static const uint8_t bars[8] = {
        COLOR_BLACK, COLOR_BLUE, COLOR_GREEN, COLOR_CYAN,
        COLOR_RED, COLOR_MAGENTA, COLOR_YELLOW, COLOR_WHITE
    };

    int bar_h = MIP_HEIGHT / 8;
    for (int i = 0; i < 8; i++) {
        fb_fill_rect(0, i * bar_h, MIP_WIDTH, bar_h, bars[i]);
    }

    int square = 18;
    int period = (MIP_WIDTH - square) * 2;
    int t = frame % period;
    int x = (t < (MIP_WIDTH - square)) ? t : (period - t);
    int y = (frame / 2) % (MIP_HEIGHT - square);

    fb_fill_rect(x, y, square, square, COLOR_WHITE);
    fb_fill_rect(x + 3, y + 3, square - 6, square - 6, COLOR_BLACK);

    for (int i = 0; i < MIP_HEIGHT; i += 8) {
        int px = (i + frame) % MIP_WIDTH;
        fb_set_pixel(px, i, COLOR_RED);
    }
}

static void vcom_task(void *arg)
{
    (void)arg;

    while (1) {
        s_vcom_bit = (s_vcom_bit == 0) ? MIP_CMD_VCOM : 0;
        ESP_ERROR_CHECK(mip_send_two_byte_cmd((uint8_t)(MIP_CMD_NO_UPDATE | s_vcom_bit)));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    esp_err_t ret;

    s_spi_mutex = xSemaphoreCreateMutex();
    if (s_spi_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create SPI mutex");
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_DISP) | (1ULL << PIN_NUM_FRONTLIGHT),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(PIN_NUM_DISP, 1);
    gpio_set_level(PIN_NUM_FRONTLIGHT, 1);

    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MIP_LINE_BYTES + 4,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = MIP_SPI_HZ,
        .mode = 0,
        .spics_io_num = PIN_NUM_SCS,
        .queue_size = 1,
        .flags = SPI_DEVICE_POSITIVE_CS,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(MIP_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(MIP_HOST, &devcfg, &s_spi));

    s_vcom_bit = 0;
    fb_fill(COLOR_BLACK);

    ESP_ERROR_CHECK(mip_send_two_byte_cmd((uint8_t)(MIP_CMD_ALL_CLEAR | s_vcom_bit)));

    xTaskCreate(vcom_task, "vcom_task", 2048, NULL, 5, NULL);

    int frame = 0;
    while (1) {
        draw_demo_frame(frame++);
        ret = mip_refresh();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "mip_refresh failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
