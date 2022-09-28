#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#define RESOLUTION 10
#define TAG "MCP3008"

#define SPI_MASTER_FREQ_1M      (APB_CLK_FREQ/80)
#define SPI_MASTER_FREQ_2M      (APB_CLK_FREQ/40)
#define SPI_MASTER_FREQ_4M      (APB_CLK_FREQ/20)


typedef struct {
    unsigned char bits;
    unsigned char channels;
    unsigned short data;
    spi_device_handle_t handle;
} MCP3008;

void mcp3008_init(MCP3008 *device, gpio_num_t gpio_miso, gpio_num_t gpio_mosi, gpio_num_t gpio_sclk, gpio_num_t gpio_cs);
void mcp3008_read(MCP3008 *device, unsigned char channel);
