#include "mcp3008.h"

void spi_init (gpio_num_t gpio_miso, gpio_num_t gpio_mosi, gpio_num_t gpio_sclk, gpio_num_t gpio_cs) {
    esp_err_t esp_err;

    spi_bus_config_t spi_bus_config = {
		.sclk_io_num = gpio_sclk,
		.mosi_io_num = gpio_mosi,
		.miso_io_num = gpio_miso,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

    esp_err = spi_bus_initialize( HSPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO );
	ESP_LOGI(TAG, "spi bus initialize=%d", esp_err);
}

void mcp3008_init(MCP3008 *device, gpio_num_t gpio_miso, gpio_num_t gpio_mosi, gpio_num_t gpio_sclk, gpio_num_t gpio_cs) {
    esp_err_t esp_err;

    gpio_reset_pin( gpio_cs );
	gpio_set_direction( gpio_cs, GPIO_MODE_OUTPUT );
	gpio_set_level( gpio_cs, 1 );

    spi_init(gpio_miso, gpio_mosi, gpio_sclk, gpio_cs);

    spi_device_interface_config_t spi_device_interface = {
		.clock_speed_hz = SPI_MASTER_FREQ_1M,
		.spics_io_num = gpio_cs,
		.queue_size = 7,
		.mode = 0,
		.flags = SPI_DEVICE_NO_DUMMY,
	};

    spi_device_handle_t handle;
	esp_err = spi_bus_add_device( HSPI_HOST, &spi_device_interface, &handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d", esp_err);
    
    device->data = 0;
    device->handle = handle;
    device->bits = 10;
}

void mcp3008_read(MCP3008 *device, unsigned char channel) {
    unsigned char read_buffer[3];
    unsigned char write_buffer[3];
    esp_err_t esp_err;

    memset(write_buffer, 0, sizeof(read_buffer));
    memset(read_buffer, 0, sizeof(read_buffer));

	// MCP3008
	// [IN]  00 START SGL/DIFF D2 D1 D0 DMY
	// [OUT] -- ----- -------- -- -- -- --- 00 B9 B8 B7 B6 B5 B4 B3 B2 B1 B0

    write_buffer[0] = 0x60 | channel << 2;
    spi_transaction_t SPI_transaction;

    memset( &SPI_transaction, 0, sizeof( spi_transaction_t ) );
	SPI_transaction.length = 3 * 8;
	SPI_transaction.tx_buffer = write_buffer;
	SPI_transaction.rx_buffer = read_buffer;

    esp_err = spi_device_transmit( device->handle, &SPI_transaction );
    if (esp_err == ESP_OK){
        device->data = (read_buffer[1]<<2) + (read_buffer[2]>>6);
        return;
    }

    device->data = NULL;
}
