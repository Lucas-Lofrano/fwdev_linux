#include "app.h"
#include "hal.h"
#include "utl_dbg.h"

#define MAX_BUS_SIZE 256;

//MISO OR MOSI
typedef struct 
{
    const uint8_t *buf;
    size_t buf_size;
    size_t buf_index;
} port_spi_bus_t;

struct hal_spi_dev_s 
{
    hal_spi_device_t device;
    hal_spi_config_t cfg;

    port_spi_bus_t *bus;
};

//MOSI AND MISO
static port_spi_bus_t PORT_BUSES[2] = { {0}, {0} };

static struct hal_spi_dev_s port_spi_devs[] =
{
    {.device = HAL_SPI_DEVICE_0, .bus = &PORT_BUSES[0], .cfg = 0},
    {.device = HAL_SPI_DEVICE_0, .bus = &PORT_BUSES[1], .cfg = 0},
};

void port_spi_slave_load_data(const uint8_t* data, size_t size)
{
    // Correctly get a pointer to the slave device structure from the devs array.
    struct hal_spi_dev_s *slave_dev = &port_spi_devs[1]; 
    
    // Now access its 'bus' member to load the data.
    slave_dev->bus->buf = data;
    slave_dev->bus->buf_size = size;
    slave_dev->bus->buf_index = 0;
}

static hal_spi_dev_t port_spi_init(hal_spi_device_t device, hal_spi_config_t* cfg)
{
    if (device >= HAL_SPI_DEVICE_NUM_DEVICES)
        return NULL;

    struct hal_spi_dev_s *dev = NULL;
    if (cfg->role == HAL_SPI_ROLE_MASTER)
        dev = &port_spi_devs[0];
    else
        dev = &port_spi_devs[1];

    dev->device = device;
    memcpy(&dev->cfg, cfg, sizeof(hal_spi_config_t));
    
    dev->bus->buf_index = 0;
    dev->bus->buf_size = 0;
    dev->bus->buf = NULL;

    return dev;
}

static void port_spi_deinit(hal_spi_dev_t *pdev)
{
    if (pdev && *pdev)
        *pdev = NULL;
}

static hal_spi_status_t port_spi_transmit_receive(hal_spi_dev_t dev, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t size, uint32_t timeout_ms)
{
    hal_spi_dev_t master_dev = dev;
    hal_spi_dev_t slave_dev  = &port_spi_devs[1];

    if (master_dev->cfg.role != HAL_SPI_ROLE_MASTER || slave_dev->cfg.role != HAL_SPI_ROLE_SLAVE)
        return HAL_SPI_STATUS_INVALID_PARAM;

    if (master_dev->cfg.polarity != slave_dev->cfg.polarity || master_dev->cfg.phase != slave_dev->cfg.phase) 
        return HAL_SPI_STATUS_ERROR;

    for (size_t i = 0; i < size; ++i) 
    {
        uint8_t mosi_byte = tx_buffer ? tx_buffer[i] : 0xFF;

        // Get the byte the slave wants to send back
        // If slave has no data, it sends a dummy byte (often 0x00 or 0xFF)
        uint8_t miso_byte = 0xFF; 
        if (slave_dev->bus->buf && slave_dev->bus->buf_index < slave_dev->bus->buf_size) 
        {
            miso_byte = slave_dev->bus->buf[slave_dev->bus->buf_index];
            slave_dev->bus->buf_index++;
        }

        if (rx_buffer) 
        {
            rx_buffer[i] = miso_byte;
        }
    }
    
    return HAL_SPI_STATUS_OK;
}

hal_spi_driver_t HAL_SPI_DRIVER =
{
    .init = port_spi_init,// Implement the init function
    .deinit = port_spi_deinit, // Implement the deinit function
    .transmit_receive = port_spi_transmit_receive // Implement the transmit_receive function
};