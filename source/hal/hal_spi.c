#include "hal.h"

static hal_spi_driver_t *drv = &HAL_SPI_DRIVER;

hal_spi_dev_t hal_spi_init(hal_spi_device_t device, hal_spi_config_t* cfg)
{
    return drv->init(device, cfg);
}

void hal_spi_deinit(hal_spi_dev_t *pdev)
{
    drv->deinit(pdev);
}

hal_spi_status_t hal_spi_transmit_receive(hal_spi_dev_t dev, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t size, uint32_t timeout_ms)
{
    return drv->transmit_receive(dev, tx_buffer, rx_buffer, size, timeout_ms);
}

hal_spi_status_t hal_spi_transmit(hal_spi_dev_t dev, const uint8_t* buffer, size_t size, uint32_t timeout_ms)
{
    return hal_spi_transmit_receive(dev, buffer, NULL, size, timeout_ms);
}

hal_spi_status_t hal_spi_receive(hal_spi_dev_t dev, uint8_t* buffer, size_t size, uint32_t timeout_ms)
{
    return hal_spi_transmit_receive(dev, NULL, buffer, size, timeout_ms);
}