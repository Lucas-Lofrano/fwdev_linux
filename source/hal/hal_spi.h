#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/*
    PARA SIMULAR AMBOS AS PARTES DA COMUNICAÇÃO 
    EU ELENQUEI DUAS FUNÇÕES LIGADAS AO MESMO DEVICE
    AI EU CONSIGO DENTRO DA CAMADA DE PORT 'LINKAR'
    O MESTRE E O ESCRAVO, NA MESMA MEMÓRIA COMPARTILHADA
    E USAR UM MUTEX PARA SIMULAR O CLOCK

    //PS: PERGUNTAR PARA O MBARROS DE PODE
*/
typedef enum hal_spi_role_e
{
    HAL_SPI_ROLE_MASTER = 0, // SPI master role
    HAL_SPI_ROLE_SLAVE,      // SPI slave role
} hal_spi_role_t;

typedef enum hal_spi_mode_e
{
    HAL_SPI_MODE_MASTER_FULL_DUPLEX = 0,
    HAL_SPI_MODE_MASTER_HALF_DUPLEX,
    HAL_SPI_MODE_SLAVE_FULL_DUPLEX,
    HAL_SPI_MODE_SLAVE_HALF_DUPLEX,
    HAL_SPI_MODE_NUM_MODES,
} hal_spi_mode_t;

typedef enum hal_spi_frame_format_e
{
    HAL_SPI_FRAME_FORMAT_MOTOROLA = 0,  // Motorola SPI format
    HAL_SPI_FRAME_FORMAT_TI,            // Texas Instruments SPI format
    HAL_SPI_FRAME_FORMAT_NXP,           // NXP SPI format
} hal_spi_frame_format_t;

typedef enum hal_spi_byte_order_e
{
    HAL_SPI_BYTE_ORDER_MSB_FIRST = 0, // Most Significant Bit first
    HAL_SPI_BYTE_ORDER_LSB_FIRST,     // Least Significant Bit first
} hal_spi_byte_order_t;

typedef enum hal_spi_data_size_e
{
    HAL_SPI_DATA_SIZE_4BITS = 0,    // 4 bits per frame
    HAL_SPI_DATA_SIZE_8BITS,        // 8 bits per frame
    HAL_SPI_DATA_SIZE_16BITS,       // 16 bits per frame
    HAL_SPI_DATA_SIZE_32BITS,       // 32 bits per frame
} hal_spi_data_size_t;

typedef enum hal_spi_polarity_e
{
    HAL_SPI_POLARITY_LOW = 0,  // CPOL=0
    HAL_SPI_POLARITY_HIGH,     // CPOL=1
} hal_spi_polarity_t;

typedef enum hal_spi_phase_e
{
    HAL_SPI_PHASE_0 = 0, // CPHA=0
    HAL_SPI_PHASE_1,     // CPHA=1
} hal_spi_phase_t;

typedef enum hal_spi_status_e
{
    HAL_SPI_STATUS_OK = 0,          // Operation successful
    HAL_SPI_STATUS_ERROR,           // General error
    HAL_SPI_STATUS_BUSY,            // Device is busy
    HAL_SPI_STATUS_TIMEOUT,         // Operation timed out
    HAL_SPI_STATUS_INVALID_PARAM,   // Invalid parameter provided
} hal_spi_status_t;

typedef enum hal_spi_device_e
{
    HAL_SPI_DEVICE_0 = 0, // SPI device 0
    HAL_SPI_DEVICE_1,     // SPI device 1
    HAL_SPI_DEVICE_NUM_DEVICES,
} hal_spi_device_t;

//PONTEIRO OPACO
typedef struct hal_spi_dev_s* hal_spi_dev_t;

//ESTRUTURA DE CONFIGURAÇÃO DO SPI
typedef struct hal_spi_config_s
{
    hal_spi_role_t role;                 // SPI role (master or slave)
    hal_spi_mode_t mode;                 // SPI mode (master/slave, full/half duplex)
    hal_spi_frame_format_t frame_format; // Frame format (Motorola, TI, NXP)
    hal_spi_byte_order_t byte_order;     // Byte order (MSB/LSB first)
    hal_spi_data_size_t data_size;       // Data size (4, 8, 16, or 32 bits)
    uint32_t baudrate;                   // Baud rate in Hz
    hal_spi_polarity_t polarity;         // Clock polarity (CPOL)
    hal_spi_phase_t phase;               // Clock phase (CPHA)
} hal_spi_config_t;

typedef struct hal_spi_driver_s
{
    hal_spi_dev_t (*init) (hal_spi_device_t device, hal_spi_config_t* cfg);
    void (*deinit) (hal_spi_dev_t *pdev);
    hal_spi_status_t (*transmit_receive) (hal_spi_dev_t dev, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t size, uint32_t timeout_ms);
} hal_spi_driver_t;

hal_spi_dev_t hal_spi_init(hal_spi_device_t device, hal_spi_config_t* cfg);
void hal_spi_deinit(hal_spi_dev_t *pdev);
hal_spi_status_t hal_spi_transmit_receive(hal_spi_dev_t dev, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t size, uint32_t timeout_ms);
hal_spi_status_t hal_spi_transmit(hal_spi_dev_t dev, const uint8_t* buffer, size_t size, uint32_t timeout_ms);
hal_spi_status_t hal_spi_receive(hal_spi_dev_t dev, uint8_t* buffer, size_t size, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif