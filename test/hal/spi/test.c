#include "hal.h"
#include "app.h"

#include "hal_spi.h"

void port_spi_slave_load_data(const uint8_t* data, size_t size);

// We'll use this to run the test only once.
static bool test_has_run = false;

void app_init(void)
{
    printf("Initializing application and SPI devices...\n");

    // 1. Initialize the HAL
    hal_init();

    // 2. Configure the Master device
    hal_spi_config_t master_cfg = {
        .role = HAL_SPI_ROLE_MASTER,
        .data_size = HAL_SPI_DATA_SIZE_8BITS,
        .byte_order = HAL_SPI_BYTE_ORDER_MSB_FIRST,
        .polarity = HAL_SPI_POLARITY_LOW,
        .phase = HAL_SPI_PHASE_0,
        // Other parameters can be set as needed
    };

    // 3. Configure the Slave device
    hal_spi_config_t slave_cfg = {
        .role = HAL_SPI_ROLE_SLAVE,
        .data_size = HAL_SPI_DATA_SIZE_8BITS,
        .byte_order = HAL_SPI_BYTE_ORDER_MSB_FIRST,
        .polarity = HAL_SPI_POLARITY_LOW, // Must match master
        .phase = HAL_SPI_PHASE_0,         // Must match master
    };

    // 4. Initialize both devices. In our simulation, they are linked internally.
    hal_spi_dev_t master_spi = hal_spi_init(HAL_SPI_DEVICE_0, &master_cfg);
    hal_spi_dev_t slave_spi = hal_spi_init(HAL_SPI_DEVICE_0, &slave_cfg);

    // Ensure initialization was successful
    assert(master_spi != NULL);
    assert(slave_spi != NULL);

    printf("SPI Master and Slave initialized successfully.\n");
}

bool app_loop(void)
{
    if (!test_has_run)
    {
        printf("Running SPI loopback test...\n");

        uint8_t master_tx_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
        uint8_t master_rx_data[sizeof(master_tx_data)] = {0};

        uint8_t slave_response_data[] = {0x11, 0x22, 0x33, 0x44};

        // --- Test Execution ---

        // 1. Load the data that the slave will send in response.
        port_spi_slave_load_data(slave_response_data, sizeof(slave_response_data));

        // 2. Master initiates the transfer.
        hal_spi_status_t status = hal_spi_transmit_receive(master_spi, master_tx_data, master_rx_data, sizeof(master_tx_data), 100);

        // --- Verification ---

        // 3. Check if the transaction was successful
        assert(status == HAL_SPI_STATUS_OK);

        // 4. Verify that the master received exactly what the slave sent
        assert(memcmp(master_rx_data, slave_response_data, sizeof(master_rx_data)) == 0);

        printf("SPI loopback test PASSED!\n");
        
        test_has_run = true;
    }
    
    // Halt the program after the test runs
    while(1) {
      // Test finished
    }
    
    return !app_terminate_get();
}