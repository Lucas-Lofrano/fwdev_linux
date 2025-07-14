#include "hal.h"
#include "app.h"
#include "hal_spi.h"

#include <pthread.h> // Include the POSIX Threads library
#include <unistd.h>  // For sleep()

// Function prototype for the external function to load slave data
void port_spi_slave_load_data(const uint8_t* data, size_t size);

// We need global handles so both threads can access the initialized devices
static hal_spi_dev_t master_spi;
static hal_spi_dev_t slave_spi;

// A semaphore or flag to signal when the test is complete
static volatile bool test_completed = false;

/**
 * @brief The function executed by the slave thread.
 * * In a real application, this thread would be in a loop, handling incoming data.
 * For this test, its main job is to prepare the response data for the master.
 */
void* slave_task(void* arg) {
    printf("Slave Task: I am alive!\n");
    
    // This is the data the slave will send back to the master.
    uint8_t slave_response_data[] = {0x11, 0x22, 0x33, 0x44};
    
    // Load the data that the slave will send in response.
    // This function populates the slave's transmit FIFO buffer.
    port_spi_slave_load_data(slave_response_data, sizeof(slave_response_data));
    printf("Slave Task: Response data loaded into my TX buffer.\n");

    // The slave's work is done for this simple test.
    // In a more complex scenario, it might wait for a signal to load more data.
    return NULL;
}

/**
 * @brief The function executed by the master thread.
 *
 * This thread initiates the SPI transaction and verifies the result.
 */
void* master_task(void* arg) {
    printf("Master Task: I am alive! Waiting a moment for the slave to be ready...\n");
    sleep(1); // Give the slave thread a moment to load its data.

    // --- Data Buffers ---
    uint8_t master_tx_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t master_rx_data[sizeof(master_tx_data)] = {0};
    uint8_t expected_slave_response[] = {0x11, 0x22, 0x33, 0x44};

    printf("Master Task: Initiating SPI transfer...\n");
    
    // --- Test Execution ---
    // Master initiates the full-duplex transfer.
    hal_spi_status_t status = hal_spi_transmit_receive(
        master_spi, 
        master_tx_data, 
        master_rx_data, 
        sizeof(master_tx_data), 
        100 // Timeout in ms
    );

    // --- Verification ---
    printf("Master Task: Verifying the transaction...\n");

    // 1. Check if the transaction status was OK
    assert(status == HAL_SPI_STATUS_OK);

    // 2. Verify that the master received exactly what the slave sent
    assert(memcmp(master_rx_data, expected_slave_response, sizeof(master_rx_data)) == 0);

    // If we got here, the asserts passed!
    printf("✅ SPI loopback test PASSED!\n");
    
    // Signal that the test is finished
    test_completed = true;
    return NULL;
}

/**
 * @brief Initializes the application and SPI devices before starting threads.
 */
void app_init(void) {
    printf("Initializing application and SPI devices...\n");

    // 1. Initialize the HAL
    hal_init();

    // 2. Configure the Master device
    hal_spi_config_t master_cfg = {
        .role = HAL_SPI_ROLE_MASTER,
        .data_size = HAL_SPI_DATA_SIZE_8BITS,
        // Other parameters can be left as default for this test
    };

    // 3. Configure the Slave device
    hal_spi_config_t slave_cfg = {
        .role = HAL_SPI_ROLE_SLAVE,
        .data_size = HAL_SPI_DATA_SIZE_8BITS,
    };

    // 4. Initialize both devices.
    master_spi = hal_spi_init(HAL_SPI_DEVICE_0, &master_cfg);
    slave_spi = hal_spi_init(HAL_SPI_DEVICE_0, &slave_cfg); // Using the same physical device ID

    // Ensure initialization was successful
    assert(master_spi != NULL);
    assert(slave_spi != NULL);

    printf("SPI Master and Slave initialized successfully.\n");
}

/**
 * @brief The main application loop, now responsible for thread management.
 */
bool app_loop(void) {
    printf("test_completed = %d\n", test_completed);
    if (!test_completed) {
        pthread_t master_thread_id, slave_thread_id;

        printf("App Loop: Creating slave and master threads...\n");

        // Create and start the slave and master threads
        pthread_create(&slave_thread_id, NULL, slave_task, NULL);
        pthread_create(&master_thread_id, NULL, master_task, NULL);

        // Wait for both threads to complete their execution
        pthread_join(slave_thread_id, NULL);
        pthread_join(master_thread_id, NULL);
        
        printf("App Loop: All threads have completed.\n");
    }
    
    // Halt the program after the test runs
    while(1) {
      // Test finished, infinite loop to halt.
    }
    
    return !app_terminate_get(); // This part might not be reached
}