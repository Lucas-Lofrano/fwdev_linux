#include "app.h"
#include "hal.h"
#include "utl_dbg.h"
#include <pthread.h> // For synchronization

// --- Better: A realistic FIFO buffer for device data ---
#define SPI_FIFO_SIZE 256

typedef struct {
    uint8_t buffer[SPI_FIFO_SIZE];
    size_t head;
    size_t tail;
    size_t count;
} spi_fifo_t;

// --- Better: A structure representing the physical bus ---
typedef struct {
    uint8_t mosi; // Master Out, Slave In
    uint8_t miso; // Master In, Slave Out

    // Synchronization primitives to simulate the SPI clock (SCK)
    pthread_mutex_t lock;
    pthread_cond_t master_cond; // Slave signals when MISO is ready
    pthread_cond_t slave_cond;  // Master signals when MOSI is ready
} port_spi_bus_t;

// --- The actual device handle ---
struct hal_spi_dev_s {
    hal_spi_device_t device;
    hal_spi_config_t cfg;
    port_spi_bus_t *bus; // Pointer to the shared bus
    spi_fifo_t tx_fifo;
    spi_fifo_t rx_fifo;
};

// --- Global, shared resources for the simulation ---
static port_spi_bus_t shared_bus;
static struct hal_spi_dev_s port_spi_devs[HAL_SPI_DEVICE_NUM_DEVICES]; // Master and Slave
static bool bus_initialized = false;

// Helper to initialize a FIFO
static void fifo_init(spi_fifo_t *fifo) {
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

// Pre-loads the slave's transmit FIFO buffer
void port_spi_slave_load_data(const uint8_t* data, size_t size) {
    struct hal_spi_dev_s *slave_dev = &port_spi_devs[1]; // Assuming dev 1 is slave
    fifo_init(&slave_dev->tx_fifo);
    for (size_t i = 0; i < size && i < SPI_FIFO_SIZE; ++i) {
        slave_dev->tx_fifo.buffer[slave_dev->tx_fifo.head] = data[i];
        slave_dev->tx_fifo.head = (slave_dev->tx_fifo.head + 1) % SPI_FIFO_SIZE;
        slave_dev->tx_fifo.count++;
    }
}

static hal_spi_dev_t port_spi_init(hal_spi_device_t device, hal_spi_config_t* cfg) {
    if (device >= HAL_SPI_DEVICE_NUM_DEVICES) return NULL;

    // One-time initialization of the shared bus and its mutex/condition variables
    if (!bus_initialized) {
        pthread_mutex_init(&shared_bus.lock, NULL);
        pthread_cond_init(&shared_bus.master_cond, NULL);
        pthread_cond_init(&shared_bus.slave_cond, NULL);
        bus_initialized = true;
    }

    struct hal_spi_dev_s *dev;
    if (cfg->role == HAL_SPI_ROLE_MASTER) {
        dev = &port_spi_devs[0];
    } else {
        dev = &port_spi_devs[1];
    }

    dev->device = device;
    dev->bus = &shared_bus; // Both devices point to the same bus
    memcpy(&dev->cfg, cfg, sizeof(hal_spi_config_t));

    // Initialize device-specific FIFOs
    fifo_init(&dev->tx_fifo);
    fifo_init(&dev->rx_fifo);

    return dev;
}

static void port_spi_deinit(hal_spi_dev_t *pdev) {
    if (pdev && *pdev) {
        // In a real scenario, you might destroy the mutex/cond if no devices are left
        *pdev = NULL;
    }
}

// This function now simulates a realistic, synchronized, byte-by-byte transfer
static hal_spi_status_t port_spi_transmit_receive(hal_spi_dev_t dev, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t size, uint32_t timeout_ms) {
    if (!dev || !dev->bus) return HAL_SPI_STATUS_INVALID_PARAM;

    // For this simulation, we hard-code the roles. A more complex system could look them up.
    hal_spi_dev_t master_dev = &port_spi_devs[0];
    hal_spi_dev_t slave_dev = &port_spi_devs[1];

    if (dev->cfg.role != HAL_SPI_ROLE_MASTER) {
         // In this model, only the master can initiate a transfer.
        return HAL_SPI_STATUS_ERROR;
    }

    for (size_t i = 0; i < size; ++i) {
        pthread_mutex_lock(&dev->bus->lock);

        // --- Master side ---
        // 1. Master places byte on MOSI line
        dev->bus->mosi = (tx_buffer) ? tx_buffer[i] : 0xFF;

        // --- Slave side (simulated) ---
        // 2. Slave reads from MOSI
        // In a real multi-threaded app, a slave thread would do this. Here we simulate it.
        uint8_t received_by_slave = dev->bus->mosi;
        
        // 3. Slave gets its response byte from its TX fifo
        uint8_t byte_from_slave = 0xFF; // Default value if slave has nothing to send
        if (slave_dev->tx_fifo.count > 0) {
            byte_from_slave = slave_dev->tx_fifo.buffer[slave_dev->tx_fifo.tail];
            slave_dev->tx_fifo.tail = (slave_dev->tx_fifo.tail + 1) % SPI_FIFO_SIZE;
            slave_dev->tx_fifo.count--;
        }
        
        // 4. Slave places its response on the MISO line
        dev->bus->miso = byte_from_slave;

        // --- Master side ---
        // 5. Master reads the MISO line
        if (rx_buffer) {
            rx_buffer[i] = dev->bus->miso;
        }

        pthread_mutex_unlock(&dev->bus->lock);
        // Note: A real implementation would use condition variables to block/wake threads.
        // For a single-threaded simulation, this direct access inside a mutex is sufficient
        // to show the synchronized data exchange concept.
    }

    return HAL_SPI_STATUS_OK;
}

hal_spi_driver_t HAL_SPI_DRIVER = {
    .init = port_spi_init,
    .deinit = port_spi_deinit,
    .transmit_receive = port_spi_transmit_receive
};