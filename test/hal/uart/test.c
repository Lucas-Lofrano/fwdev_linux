#include "hal.h"
#include "app.h"
#include "hal_uart.h"
#include <unistd.h>   // For sleep()
#include <pthread.h>  // For threading

// A global handle for our UART device
static hal_uart_dev_t uart_device;
static pthread_t sender_thread_id;
static bool sender_thread_started = false;

/**
 * @brief This callback is triggered by the reader thread for each byte received.
 */
static void uart_receive_callback(uint8_t c)
{
    utl_printf("%c", c);
    fflush(stdout);
}

/**
 * @brief Thread dedicated to sending data periodically.
 */
static void* sender_thread(void* arg)
{
    (void)arg; // Unused
    uint32_t counter = 0;
    char tx_buffer[64];

    UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "Sender thread started.\n");

    while (!app_terminate_get())
    {
        if (uart_device != NULL)
        {
            UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "Sending packet #%u\n", counter);
            int len = snprintf(tx_buffer, sizeof(tx_buffer), "Hello from UART! Counter: %u\r\n", counter++);
            hal_uart_write(uart_device, (const uint8_t*)tx_buffer, len);
        }
        sleep(1); // Wait for 1 second
    }

    UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "Sender thread exiting.\n");
    return NULL;
}

/**
 * @brief Initializes the application, configures UART, and starts threads.
 */
void app_init(void)
{
    utl_dbg_init();
    utl_dbg_mod_enable(UTL_DBG_MOD_APP);
    utl_dbg_mod_enable(UTL_DBG_MOD_PORT);

    UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "--- UART Test Application ---\n");
    UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "Attempting to open UART port...\n");

    hal_uart_config_t uart_config = {
        .baud_rate = HAL_UART_BAUD_RATE_115200,
        .parity = HAL_UART_PARITY_NONE,
        .stop_bits = HAL_UART_STOP_BITS_1,
        .flow_control = HAL_UART_FLOW_CONTROL_NONE,
        .interrupt_callback = uart_receive_callback
    };

    hal_uart_init();
    uart_device = hal_uart_open(HAL_UART_PORT0, &uart_config);

    if (uart_device == NULL)
    {
        UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "ERROR: Failed to open UART port.\n");
    }
    else
    {
        UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "SUCCESS: UART port opened. Starting sender thread.\n");
        if (pthread_create(&sender_thread_id, NULL, &sender_thread, NULL) != 0)
        {
            UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "ERROR: Failed to create sender thread.\n");
        } else {
            sender_thread_started = true;
        }
    }
}

/**
 * @brief The main application loop.
 */
bool app_loop(void)
{
    // Check if the application is trying to terminate.
    if (app_terminate_get())
    {
        // If the sender thread was started, wait for it to finish.
        if (sender_thread_started)
        {
            UTL_DBG_PRINTF(UTL_DBG_MOD_APP, "Shutdown detected, joining sender thread...\n");
            pthread_join(sender_thread_id, NULL);
            sender_thread_started = false; // Prevent re-joining
        }
        return false; // Signal to the main framework that we want to exit.
    }

    // Keep the main thread alive by sleeping.
    pause();

    // Signal to the main framework to continue the loop.
    return true;
}
