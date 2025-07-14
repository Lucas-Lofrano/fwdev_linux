#include "hal.h"
#include "utl_dbg.h"
#include "app.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>


// Using the ports from your latest socat output.
// The app will use HAL_UART_PORT0, which is /dev/pts/1.
static const char* UART_PORT_MAP[HAL_UART_NUM_PORTS] =
{
    [HAL_UART_PORT0] = "/dev/pts/1",
    [HAL_UART_PORT1] = "/dev/pts/2",
};

// Internal structure to hold the state of a UART device
struct hal_uart_dev_s
{
    int fd;                             // File descriptor for the serial port
    hal_uart_config_t config;           // UART port configuration
    hal_uart_port_t port_num;           // HAL port number
    pthread_t reader_thread_id;         // ID of the asynchronous reader thread
    volatile bool reader_thread_running; // Flag to control the thread's execution
};

static void* port_uart_rx_thread(void* arg)
{
    hal_uart_dev_t dev = (hal_uart_dev_t)arg;
    uint8_t buffer[64];

    while (dev->reader_thread_running)
    {
        ssize_t bytes_read = read(dev->fd, buffer, sizeof(buffer));
        if (bytes_read > 0)
        {
            for (ssize_t i = 0; i < bytes_read; i++)
            {
                if (dev->config.interrupt_callback)
                {
                    dev->config.interrupt_callback(buffer[i]);
                }
            }
        }
        else
        {
            usleep(10000); // 10ms sleep to prevent busy-waiting
        }
    }
    return NULL;
}

static void port_uart_init(void) { /* Handled by OS */ }
static void port_uart_deinit(void) { /* Handled by OS */ }

static hal_uart_dev_t port_uart_open(hal_uart_port_t port, hal_uart_config_t* cfg)
{
    if (port >= HAL_UART_NUM_PORTS || cfg == NULL) return NULL;

    hal_uart_dev_t device = (hal_uart_dev_t)calloc(1, sizeof(struct hal_uart_dev_s));
    if (device == NULL) return NULL;

    device->port_num = port;
    device->config   = *cfg;

    device->fd = open(UART_PORT_MAP[port], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (device->fd < 0)
    {
        UTL_DBG_PRINTF(UTL_DBG_MOD_PORT, "ERROR: open() failed for %s: %s\n", UART_PORT_MAP[port], strerror(errno));
        free(device);
        return NULL;
    }

    struct termios config;
    if (tcgetattr(device->fd, &config) < 0) {
        close(device->fd);
        free(device);
        return NULL;
    }

    speed_t speed;
    switch (cfg->baud_rate) {
        case HAL_UART_BAUD_RATE_9600:   speed = B9600;   break;
        case HAL_UART_BAUD_RATE_19200:  speed = B19200;  break;
        case HAL_UART_BAUD_RATE_38400:  speed = B38400;  break;
        case HAL_UART_BAUD_RATE_57600:  speed = B57600;  break;
        case HAL_UART_BAUD_RATE_115200: speed = B115200; break;
        default:                        speed = B9600;   break;
    }
    cfsetispeed(&config, speed);
    cfsetospeed(&config, speed);

    config.c_cflag &= ~(PARENB | PARODD);
    if (cfg->parity == HAL_UART_PARITY_ODD) config.c_cflag |= (PARENB | PARODD);
    else if (cfg->parity == HAL_UART_PARITY_EVEN) config.c_cflag |= PARENB;

    if (cfg->stop_bits == HAL_UART_STOP_BITS_2) config.c_cflag |= CSTOPB;
    else config.c_cflag &= ~CSTOPB;

    if (cfg->flow_control == HAL_UART_FLOW_CONTROL_CTS_RTS) config.c_cflag |= CRTSCTS;
    else config.c_cflag &= ~CRTSCTS;
    
    config.c_cflag |= (CLOCAL | CREAD);
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    config.c_iflag &= ~(IXON | IXOFF | IXANY);
    config.c_oflag &= ~OPOST;

    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 1;

    tcsetattr(device->fd, TCSANOW, &config);
    tcflush(device->fd, TCIOFLUSH);

    if (cfg->interrupt_callback != NULL)
    {
        device->reader_thread_running = true;
        if (pthread_create(&device->reader_thread_id, NULL, &port_uart_rx_thread, (void*)device) != 0)
        {
            close(device->fd);
            free(device);
            return NULL;
        }
    }

    return device;
}

static void port_uart_close(hal_uart_dev_t dev)
{
    if (dev == NULL) return;
    if (dev->reader_thread_running)
    {
        dev->reader_thread_running = false;
        pthread_join(dev->reader_thread_id, NULL);
    }
    close(dev->fd);
    free(dev);
}

static size_t port_uart_bytes_available(hal_uart_dev_t dev)
{
    if (dev == NULL) return 0;
    int bytes_available = 0;
    if (ioctl(dev->fd, FIONREAD, &bytes_available) < 0) return 0;
    return (size_t)bytes_available;
}

static int32_t port_uart_read(hal_uart_dev_t dev, uint8_t* buffer, size_t size)
{
    if (dev == NULL || buffer == NULL || size == 0) return -EINVAL;
    ssize_t bytes_read = read(dev->fd, buffer, size);
    if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    return (int32_t)bytes_read;
}

// CORRECTED: Signature now matches the const-correct header.
static int32_t port_uart_write(hal_uart_dev_t dev, const uint8_t* buffer, size_t size)
{
    if (dev == NULL || buffer == NULL) return -EINVAL;
    return (int32_t)write(dev->fd, buffer, size);
}

static void port_uart_flush(hal_uart_dev_t dev)
{
    if (dev == NULL) return;
    tcdrain(dev->fd);
}

hal_uart_driver_t HAL_UART_DRIVER =
{
    .init = port_uart_init,
    .deinit = port_uart_deinit,
    .open = port_uart_open,
    .close = port_uart_close,
    .bytes_available = port_uart_bytes_available,
    .read = port_uart_read,
    // CORRECTED: No longer needs a cast as the signatures now match.
    .write = port_uart_write,
    .flush = port_uart_flush,
};
