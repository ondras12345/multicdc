#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

#include <zephyr/shell/shell.h>
#include <pico/bootrom.h>

#include <zephyr/drivers/watchdog.h>


BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Console device is not ACM CDC UART device");

LOG_MODULE_REGISTER(multicdc, LOG_LEVEL_INF);

// We might not need this much (especially for non-CDC channels which can just
// pause reception on the CDC counterpart), but there's still a lot of RAM
// left.
#define RING_BUF_SIZE 10240

struct serial_channel {
    const struct device * dev;
    struct serial_channel * other;
    struct ring_buf rb;
    uint8_t buf[RING_BUF_SIZE];
    bool rx_throttled;
    uint8_t id;
};

struct serial_channel serial_channels[] = {
    // masters
    { .dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0)), },  // master 1
    { .dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart2)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart3)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart4)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart5)), },
    // slaves
    { .dev = DEVICE_DT_GET(DT_NODELABEL(uart0)), },  // slave 1
    { .dev = DEVICE_DT_GET(DT_NODELABEL(pio0_uart0)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(uart1)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(pio0_uart1)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(pio1_uart0)), },
    { .dev = DEVICE_DT_GET(DT_NODELABEL(pio1_uart1)), },
    // Master n is connected to slave n.
};

const unsigned n_ch_pairs = ARRAY_SIZE(serial_channels)/2;
uint16_t polling_channels = 0x00;

// thread stacks cannot be allocated dynamically on the heap
K_THREAD_STACK_DEFINE(stack_1, 256);
K_THREAD_STACK_DEFINE(stack_2, 256);
K_THREAD_STACK_DEFINE(stack_3, 256);
K_THREAD_STACK_DEFINE(stack_4, 256);

k_thread_stack_t * stacks[] = {
    stack_1,
    stack_2,
    stack_3,
    stack_4,
    NULL
};

unsigned stack_sizes[] = {
    K_THREAD_STACK_SIZEOF(stack_1),
    K_THREAD_STACK_SIZEOF(stack_2),
    K_THREAD_STACK_SIZEOF(stack_3),
    K_THREAD_STACK_SIZEOF(stack_4),
};

struct k_thread thread_datas[4];

const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));


static void interrupt_handler(const struct device *dev, void *user_data)
{
    struct serial_channel * channel = user_data;

    while (uart_irq_update(dev) && uart_irq_is_pending(dev))
    {
        if (!channel->rx_throttled && uart_irq_rx_ready(dev))
        {
            struct ring_buf *rb = &channel->other->rb;

            uint8_t buffer[64];
            size_t len = MIN(ring_buf_space_get(rb), sizeof(buffer));

            if (len == 0)
            {
                LOG_DBG("ch %u: throttling rx", channel->id);
                channel->rx_throttled = true;
                uart_irq_rx_disable(dev);
            }

            int recv_len = uart_fifo_read(dev, buffer, len);
            LOG_DBG("ch %u: ISR rx %u bytes", channel->id, recv_len);
            if (recv_len < 0) {
                LOG_ERR("ch %u: failed to read UART FIFO", channel->id);
                recv_len = 0;
            };

            int rb_len = ring_buf_put(rb, buffer, recv_len);
            if (rb_len < recv_len) {
                LOG_ERR("ch %u: ISR rx drop %u bytes", channel->id, recv_len - rb_len);
            }

            if (rb_len) {
                uart_irq_tx_enable(channel->other->dev);
            }
        }

        if (uart_irq_tx_ready(dev))
        {
            uint8_t buffer[64];  // max packet size for CDC

            int rb_len = ring_buf_peek(&channel->rb, buffer, sizeof(buffer));

            if (!rb_len) {
                LOG_DBG("ch %u: ring buffer empty, disable TX IRQ", channel->id);
                uart_irq_tx_disable(dev);
                continue;
            }

            int send_len = uart_fifo_fill(dev, buffer, rb_len);
            LOG_DBG("ch %u: ISR tx %u bytes", channel->id, send_len);
            // actually remove the bytes from the ring buffer
            ring_buf_get(&channel->rb, buffer, send_len);
            // allow further rx
            if (channel->other->rx_throttled)
            {
                LOG_DBG("ch %u: unthrottling rx", channel->other->id);
                channel->other->rx_throttled = false;
                uart_irq_rx_enable(channel->other->dev);
            }
        }
    }
}


void polling_timer_handler(struct k_timer *dummy)
{
    ARG_UNUSED(dummy);

    for (unsigned i = 0; i < ARRAY_SIZE(serial_channels); i++)
    {
        const struct serial_channel * channel = &serial_channels[i];
        if (!(polling_channels & (1<<i))) continue;
        // receive everything from this channel
        char c;
        size_t bytes_read = 0;
        while (!uart_poll_in(channel->dev, &c))
        {
            bytes_read++;
            LOG_DBG("ch %u: rx 1 byte '%c'", channel->id, c);
            if (ring_buf_put(&channel->other->rb, &c, 1))
            {
                uart_irq_tx_enable(channel->other->dev);
            }
            // There's no good way around dropping the byte,
            // the rx FIFO is extremely short.
            else LOG_ERR("ch %u: rx drop 1 byte", channel->id);
        }
        if (bytes_read > 2)
        {
            LOG_WRN("ch %u: rx >2 bytes, FIFO overrun possible", channel->id);
        }
    }
}

K_TIMER_DEFINE(polling_timer, polling_timer_handler, NULL);


/// thread entry point for polling uart
void polling_echo(void * p1, void * p2, void * p3)
{
    struct serial_channel * channel = p1;
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (;;)
    {
        // uart_poll_out is a blocking call, hence the need for a separate
        // thread
        char c;
        while (ring_buf_get(&channel->rb, &c, 1))
        {
            if (channel->other->rx_throttled)
            {
                LOG_DBG("ch %u: unthrottling rx", channel->id);
                channel->other->rx_throttled = false;
                uart_irq_rx_enable(channel->other->dev);
            }

            LOG_DBG("ch %u: tx 1 byte '%c'", channel->id, c);
            uart_poll_out(channel->dev, c);
        }
        // If we fall out of the while loop, there is nothing to tx, and
        // there's no blocking call in the loop.
        // We need to yield to let other threads do their thing.
        k_sleep(K_USEC(50));
    }
}


static int cmd_bootload(const struct shell *sh, size_t argc, char **argv)
{
    wdt_disable(wdt);  // TODO is this needed?
    shell_print(sh, "jumping to bootloader");
    k_sleep(K_MSEC(500));
    // TODO this sometimes fails
    reset_usb_boot(0, 0);
    return 0;
}

SHELL_CMD_ARG_REGISTER(bootload, NULL, "jump to bootloader", cmd_bootload, 1, 0);

int main(void)
{
    if (!device_is_ready(wdt)) {
        printk("%s: device not ready.\n", wdt->name);
        return 0;
    }
    struct wdt_timeout_cfg wdt_config = {
        /* Reset SoC when watchdog timer expires. */
        .flags = WDT_FLAG_RESET_SOC,

        /* Expire watchdog after max window */
        .window.min = 0U,
        .window.max = 2000U,
    };
    int wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if (wdt_channel_id < 0)
    {
        LOG_ERR("watchdog install error");
        return 0;
    }

    if (wdt_setup(wdt, 0) < 0)
    {
        LOG_ERR("watchdog setup error");
        return 0;
    }

    wdt_feed(wdt, wdt_channel_id);

    if (usb_enable(NULL)) {
        LOG_ERR("Failed to enable USB");
        return 0;
    }

    for (unsigned i = 0; i < ARRAY_SIZE(serial_channels); i++)
    {
        serial_channels[i].id = i;
        if (!device_is_ready(serial_channels[i].dev)) {
            LOG_ERR("ch %d not ready", i);
            return 0;
        }

        ring_buf_init(&serial_channels[i].rb, sizeof(serial_channels[i].buf), serial_channels[i].buf);
    }

    // connect channels to each other
    for (unsigned i = 0; i < n_ch_pairs; i++)
    {
        serial_channels[i].other = &serial_channels[n_ch_pairs+i];
        serial_channels[n_ch_pairs+i].other = &serial_channels[i];
    }

    /* Wait 100ms for the host to do all settings */
    //k_msleep(100);

    for (unsigned i = 0; i < ARRAY_SIZE(serial_channels); i++)
    {
        int ret = uart_irq_callback_user_data_set(serial_channels[i].dev, interrupt_handler, &serial_channels[i]);
        // pio uart does not support interrupt API
        if (ret) polling_channels |= (1<<i);
    }

    LOG_WRN("channels do not support interrupts: %x", polling_channels);

    unsigned stack_id = 0;
    for (unsigned i = 0; i < ARRAY_SIZE(serial_channels); i++)
    {
        const struct serial_channel * channel = &serial_channels[i];
        if (polling_channels & (1<<i))
        {
            LOG_INF("creating thread for channel %i", i);
            if (stacks[stack_id] == NULL)
            {
                LOG_ERR("ran out of stacks for thread");
                k_sleep(K_MSEC(5));
                return 0;
            }

            k_thread_create(&thread_datas[stack_id], stacks[stack_id],
                    stack_sizes[stack_id],
                    polling_echo, (void *)channel, NULL, NULL,
                    2 /* priority */, 0, K_NO_WAIT);
            stack_id++;
        }
        else
        {
            /* Enable rx interrupts */
            uart_irq_rx_enable(channel->dev);
        }
    }

    /* start polling rx timer */
    /* 50 us should be frequent enough @ 115200 baud */
    k_timer_start(&polling_timer, K_USEC(50), K_USEC(50));

    for (;;)
    {
        // set the correct baud rate on all port pairs
        for (unsigned i = 0; i < n_ch_pairs; i++)
        {
            const struct device * master_dev = serial_channels[i].dev;
            const struct device * slave_dev = serial_channels[n_ch_pairs+i].dev;

            uint32_t baudrate;
            int ret = uart_line_ctrl_get(master_dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
            if (ret) {
                LOG_WRN("Failed to get baudrate, ret code %d", ret);
                continue;
            }

            struct uart_config cfg;
            // TODO pio UARTs don't support setting baudrate at runtime
            if ((ret = uart_config_get(slave_dev, &cfg)))
            {
                //LOG_WRN("Failed to get UART config on channel %d, ret code %d", i, ret);
                continue;
            }
            if (baudrate != cfg.baudrate)
            {
                LOG_INF("changing baud rate on channel %d to %u", i, baudrate);
                cfg.baudrate = baudrate;
                if ((ret = uart_configure(slave_dev, &cfg)))
                {
                    LOG_ERR("Failed to set UART config on channel %d, ret code %d", i, ret);
                    continue;
                }
            }
        }

        wdt_feed(wdt, wdt_channel_id);
        k_sleep(K_MSEC(10));
    }

    return 0;
}
