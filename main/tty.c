#include <user_interface.h>
#include <osapi.h>

#include "uart.h"
#include "hw_timer.h"
#include "tty.h"
#include "gpio_drv.h"

#define TTY_BSIZE           512
#define TTY_BITBANG_BITS    10
#define TTY_TIMEOUT         50

/* UART 1 - GPIO 5 */
#define UART1_RX_PIN        5
#define UART1_RX_MUX        PERIPHS_IO_MUX_GPIO5_U
#define UART1_RX_FUNC       FUNC_GPIO5
#define UART1_RX_INTR       0

/* UART 2 - GPIO 4 and GPIO 15 */
#define UART2               2
#define UART2_RX_PIN        4
#define UART2_RX_MUX        PERIPHS_IO_MUX_GPIO4_U
#define UART2_RX_FUNC       FUNC_GPIO4
#define UART2_RX_INTR       1
#define UART2_TX_PIN        15
#define UART2_TX_MUX        PERIPHS_IO_MUX_MTDO_U
#define UART2_TX_FUNC       FUNC_GPIO15

#define TTY_FIFO_CNT(head,tail,size) \
    ((tail >= head) ? (tail - head) : (size - (head - tail)))
#define TTY_FIFO_SPACE(head,tail,size) \
    ((tail >= head) ? (size - (tail - head) - 1) : (head - tail - 1))

typedef struct _tty_dev {
    int tty;
    tty_func_t func;
    void *user_data;
    uint8_t recv_buf[TTY_BSIZE];
    uint16_t recv_head;
    uint16_t recv_tail;
    uint32_t recv_xsr;
    uint8_t recv_bits;
    uint8_t xmit_buf[TTY_BSIZE];
    uint16_t xmit_head;
    uint16_t xmit_tail;
    uint32_t xmit_xsr;
    uint8_t xmit_bits;
} tty_dev_t;

static tty_dev_t tty_dev[TTY_NUM_DEV] = { 
    {
        .tty = 0,
        .func = NULL,
        .user_data = NULL,
    },
    {   
        .tty = 1,
        .func = NULL,
        .user_data = NULL,
    },
    {   
        .tty = 2,
        .func = NULL,
        .user_data = NULL,
    }
};

static bool hw_timer_enabled = FALSE;
static uint32_t hw_timer_counter = 0;
static os_timer_t tty_timer;
static os_event_t tty_event;


static tty_hw_timer_enable(void)
{
    if (!hw_timer_enabled) {
        hw_timer_enabled = TRUE;
        hw_timer_counter = 0;
        hw_timer_arm(52, TRUE);
    }
}

static tty_hw_timer_disable(void)
{
    if (hw_timer_enabled) {
        hw_timer_enabled = FALSE;
        hw_timer_counter = 0;
        hw_timer_disarm();
    }
}

static void tty_gpio_interrupt(int intr, void *user_data)
{
    tty_dev_t *p = user_data;

    ets_intr_lock();

    if (p->tty == UART1) {
        p->recv_bits = TTY_BITBANG_BITS;
        tty_hw_timer_enable();
    } else if (p->tty == UART2) {
        p->recv_bits = TTY_BITBANG_BITS;
        tty_hw_timer_enable();
    }

    ets_intr_unlock();
}

static void tty_hw_timeout(void)
{
    tty_dev_t *p;
    uint32_t bit;
    int rc = 0;

    ets_intr_lock();

    /* Only receive */
    p = &tty_dev[UART1];
    if (p->func) {
        if (!(hw_timer_counter & 1)) {
            if (p->recv_bits > 0) {
                p->recv_bits--;
                /* Read GPIO */
                bit = GPIO_INPUT_GET(UART1_RX_PIN);
                p->recv_xsr >>= 1;
                p->recv_xsr |= (bit << (TTY_BITBANG_BITS - 1));
                if (!p->recv_bits) {
                    /* Check stop bit */
                    if (bit) {
                        p->recv_xsr >>= 1;
                        p->recv_buf[p->recv_tail] = p->recv_xsr & 0xFF;
                        p->recv_tail = ++p->recv_tail & (TTY_BSIZE - 1);
                    }
                    p->recv_xsr = 0;
                    /* Enable interrupt */
                    gpio_pin_intr_state_set(UART1_RX_PIN, GPIO_PIN_INTR_NEGEDGE);
                }
            }
        }
        /* Work pending */
        if (p->recv_bits) {
            rc++;
        }
    }

    /* Transmit and Receive */
    p = &tty_dev[UART2];
    if (p->func) {
        if (!(hw_timer_counter & 1)) {
            if (p->recv_bits > 0) {
                p->recv_bits--;
                /* Read GPIO */
                bit = GPIO_INPUT_GET(UART2_RX_PIN);
                p->recv_xsr >>= 1;
                p->recv_xsr |= (bit << (TTY_BITBANG_BITS - 1));
                if (!p->recv_bits) {
                    /* Check stop bit */
                    if (bit) {
                        p->recv_xsr >>= 1;
                        p->recv_buf[p->recv_tail] = p->recv_xsr & 0xFF;
                        p->recv_tail = ++p->recv_tail & (TTY_BSIZE - 1);
                    }
                    p->recv_xsr = 0;
                    /* Enable interrupt */
                    gpio_pin_intr_state_set(UART2_RX_PIN, GPIO_PIN_INTR_NEGEDGE);
                }
            }
            if (p->xmit_head != p->xmit_tail) {
                if (!p->xmit_bits) {
                    p->xmit_xsr = p->xmit_buf[p->xmit_head];
                    /* Put start and stop bit */
                    p->xmit_xsr <<= 1;
                    p->xmit_xsr |= (1 << (TTY_BITBANG_BITS - 1));
                    p->xmit_bits = TTY_BITBANG_BITS;
                }
                /* Write GPIO */
                bit = (p->xmit_xsr >> (TTY_BITBANG_BITS - p->xmit_bits)) & 1;
                GPIO_OUTPUT_SET(UART2_TX_PIN, bit);
                p->xmit_bits--;
                if (!p->xmit_bits)
                    p->xmit_head = ++p->xmit_head & (TTY_BSIZE - 1);
            }
        }
        /* Work pending */
        if (p->recv_bits || p->xmit_bits || p->xmit_head != p->xmit_tail) {
            rc++;
        }
    }

    hw_timer_counter++;

    if (!rc)
        tty_hw_timer_disable();

    ets_intr_unlock();
}

ICACHE_FLASH_ATTR
static int tty_read_fifo(int tty, uint8_t *buf, int len)
{
    tty_dev_t *p;
    int size;
    int i;

    p = &tty_dev[tty];

    size = TTY_FIFO_CNT(p->recv_head, p->recv_tail, TTY_BSIZE);
    if (size) {
        if (size > len) size = len;
        for (i = 0; i < size; i++) {
            buf[i] = p->recv_buf[p->recv_head];
            p->recv_head = ++p->recv_head & (TTY_BSIZE - 1);
        }
    }

    return size;
}

ICACHE_FLASH_ATTR
static int tty_write_fifo(int tty, const uint8_t *buf, int len)
{
    tty_dev_t *p;
    int rc = -1;
    int size;
    int i;

    p = &tty_dev[tty];

    size = TTY_FIFO_SPACE(p->xmit_head, p->xmit_tail, TTY_BSIZE);
    if (size >= len) {
        for (i = 0; i < len; i++) {
            p->xmit_buf[p->xmit_tail] = buf[i];
            p->xmit_tail = ++p->xmit_tail & (TTY_BSIZE - 1);
        }
        rc = OK;
    }

    /* Enable UART2 bitbang */
    if (tty == UART2)
        tty_hw_timer_enable();

    return rc;
}

static void tty_task(os_event_t *events)
{
    char buf[TTY_BSIZE];
    tty_dev_t *p;
    int size;
    int len;
    int i;

    if (events->sig == 0) {
        p = &tty_dev[UART0];
        if (p->func) {
            size = uart_read_fifo(UART0, buf, sizeof(buf));
            if (size)
                p->func(UART0, buf, size, p->user_data);
        }
    } else if (events->sig == 1) {
        p = &tty_dev[UART1];
        if (p->func) {
            size = TTY_FIFO_CNT(p->xmit_head, p->xmit_tail, TTY_BSIZE);
            if (size) {
                len = UART_FIFO_LEN - ((READ_PERI_REG(UART_STATUS(UART1)) >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT);
                if (size > len) size = len;
                for (i = 0; i < size; i++) {
                    WRITE_PERI_REG(UART_FIFO(UART1), p->xmit_buf[p->xmit_head]);
                    p->xmit_head = ++p->xmit_head & (TTY_BSIZE - 1);
                }
            }
            size = tty_read_fifo(UART1, buf, sizeof(buf));
            if (size)
                p->func(UART1, buf, size, p->user_data);
        }
        p = &tty_dev[UART2];
        if (p->func) {
            size = tty_read_fifo(UART2, buf, sizeof(buf));
            if (size)
                p->func(UART2, buf, size, p->user_data);
        }
    }
}

ICACHE_FLASH_ATTR
int tty_init(void)
{
    os_info("TTY", "TTY init");

    uart_init(BIT_RATE_115200, BIT_RATE_9600, tty_task);

    tty_event.sig = 1;
    tty_event.par = 0;
    os_timer_setfn(&tty_timer, (os_timer_func_t *)tty_task, &tty_event);
    os_timer_arm(&tty_timer, TTY_TIMEOUT, TRUE);

    gpio_drv_init();

    /* Configure GPIO and Hardware timer */
    PIN_FUNC_SELECT(UART1_RX_MUX, UART1_RX_FUNC);
    GPIO_DIS_OUTPUT(UART1_RX_PIN);
    PIN_FUNC_SELECT(UART2_RX_MUX, UART2_RX_FUNC);
    GPIO_DIS_OUTPUT(UART2_RX_PIN);
    PIN_FUNC_SELECT(UART2_TX_MUX, UART2_TX_FUNC);
    GPIO_OUTPUT_SET(UART2_TX_PIN, 1);

    hw_timer_init();
    hw_timer_set_func(tty_hw_timeout);
}

ICACHE_FLASH_ATTR
void tty_release(void)
{
    os_timer_disarm(&tty_timer);
    gpio_drv_release();
    hw_timer_disarm();
}

ICACHE_FLASH_ATTR
int tty_open(int tty, tty_func_t func,
             void *user_data)
{
    tty_dev_t *p;

    if (tty >= TTY_NUM_DEV) return -1;

    os_info("TTY", "TTY: %d open", tty);

    if (func) {
        p = &tty_dev[tty];
        switch (tty) {
            case UART0:
                uart_rx_intr_enable(UART0);
                break;
            case UART1:
                gpio_interrupt_open(UART1_RX_INTR, UART1_RX_PIN,
                                    GPIO_PIN_INTR_NEGEDGE, GPIO_INTR_DISABLED,
                                    tty_gpio_interrupt, p); 
                break;
            case UART2:
                gpio_interrupt_open(UART2_RX_INTR, UART2_RX_PIN,
                                    GPIO_PIN_INTR_NEGEDGE, GPIO_INTR_DISABLED,
                                    tty_gpio_interrupt, p); 
                break;
        }
        p->func = func;
        p->user_data = user_data;
        p->recv_head = p->recv_tail = 0;
        p->recv_xsr = 0;
        p->recv_bits = 0;
        p->xmit_head = p->xmit_tail = 0;
        p->xmit_xsr = 0;
        p->xmit_bits = 0;
    }

    return 0;
}

ICACHE_FLASH_ATTR
int tty_close(int tty)
{
    tty_dev_t *p;

    if (tty >= TTY_NUM_DEV) return -1;

    os_info("TTY", "TTY: %d close", tty);

    p = &tty_dev[tty];
    switch (tty) {
        case UART0:
            uart_rx_intr_disable(UART0);
            break;
        case UART1:
            gpio_interrupt_close(0);
            break;
        case UART2:
            gpio_interrupt_close(1);
            break;
    }
    p->func = NULL;
    p->user_data = NULL;
    p->recv_head = p->recv_tail = 0;
    p->recv_xsr = 0;
    p->recv_bits = 0;
    p->xmit_head = p->xmit_tail = 0;
    p->xmit_xsr = 0;
    p->xmit_bits = 0;

    return 0;
}

ICACHE_FLASH_ATTR
int tty_write(int tty, const uint8_t *data, int len)
{
    tty_dev_t *p;
    int rc = -1;
    int size;
    int i;

    if (tty >= TTY_NUM_DEV) return -1;

#ifdef DEBUG
    os_debug("TTY", "TTY: %d write %d bytes", tty, len);
#endif

    p = &tty_dev[tty];
    
    switch (tty) {
        case UART0:
            for (i = 0; i < len; i++)
                rc = uart_tx_one_char(tty, data[i]);
            break;
        case UART1:
            rc = tty_write_fifo(tty, data, len);
            if (!rc) {
                size = TTY_FIFO_CNT(p->xmit_head, p->xmit_tail, TTY_BSIZE);
                len = UART_FIFO_LEN - ((READ_PERI_REG(UART_STATUS(tty)) >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT);
                if (size > len) size = len;
                for (i = 0; i < size; i++) {
                    WRITE_PERI_REG(UART_FIFO(tty), p->xmit_buf[p->xmit_head]);
                    p->xmit_head = ++p->xmit_head & (TTY_BSIZE - 1);
                }
            }
            break;
        case UART2:
            rc = tty_write_fifo(tty, data, len);
            break;
    }

    return rc;
}

ICACHE_FLASH_ATTR
int tty_tx_fifo_size(int tty)
{
    tty_dev_t *p;
    int size;

    if (tty == UART0) {
        /* UART FIFO */
        size = ((READ_PERI_REG(UART_STATUS(tty)) >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT);
        return size;
    } else if (tty == UART1) {
        p = &tty_dev[tty];
        size = TTY_FIFO_CNT(p->xmit_head, p->xmit_tail, TTY_BSIZE);
        /* UART FIFO */
        size += ((READ_PERI_REG(UART_STATUS(tty)) >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT);
        return size;
    } else if (tty == UART2) {
        p = &tty_dev[tty];
        size = TTY_FIFO_CNT(p->xmit_head, p->xmit_tail, TTY_BSIZE);
        return size;
    }

    return 0;
}

ICACHE_FLASH_ATTR
void tty_set_baudrate(int tty, int baudrate)
{
    switch (tty) {
        case UART0:
            UART_SetBaudrate(UART0, baudrate);
            break;
        default:
            /* Not allowed */
            break;
    }
}

ICACHE_FLASH_ATTR
void tty_set_parity(int tty, int mode)
{
    switch (tty) {
        case UART0:
            UART_SetParity(tty, mode);
            break;
        default:
            /* Not allowed */
            break;
    }
}
