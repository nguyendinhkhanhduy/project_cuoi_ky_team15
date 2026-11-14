#include "uart_io.h"

#include <stdarg.h>
#include <stdio.h>

#include "board_config.h"

#define RX_LEN 128U
#define TX_LEN 512U   /* ~0,5 s ở 9600 baud */

static volatile uint8_t rx_buf[RX_LEN];
static volatile uint16_t rx_head, rx_tail;
static uint8_t tx_buf[TX_LEN];
static uint16_t tx_head, tx_tail;

void Uart_Service(void)
{
    USART_TypeDef *u = BOARD_UART->Instance;
    uint32_t sr = u->SR;
    /* Đọc SR rồi DR vừa lấy byte vừa xoá cờ overrun (ORE), nên không mất byte đang nằm trong DR. */
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t c = (uint8_t)(u->DR & 0xFFU);
        uint16_t next = (uint16_t)((rx_head + 1U) % RX_LEN);
        if (next != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
    if ((sr & USART_SR_TXE) && tx_tail != tx_head) {
        u->DR = tx_buf[tx_tail];
        tx_tail = (uint16_t)((tx_tail + 1U) % TX_LEN);
    }
}

int Uart_Getc(void)
{
    if (rx_tail == rx_head) return -1;
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_LEN);
    return c;
}

uint16_t Uart_TxFree(void)
{
    return (uint16_t)((tx_tail + TX_LEN - tx_head - 1U) % TX_LEN);
}

void Uart_Write(const char *s)
{
    while (*s) {
        uint16_t next = (uint16_t)((tx_head + 1U) % TX_LEN);
        if (next == tx_tail) return;  /* đầy: bỏ phần còn lại, không chặn vòng điều khiển */
        tx_buf[tx_head] = (uint8_t)*s++;
        tx_head = next;
    }
}

void Uart_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) Uart_Write(buf);
}

void Uart_Flush(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (tx_tail != tx_head && (HAL_GetTick() - t0) < timeout_ms) Uart_Service();
}

char *Uart_FmtFloat(char *buf, float v)
{
    long milli = (long)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    const char *sign = (milli < 0) ? "-" : "";
    if (milli < 0) milli = -milli;
    snprintf(buf, 16, "%s%ld.%03ld", sign, milli / 1000L, milli % 1000L);
    return buf;
}
