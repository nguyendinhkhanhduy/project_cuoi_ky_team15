/* UART (HC-05) không chặn, không cần ngắt: hai bộ đệm vòng, Uart_Service() dò cờ RXNE/TXE và chuyển
 * từng byte. Gọi Uart_Service() thật thường xuyên (mỗi vòng main và giữa các giao dịch I2C):
 * ở 9600 baud mỗi byte cách nhau ~1 ms. */
#ifndef UART_IO_H
#define UART_IO_H

#include <stdint.h>

void Uart_Service(void);
int  Uart_Getc(void);                     /* -1 nếu chưa có byte */
void Uart_Write(const char *s);           /* đệm đầy thì bỏ phần thừa (không chặn) */
void Uart_Printf(const char *fmt, ...);   /* không hỗ trợ %f (newlib-nano): dùng Uart_FmtFloat */
void Uart_Flush(uint32_t timeout_ms);     /* chờ gửi hết, chỉ dùng khi xe đứng yên */
uint16_t Uart_TxFree(void);

/* Ghi số thực dạng "-12.345" (3 chữ số thập phân) vào buf, trả về buf. */
char *Uart_FmtFloat(char *buf, float v);

#endif /* UART_IO_H */
