/* Lưu một khối dữ liệu vào Flash sector 5 (0x08020000, 128 KB) của STM32F401CC.
 *
 * Firmware phải nằm gọn trong sector 0..4 (128 KB đầu). Trong linker script (STM32F401xx_FLASH.ld)
 * đặt FLASH LENGTH = 128K để linker báo lỗi nếu code to quá; Store_Save cũng tự kiểm tra lại.
 * Xoá sector 128 KB mất ~1–2 s và CPU bị treo trong lúc đó: chỉ gọi khi xe đứng yên. */
#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdbool.h>
#include <stdint.h>

/* version: tăng khi bố cục dữ liệu đổi; Store_Load từ chối dữ liệu khác version. */
bool Store_Save(const void *data, uint32_t len, uint32_t version);
bool Store_Load(void *data, uint32_t len, uint32_t version);

#endif /* FLASH_STORE_H */
