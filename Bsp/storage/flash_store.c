#include "flash_store.h"

#include <string.h>

#include "board_config.h"

#define STORE_ADDR    0x08020000UL
#define STORE_SECTOR  FLASH_SECTOR_5
#define STORE_SIZE    (128UL * 1024UL)
#define STORE_MAGIC   0x54453135UL  /* "TE15" */

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t len;
    uint32_t crc;
} StoreHeader;

/* Ký hiệu do linker script của CubeMX định nghĩa: đầu vùng dữ liệu khởi tạo trong Flash và độ dài. */
extern uint32_t _sidata, _sdata, _edata;

static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFUL;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320UL & (0U - (c & 1U)));
    }
    return ~c;
}

static bool firmware_fits(void)
{
    uintptr_t end = (uintptr_t)&_sidata + ((uintptr_t)&_edata - (uintptr_t)&_sdata);
    return end <= STORE_ADDR;
}

bool Store_Save(const void *data, uint32_t len, uint32_t version)
{
    if (len + sizeof(StoreHeader) > STORE_SIZE || !firmware_fits()) return false;
    StoreHeader h = {STORE_MAGIC, version, len, crc32((const uint8_t *)data, len)};

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_EraseInitTypeDef er = {0};
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Sector = STORE_SECTOR;
    er.NbSectors = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* VDD 2,7–3,6 V: ghi từng word 32 bit */
    uint32_t bad = 0;
    bool ok = HAL_FLASHEx_Erase(&er, &bad) == HAL_OK;

    uint32_t addr = STORE_ADDR;
    const uint32_t *hw = (const uint32_t *)&h;
    for (uint32_t i = 0; ok && i < sizeof(h) / 4U; i++, addr += 4U) {
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, hw[i]) == HAL_OK;
    }
    const uint8_t *p = (const uint8_t *)data;
    for (uint32_t off = 0; ok && off < len; off += 4U, addr += 4U) {
        uint32_t w = 0xFFFFFFFFUL;
        memcpy(&w, p + off, (len - off >= 4U) ? 4U : (len - off));
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok;
}

bool Store_Load(void *data, uint32_t len, uint32_t version)
{
    const StoreHeader *h = (const StoreHeader *)STORE_ADDR;
    const uint8_t *src = (const uint8_t *)(STORE_ADDR + sizeof(StoreHeader));
    if (h->magic != STORE_MAGIC || h->version != version || h->len != len) return false;
    if (crc32(src, len) != h->crc) return false;
    memcpy(data, src, len);
    return true;
}
