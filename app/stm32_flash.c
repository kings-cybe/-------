#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"

#define FLASH_BASE_ADDRESS  0x08000000

typedef struct
{
    uint32_t sector;
    uint32_t size;
} sector_desc_t;

static const sector_desc_t sector_descs[] =
 {
    {FLASH_Sector_0, 16 * 1024},  // Sector 0: 16 KB
    {FLASH_Sector_1, 16 * 1024},  // Sector 1: 16 KB
    {FLASH_Sector_2, 16 * 1024},  // Sector 2: 16 KB
    {FLASH_Sector_3, 16 * 1024},  // Sector 3: 16 KB
    {FLASH_Sector_4, 64 * 1024},  // Sector 4: 64 KB
    {FLASH_Sector_5, 128 * 1024}, // Sector 5: 128 KB
    {FLASH_Sector_6, 128 * 1024}, // Sector 6: 128 KB
    {FLASH_Sector_7, 128 * 1024}, // Sector 7: 128 KB
    {FLASH_Sector_8, 128 * 1024}, // Sector 8: 128 KB
    {FLASH_Sector_9, 128 * 1024}, // Sector 9: 128 KB
    {FLASH_Sector_10, 128 * 1024},// Sector10: 128 KB
    {FLASH_Sector_11, 128 * 1024},// Sector11: 128 KB
};

void stm32_flash_lock(void)
{
    FLASH_Lock();
}

void stm32_flash_unlock(void)
{
    FLASH_Unlock();
}

void stm32_flash_erase(uint32_t address, uint32_t size)
{
    uint32_t addr = FLASH_BASE_ADDRESS;

    for (uint32_t i = 0; i < sizeof(sector_descs) / sizeof(sector_desc_t); i++)
    {
        if (addr >= address && addr < address + size)
        {
            printf("erasing sector %lu at address 0x%08lX size %lu\n", i, addr, sector_descs[i].size);
            if (FLASH_EraseSector(sector_descs[i].sector, VoltageRange_3) != FLASH_COMPLETE)
            {
                printf("flash erase error\n");
            }
        }

        addr += sector_descs[i].size;
    }
}

void stm32_flash_program(uint32_t address, const uint8_t *data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i += 4)
    {
        if (FLASH_ProgramWord(address + i, *(uint32_t *)(data + i)) != FLASH_COMPLETE)
        {
            printf("flash program error at address 0x%08lX\n", address + i);
        }
    }
}
