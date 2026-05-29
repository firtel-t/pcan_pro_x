/*
 * pcan_eeprom.c - Flash EEPROM emulation for STM32G431
 * Stores device_nr and channel_nr in last Flash page.
 */
#if defined(STM32G431xx)
#include <stm32g4xx_hal.h>
#include "pcan_eeprom.h"

#define EEPROM_PAGE_ADDR  0x0801F800u
#define EEPROM_PAGE_NR    63  /* last page of 128KB Flash */

static struct pcan_eeprom_data eeprom_cache;

void pcan_eeprom_init(void)
{
  /* Read stored data from Flash */
  const struct pcan_eeprom_data *p = (const struct pcan_eeprom_data *)EEPROM_PAGE_ADDR;

  if(p->magic == EEPROM_MAGIC)
  {
    eeprom_cache = *p;
  }
  else
  {
    /* No valid data - use defaults */
    eeprom_cache.magic = EEPROM_MAGIC;
    eeprom_cache.device_nr = 0xFFFFFFFF;
    eeprom_cache.channel_nr = 0xFFFFFFFF;
  }
}

void pcan_eeprom_read(uint32_t *device_nr, uint32_t *channel_nr)
{
  if(device_nr) *device_nr = eeprom_cache.device_nr;
  if(channel_nr) *channel_nr = eeprom_cache.channel_nr;
}

void pcan_eeprom_write(uint32_t device_nr, uint32_t channel_nr)
{
  /* Skip if nothing changed */
  if(eeprom_cache.device_nr == device_nr && eeprom_cache.channel_nr == channel_nr)
    return;

  eeprom_cache.device_nr = device_nr;
  eeprom_cache.channel_nr = channel_nr;

  /* Erase page */
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = EEPROM_PAGE_NR;
  erase.NbPages = 1;

  HAL_FLASHEx_Erase(&erase, &page_error);

  /* Write data (64-bit aligned writes required) */
  uint64_t data[2];
  data[0] = ((uint64_t)eeprom_cache.device_nr << 32) | eeprom_cache.magic;
  data[1] = (uint64_t)eeprom_cache.channel_nr;

  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_PAGE_ADDR, data[0]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_PAGE_ADDR + 8, data[1]);

  HAL_FLASH_Lock();
}
#endif /* STM32G431xx */
