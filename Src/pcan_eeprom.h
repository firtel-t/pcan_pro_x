#pragma once
#include <stdint.h>

/* Simple Flash EEPROM emulation for STM32G431
 * Uses last Flash page (2KB at 0x0801F800) to store device settings.
 * Layout: 4 bytes magic + 4 bytes device_nr + 4 bytes channel_nr
 */

#define EEPROM_MAGIC  0x5043414E  /* "PCAN" */

struct pcan_eeprom_data
{
  uint32_t magic;
  uint32_t device_nr;
  uint32_t channel_nr;
};

void pcan_eeprom_init(void);
void pcan_eeprom_read(uint32_t *device_nr, uint32_t *channel_nr);
void pcan_eeprom_write(uint32_t device_nr, uint32_t channel_nr);
