#include "headerFiles.h"

#define SPI_CS_OFF GPIO_PinWrite(BOARD_INITPINS_FLASH_CS_GPIO_PORT,BOARD_INITPINS_FLASH_CS_PIN, 0);
#define SPI_CS_ON  GPIO_PinWrite(BOARD_INITPINS_FLASH_CS_GPIO_PORT,BOARD_INITPINS_FLASH_CS_PIN, 1);

void sFLASH_ChipSelect(uint8_t State) {
  if (State == LOW){SPI_CS_OFF;}      
	else SPI_CS_ON;
}

