#ifndef _FLASH_SPI_
#define _FLASH_SPI_

#define SPI_MASTER              SPI1

#define LOW    	 		0x00  /* Chip Select line low */
#define HIGH    		0x01  /* Chip Select line high */
#define SET    			0x01

/* instruction of flash */

#define sFLASH_CMD_WREN           0x06  /* Write enable instruction */
#define sFLASH_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sFLASH_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction *///////////////////////////////

#define sFLASH_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sFLASH_CMD_SE             0x20  /* Sector Erase instruction */
#define sFLASH_CMD_BE             0xD8  /* Bulk Erase instruction 0x52*/
#define sFLASH_CMD_CE             0xc7  /* Chip Erase instruction 0xC7/0x60 */
#define sFLASH_CMD_PP             0x02  /* Page Program instruction */
#define sFLASH_CMD_RDID           0x9F  /* Read identification */

#define sFLASH_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_SPI_PAGESIZE       0x100

/* High layer functions  */
void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_EraseChip(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
//void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void sFLASH_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
void sFLASH_ChipSelect(uint8_t State);
uint8_t sFLASH_WriteByte(uint8_t byte);
uint8_t sFLASH_ReadByte(void);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
#ifdef __cplusplus
}
#endif

#endif 