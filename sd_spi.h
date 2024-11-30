#ifndef __SD_SPI_H__
#define __SD_SPI_H__

#include "hardware/spi.h"

#define SD_BLOCK_SIZE 512 // XXX can actually be something different?

int SDCARD_readblock(spi_inst_t *spi, unsigned int blocknum, unsigned char *block);
int SDCARD_writeblock(spi_inst_t *spi, unsigned int blocknum, const unsigned char *block);
int SDCARD_init(spi_inst_t *spi);

#endif /* __SD_SPI_H__ */
