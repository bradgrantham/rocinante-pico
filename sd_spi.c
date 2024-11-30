#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "sd_spi.h"
#include "crc7.h"
#include "rocinante.h"

enum DebugLevels {
    DEBUG_SILENT = 0,
    DEBUG_ERRORS,
    DEBUG_WARNINGS,
    DEBUG_EVENTS,
    DEBUG_DATA,
    DEBUG_ALL,
    DEBUG_INSANE = 99,
};
int gDebugLevel = DEBUG_DATA;

void logprintf(int level, char *fmt, ...)
{
    va_list args;
    static char dummy[512];

    if(level > gDebugLevel)
        return;

    va_start(args, fmt);
    vsprintf(dummy, fmt, args);
    va_end(args);

    char *s = dummy;
    while(*s) {
        putchar(*s++);
    }
}

/*--------------------------------------------------------------------------*/
/* SD card -----------------------------------------------------------------*/

int gSDCardTimeoutMillis = 1000;

// cribbed somewhat from http://elm-chan.org/docs/mmc/mmc_e.html
enum SDCardCommand {
    CMD0 = 0,    // init; go to idle state
    CMD8 = 8,    // send interface condition
    CMD17 = 17,  // read single block
    CMD24 = 24,  // write single block
    CMD55 = 55,  // prefix command for application command
    ACMD41 = 41, // application command to send operating condition
};
const unsigned char gSDCardResponseIDLE = 0x01;
const unsigned char gSDCardResponseSUCCESS = 0x00;
const unsigned char gSDCardResponseDATA_ACCEPTED = 0xE5;
const unsigned char gSDCardToken_17_18_24 = 0xFE;

// response length must include initial R1, so 1 for CMD0
int SDCARD_send_command(spi_inst_t *spi, enum SDCardCommand command, unsigned long parameter, unsigned char *response, int response_length)
{
    static unsigned char command_buffer[6];
    static unsigned char command_buffer_read[6];

    command_buffer[0] = 0xff;
    spi_write_blocking(spi, command_buffer, 1);

    command_buffer[0] = 0x40 | command;
    command_buffer[1] = (parameter >> 24) & 0xff;
    command_buffer[2] = (parameter >> 16) & 0xff;
    command_buffer[3] = (parameter >> 8) & 0xff;
    command_buffer[4] = (parameter >> 0) & 0xff;
    command_buffer[5] = ((crc7_generate_bytes(command_buffer, 5) & 0x7f) << 1) | 0x01;

    logprintf(DEBUG_DATA, "command constructed: %02X %02X %02X %02X %02X %02X\n",
        command_buffer[0], command_buffer[1], command_buffer[2],
        command_buffer[3], command_buffer[4], command_buffer[5]);

    spi_write_read_blocking(spi, command_buffer, command_buffer_read, sizeof(command_buffer));
    logprintf(DEBUG_ALL, "returned in buffer: %02X %02X %02X %02X %02X %02X\n",
        command_buffer_read[0], command_buffer_read[1], command_buffer_read[2],
        command_buffer_read[3], command_buffer_read[4], command_buffer_read[5]);

    int then = RoGetMillis();
    do {
        int now = RoGetMillis();
        if(now - then > gSDCardTimeoutMillis) {
            logprintf(DEBUG_ERRORS, "SDCARD_send_command: timed out waiting on response\n");
            panic("SD timeout");
        }
        response[0] = 0xff;
        spi_read_blocking(spi, 0, response, 1);
        logprintf(DEBUG_ALL, "response 0x%02X\n", response[0]);
    } while(response[0] & 0x80);

    if(response_length > 1) {
        spi_read_blocking(spi, 0, response + 1, response_length - 1);
    }

    return 1;
}

// precondition: SD card CS is high (disabled)
// postcondition: SD card CS is low (enabled)
int SDCARD_init(spi_inst_t *spi)
{
    static unsigned char response[8];
    unsigned long OCR;

    /* CS false, 80 clk pulses (read 10 bytes) */
    static unsigned char buffer[10];
    for(unsigned int u = 0; u < sizeof(buffer); u++)
        buffer[u] = 0xff;
    spi_write_blocking(spi, buffer, sizeof(buffer));

    RoDelayMillis(100);
    /* interface init */
    if(!SDCARD_send_command(spi, CMD0, 0, response, 1))
        return 0;
    if(response[0] != gSDCardResponseIDLE) {
        logprintf(DEBUG_WARNINGS, "SDCARD_init: failed to enter IDLE mode, response was 0x%02X\n", response[0]);
        return 0;
    }
    RoDelayMillis(100);

    /* check voltage */
    if(!SDCARD_send_command(spi, CMD8, 0x000001AA, response, 5))
        return 0;
    if(response[0] != gSDCardResponseIDLE) {
        logprintf(DEBUG_WARNINGS, "SDCARD_init: failed to get OCR, response was 0x%02X\n", response[0]);
        return 0;
    }
    OCR = (((unsigned long)response[1]) << 24) | (((unsigned long)response[2]) << 16) | (((unsigned long)response[3]) << 8) | (((unsigned long)response[4]) << 0);
    logprintf(DEBUG_DATA, "SDCARD_init: OCR response is 0x%08lX\n", OCR);

    // should get CSD, CID, print information about them

    // Ask the card to initialize itself, and wait for it to get out of idle mode.
    int then = RoGetMillis();
    do {
        int now = RoGetMillis();
        if(now - then > gSDCardTimeoutMillis) {
            printf("SDCARD_init: timed out waiting on transition to ACMD41\n");
            return 0;
        }
        /* leading command to the ACMD41 command */
        if(!SDCARD_send_command(spi, CMD55, 0x00000000, response, 1))
            return 0;
        if(response[0] != gSDCardResponseIDLE) {
            logprintf(DEBUG_WARNINGS, "SDCARD_init: not in IDLE mode for CMD55, response was 0x%02X\n", response[0]);
            return 0;
        }
        /* start initialization process, set HCS (high-capacity) */
        if(!SDCARD_send_command(spi, ACMD41, 0x40000000, response, 1))
            return 0;
    } while(response[0] != gSDCardResponseSUCCESS);
    logprintf(DEBUG_ALL, "returned from ACMD41: %02X\n", response[0]);

    // After init, we should be able to set the clock as
    // high as 25MHz.
    spi_set_baudrate(spi, 1000000);

    return 1;
}

void dump_more_spi_bytes(spi_inst_t *spi, const char *why)
{
    static unsigned char response[8];
    spi_read_blocking(spi, 0, response, sizeof(response));
    printf("trailing %s: %02X %02X %02X %02X %02X %02X %02X %02X\n", why,
        response[0], response[1], response[2], response[3],
        response[4], response[5], response[6], response[7]);
}

/* precondition: SDcard CS is low (active) */
int SDCARD_readblock(spi_inst_t *spi, unsigned int blocknum, unsigned char *block)
{
    static unsigned char response[8];

    // Send read block command.
    response[0] = 0xff;
    if(!SDCARD_send_command(spi, CMD17, blocknum, response, 1))
        return 0;
    if(response[0] != gSDCardResponseSUCCESS) {
        logprintf(DEBUG_ERRORS, "SDCARD_readblock: failed to respond with SUCCESS, response was 0x%02X\n", response[0]);
        return 0;
    }

    // Wait for the data token.
    int then = RoGetMillis();
    do {
        int now = RoGetMillis();
        if(now - then > gSDCardTimeoutMillis) {
            logprintf(DEBUG_ERRORS, "SDCARD_readblock: timed out waiting for data token\n");
            return 0;
        }
        spi_read_blocking(spi, 0, response, 1);
        logprintf(DEBUG_ALL, "readblock response 0x%02X\n", response[0]);
    } while(response[0] != gSDCardToken_17_18_24);

    // Read data.
    spi_read_blocking(spi, 0, block, SD_BLOCK_SIZE);

    // Read CRC
    spi_read_blocking(spi, 0, response, 2);
    logprintf(DEBUG_DATA, "CRC is 0x%02X%02X\n", response[0], response[1]);

    unsigned short crc_theirs = response[0] * 256 + response[1];

    // calculate our version of CRC and compare
    unsigned short crc_ours = crc_itu_t(0, block, SD_BLOCK_SIZE);

    if(crc_theirs != crc_ours) {
        logprintf(DEBUG_ERRORS, "CRC mismatch (theirs %04X versus ours %04X, reporting failure)\n", crc_theirs, crc_ours);
        return 0;
    } else {
        logprintf(DEBUG_DATA, "CRC matches\n");
    }

    // Wait for DO to go high. I don't think we need to do this for block reads,
    // but I don't think it'll hurt.
    then = RoGetMillis();
    do {
        int now = RoGetMillis();
        if(now - then > gSDCardTimeoutMillis) {
            logprintf(DEBUG_ERRORS, "SDCARD_readblock: timed out waiting on completion\n");
            return 0;
        }
        spi_read_blocking(spi, 0, response, 1);
        logprintf(DEBUG_ALL, "readblock response 0x%02X\n", response[0]);
    } while(response[0] != 0xFF);

    if(gDebugLevel >= DEBUG_ALL) dump_more_spi_bytes(spi, "read completion");

    return 1;
}

/* precondition: SDcard CS is low (active) */
int SDCARD_writeblock(spi_inst_t *spi, unsigned int blocknum, const unsigned char *block)
{
    int count;
    static unsigned char response[8];

    // Send write block command.
    if(!SDCARD_send_command(spi, CMD24, blocknum, response, 1))
        return 0;
    if(response[0] != gSDCardResponseSUCCESS) {
        logprintf(DEBUG_ERRORS, "SDCARD_writeblock: failed to respond with SUCCESS, response was 0x%02X\n", response[0]);
        return 0;
    }
    // XXX - elm-chan.org says I should be waiting >= 1byte here

    // Data token.
    response[0] = gSDCardToken_17_18_24;
    spi_write_blocking(spi, response, 1);

    // Send data.
    spi_write_blocking(spi, block, SD_BLOCK_SIZE);

    // junk CRC
    response[0] = 0xff;
    response[1] = 0xff;
    spi_write_blocking(spi, response, 2);

    // Get DATA_ACCEPTED response from WRITE
    spi_read_blocking(spi, 0, response, 1);
    logprintf(DEBUG_DATA, "writeblock response 0x%02X\n", response[0]);
    if(response[0] != gSDCardResponseDATA_ACCEPTED) {
        logprintf(DEBUG_ERRORS, "SDCARD_writeblock: failed to respond with DATA_ACCEPTED, response was 0x%02X\n", response[0]);
        return 0;
    }

    // Wait while busy (DO = low).
    int then = RoGetMillis();
    count = 0;
    do {
        int now = RoGetMillis();
        if(now - then > gSDCardTimeoutMillis) {
            logprintf(DEBUG_ERRORS, "SDCARD_writeblock: timed out waiting on completion\n");
            panic("SD timeout");
        }
        spi_read_blocking(spi, 0, response, 1);
        logprintf(DEBUG_ALL, "writeblock completion 0x%02X\n", response[0]);
        count++;
    } while(response[0] != 0xFF);
    logprintf(DEBUG_DATA, "read %d SPI bytes waiting on write to complete.\n", count);

    if(gDebugLevel >= DEBUG_ALL) dump_more_spi_bytes(spi, "write completion");

    return 1;
}
