#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include "rocinante.pio.h"

#include "rocinante.h"

extern const uint8_t buffer[];

const uint LED_PIN = 25;

const uint32_t NTSC_PIN_BASE = 2;
const uint32_t NTSC_PIN_COUNT = 8;

#define SECTION_CCMRAM

// BEGIN copied from STM32F Rocinante Core/Src/main

// These should be in tightly coupled memory to reduce contention with RAM during DMA 
#define ROW_SAMPLE_STORAGE_MAX 1368
uint8_t NTSCEqSyncPulseLine[ROW_SAMPLE_STORAGE_MAX];
uint8_t NTSCVSyncLine[ROW_SAMPLE_STORAGE_MAX];
uint8_t NTSCBlankLineBW[ROW_SAMPLE_STORAGE_MAX];
uint8_t NTSCBlankLineColor[ROW_SAMPLE_STORAGE_MAX];

uint8_t NTSCSyncTip;
uint8_t NTSCSyncPorch;
uint8_t NTSCBlack;
uint8_t NTSCWhite;

typedef struct NTSCModeTiming
{
    int line_clocks;
    int eq_pulse;
    int vsync;
    int hsync;
    int front_porch;
    int back_porch;
    int colorburst_offset;
} NTSCModeTiming;

uint8_t NTSCColorburst0;
uint8_t NTSCColorburst90;
uint8_t NTSCColorburst180;
uint8_t NTSCColorburst270;

uint8_t NTSCColorburst60;
uint8_t NTSCColorburst120;
uint8_t NTSCColorburst240;
uint8_t NTSCColorburst300;

NTSCModeTiming NTSCTiming912, NTSCTiming1368;

void NTSCCalculateLineClocks(NTSCModeTiming* timing, int samples)
{
    timing->line_clocks = samples;
    timing->hsync = floorf(timing->line_clocks * NTSC_HOR_SYNC_DUR + 0.5);
    timing->front_porch = timing->line_clocks * NTSC_FRONTPORCH;
    timing->back_porch = timing->line_clocks * NTSC_BACKPORCH;
    timing->eq_pulse = timing->line_clocks * NTSC_EQ_PULSE_INTERVAL;
    timing->vsync = timing->line_clocks * NTSC_VSYNC_BLANK_INTERVAL;
    timing->colorburst_offset = (samples == 912) ? 76 : 114; // Magic numbers preventing generalization
}

void NTSCCalculateParameters()
{
    // Calculate values for a scanline
    NTSCCalculateLineClocks(&NTSCTiming912, 912);
    NTSCCalculateLineClocks(&NTSCTiming1368, 1368);

    NTSCSyncTip = RoDACVoltageToValue(NTSC_SYNC_TIP_VOLTAGE);
    NTSCSyncPorch = RoDACVoltageToValue(NTSC_SYNC_PORCH_VOLTAGE);
    NTSCBlack = RoDACVoltageToValue(NTSC_SYNC_BLACK_VOLTAGE);
    NTSCWhite = RoDACVoltageToValue(NTSC_SYNC_WHITE_VOLTAGE);

    // Calculate the values for the colorburst that we'll repeat
    // The waveform is defined as sine in the FCC broadcast doc, but for
    // composite the voltages are reversed, so the waveform becomes -sine.
    // Scale amplitude of wave added to DC by .6 to match seen from other
    // sources on oscilloscope
    NTSCColorburst0 = NTSCSyncPorch;
    NTSCColorburst90 = NTSCSyncPorch - .6 * NTSCSyncPorch;
    NTSCColorburst180 = NTSCSyncPorch;
    NTSCColorburst270 = NTSCSyncPorch + .6 * NTSCSyncPorch;

    NTSCColorburst60 = NTSCSyncPorch - .866 * .6 * NTSCSyncPorch;
    NTSCColorburst120 = NTSCSyncPorch - .866 * .6 * NTSCSyncPorch;
    NTSCColorburst240 = NTSCSyncPorch + .866 * .6 * NTSCSyncPorch;
    NTSCColorburst300 = NTSCSyncPorch + .866 * .6 * NTSCSyncPorch;
}

void NTSCFillEqPulseLine(NTSCModeTiming *timing, unsigned char *rowBuffer)
{
    for (int col = 0; col < timing->line_clocks; col++) {
        if (col < timing->eq_pulse || (col > timing->line_clocks/2 && col < timing->line_clocks/2 + timing->eq_pulse)) {
            rowBuffer[col] = NTSCSyncTip;
        } else {
            rowBuffer[col] = NTSCSyncPorch;
        }
    }
}

void NTSCFillVSyncLine(NTSCModeTiming *timing, unsigned char *rowBuffer)
{
    for (int col = 0; col < timing->line_clocks; col++) {
        if (col < timing->vsync || (col > timing->line_clocks/2 && col < timing->line_clocks/2 + timing->vsync)) {
            rowBuffer[col] = NTSCSyncTip;
        } else {
            rowBuffer[col] = NTSCSyncPorch;
        }
    }
}

void NTSCAddColorburst(NTSCModeTiming* timing, unsigned char *rowBuffer, int row)
{
    int startOfColorburstClocks = timing->colorburst_offset;

    if(timing->line_clocks == 912) 
    {
        for(int col = startOfColorburstClocks; col < startOfColorburstClocks + NTSC_COLORBURST_CYCLES * 4; col++)
        {
            switch((col - startOfColorburstClocks) % 4) {
                case 0: rowBuffer[col] = NTSCColorburst0; break;
                case 1: rowBuffer[col] = NTSCColorburst90; break;
                case 2: rowBuffer[col] = NTSCColorburst180; break;
                case 3: rowBuffer[col] = NTSCColorburst270; break;
            }
        }
    }
    else
    {
        for(int col = startOfColorburstClocks; col < startOfColorburstClocks + NTSC_COLORBURST_CYCLES * 6; col++)
        {
            switch((col - startOfColorburstClocks) % 6) {
                case 0: rowBuffer[col] = NTSCColorburst0; break;
                case 1: rowBuffer[col] = NTSCColorburst60; break;
                case 2: rowBuffer[col] = NTSCColorburst120; break;
                case 3: rowBuffer[col] = NTSCColorburst180; break;
                case 4: rowBuffer[col] = NTSCColorburst240; break;
                case 5: rowBuffer[col] = NTSCColorburst300; break;
            }
        }
    }
}

void NTSCFillBlankLine(NTSCModeTiming *timing, unsigned char *rowBuffer, int withColorburst)
{
    memset(rowBuffer, NTSCBlack, timing->line_clocks);
    for (int col = 0; col < timing->line_clocks; col++)
    {
        if (col < timing->hsync) {
            rowBuffer[col] = NTSCSyncTip;
        } else if(col < timing->hsync + timing->back_porch) {
            rowBuffer[col] = NTSCSyncPorch;
        } else if(col >= timing->line_clocks - timing->front_porch) {
            rowBuffer[col] = NTSCSyncPorch;
        } else {
            rowBuffer[col] = NTSCBlack;
        }
    }
    if(withColorburst)
    {
        NTSCAddColorburst(timing, rowBuffer, 0);
    }
}

void NTSCGenerateLineBuffers(NTSCModeTiming *timing)
{
    // one line = (1 / 3579545) * (455/2)

    // front porch is (.165) * (1 / 15734) / (1 / 3579545) = 37.53812921062726565701 cycles (37.5)
    //     74 cycles at double clock
    // pixels is (1 - .165) * (1 / 15734) / (1 / 3579545) = 189.96568418711380557696 cycles (190)
    //     280 cycles at double clock

    NTSCFillEqPulseLine(timing, NTSCEqSyncPulseLine);
    NTSCFillVSyncLine(timing, NTSCVSyncLine);
    NTSCFillBlankLine(timing, NTSCBlankLineBW, 0);
    NTSCFillBlankLine(timing, NTSCBlankLineColor, 1);
}

int DefaultNeedsColorburst()
{
    return 1;
}

void DefaultFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    if((rowNumber > 60) && (rowNumber < 60 + 192 * 2)) {
        memset(rowBuffer, (NTSCBlack + NTSCWhite) / 2, maxSamples);
    }
}

uint8_t NTSCRowDoubleBuffer[2][ROW_SAMPLE_STORAGE_MAX];
volatile int NTSCRowNumber = 0;
volatile int NTSCFrameNumber = 0;
volatile int markHandlerInSamples = 0;
int NTSCModeFuncsValid = 0;
int NTSCModeInterlaced = 1;
RoNTSCModeFillRowBufferFunc NTSCModeFillRowBuffer = DefaultFillRowBuffer;
RoNTSCModeInitVideoMemoryFunc NTSCModeInitVideoMemory = NULL;
RoNTSCModeNeedsColorburstFunc NTSCModeNeedsColorburst = DefaultNeedsColorburst;
RoRowConfig NTSCRowConfig = RO_VIDEO_ROW_SAMPLES_912;
NTSCModeTiming *NTSCCurrentTiming;
size_t NTSCRowSamples = 912;
size_t NTSCAppRowSamples = 704;
size_t NTSCAppRowOffset = 164;
uint8_t NTSCVideoMemory[65536];

void RoNTSCSetMode(int interlaced, RoRowConfig row_config, RoNTSCModeInitVideoMemoryFunc initFunc, RoNTSCModeFillRowBufferFunc fillBufferFunc, RoNTSCModeNeedsColorburstFunc needsColorBurstFunc)
{
    if((NTSCModeInterlaced == interlaced) &&
        (initFunc == NTSCModeInitVideoMemory) &&
        (fillBufferFunc == NTSCModeFillRowBuffer) &&
        (needsColorBurstFunc == NTSCModeNeedsColorburst))
    {
        return;
    }

    NTSCModeFuncsValid = 0;
    // XXX handle row_config != previous row_config by tearing down NTSC scanout, changing clock, and restarting NTSC scanout
    NTSCRowConfig = row_config;
    switch(NTSCRowConfig) 
    {
        case RO_VIDEO_ROW_SAMPLES_912:
            NTSCRowSamples = 912;
            NTSCAppRowSamples = 704;
            NTSCAppRowOffset = 164; // ?? Why is this a magic number different from (912 - 704) / 2?  HSYNC?  Porch?  Etc?
            NTSCCurrentTiming = &NTSCTiming912;
            break;
        case RO_VIDEO_ROW_SAMPLES_1368:
            NTSCRowSamples = 1368;
            NTSCAppRowSamples = 1056;
            NTSCAppRowOffset = 246; // ?? Why is this a magic number different from (1368 - 1056) / 2?  HSYNC?  Porch?  Etc?
            NTSCCurrentTiming = &NTSCTiming1368;
            break;
    }
    NTSCGenerateLineBuffers(NTSCCurrentTiming);
    NTSCModeNeedsColorburst = needsColorBurstFunc;
    NTSCModeInitVideoMemory = initFunc;
    NTSCModeFillRowBuffer = fillBufferFunc;
    NTSCModeInterlaced = interlaced;
    initFunc(NTSCVideoMemory, sizeof(NTSCVideoMemory), NTSCBlack, NTSCWhite);
    NTSCRowNumber = 0;
    NTSCFrameNumber = 0;
    NTSCModeFuncsValid = 1;
}

// NTSC interlaced is made up of "odd" and "even" fields.  For NTSC, the first field is
// odd, which means it's #1.

void NTSCFillRowBuffer(NTSCModeTiming *timing, int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    // XXX could optimize these by having one branch be lines < 21

    /*
     * Rows 0 through 8 are equalizing pulse, then vsync, then equalizing pulse
     */

    // if(lineNumber < NTSC_EQPULSE_LINES) {
    if(lineNumber ==  0) {

        // odd field equalizing pulse
        memcpy(rowBuffer, NTSCEqSyncPulseLine, sizeof(NTSCEqSyncPulseLine));

    // } else if(lineNumber - NTSC_EQPULSE_LINES < NTSC_VSYNC_LINES) {
    } else if(lineNumber == NTSC_EQPULSE_LINES) {

        // odd field VSYNC
        memcpy(rowBuffer, NTSCVSyncLine, sizeof(NTSCVSyncLine));

    // } else if(lineNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES) < NTSC_EQPULSE_LINES) {
    } else if(lineNumber == NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES) {

        // odd field equalizing pulse
        memcpy(rowBuffer, NTSCEqSyncPulseLine, sizeof(NTSCEqSyncPulseLine));

    // } else if(lineNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES) < NTSC_VBLANK_LINES) {
    } else if(lineNumber == NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES) {

        /*
         * Rows 9 through 2X are other part of vertical blank
         */

        // odd field vblank
        if(NTSCModeFuncsValid) {
            memcpy(rowBuffer, NTSCModeNeedsColorburst() ? NTSCBlankLineColor : NTSCBlankLineBW, NTSCRowSamples);
        } else {
            memcpy(rowBuffer, NTSCBlankLineBW, NTSCRowSamples);
        }

    } else if(lineNumber >= 263 && lineNumber <= 271) {

        // Handle interlace half line and vertical retrace and sync.
        if(lineNumber <= 264) {
            // lines 263, 264 - last 405 of even field eq pulse then first 405 of eq pulse
            memcpy(rowBuffer, NTSCEqSyncPulseLine + NTSCRowSamples / 2, NTSCRowSamples / 2);
            memcpy(rowBuffer + NTSCRowSamples / 2, NTSCEqSyncPulseLine, NTSCRowSamples / 2);
        } else if(lineNumber == 265) {
            // line 265 - last 405 of even field eq pulse then first 405 of vsync
            memcpy(rowBuffer, NTSCEqSyncPulseLine + NTSCRowSamples / 2, NTSCRowSamples / 2);
            memcpy(rowBuffer + NTSCRowSamples / 2, NTSCVSyncLine, NTSCRowSamples / 2);
        } else if(lineNumber <= 267) {
            // lines 266, 267 - last 405 of even field vsync then first 405 of vsync
            memcpy(rowBuffer, NTSCVSyncLine + NTSCRowSamples / 2, NTSCRowSamples / 2);
            memcpy(rowBuffer + NTSCRowSamples / 2, NTSCVSyncLine, NTSCRowSamples / 2);
        } else if(lineNumber == 268) {
            // even vield lines 268 - last 405 of even field vsync then first 405 of eq pulse
            memcpy(rowBuffer, NTSCVSyncLine + NTSCRowSamples / 2, NTSCRowSamples / 2);
            memcpy(rowBuffer + NTSCRowSamples / 2, NTSCEqSyncPulseLine, NTSCRowSamples / 2);
        } else if(lineNumber <= 270) {
            // lines 269, 270 - last 405 of even field eq pulse then first 405 of eq pulse
            memcpy(rowBuffer, NTSCEqSyncPulseLine + NTSCRowSamples / 2, NTSCRowSamples / 2);
            memcpy(rowBuffer + NTSCRowSamples / 2, NTSCEqSyncPulseLine, NTSCRowSamples / 2);
        } else if(lineNumber == 271) {
            // line 271 - last 405 of even field eq pulse then 405 of SyncPorch
            memcpy(rowBuffer, NTSCEqSyncPulseLine + NTSCRowSamples / 2, NTSCRowSamples / 2);
            memset(rowBuffer + NTSCRowSamples / 2, NTSCSyncPorch, NTSCRowSamples / 2);
        }

    // } else if((lineNumber >= 272) && (lineNumber <= 281)) { // XXX half line at 282
    } else if(lineNumber == 272) { // XXX half line at 282

        /*
         * Rows 272 through 2XX are other part of vertical blank
         */

        // even field vertical safe area
        if(NTSCModeFuncsValid) {
            memcpy(rowBuffer, NTSCModeNeedsColorburst() ? NTSCBlankLineColor : NTSCBlankLineBW, NTSCRowSamples);
        } else {
            memcpy(rowBuffer, NTSCBlankLineBW, NTSCRowSamples);
        }

    } else if(
        ((lineNumber >= NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES + NTSC_VBLANK_LINES) && (lineNumber < 263))
        ||
        (lineNumber >= 282)) {

        if((lineNumber == (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES + NTSC_VBLANK_LINES)) || (lineNumber >= 282))
        {
            if(NTSCModeFuncsValid) {
                memcpy(rowBuffer, NTSCModeNeedsColorburst() ? NTSCBlankLineColor : NTSCBlankLineBW, NTSCRowSamples);
            } else {
                memcpy(rowBuffer, NTSCBlankLineBW, NTSCRowSamples);
            }
        }

        int rowWithinFrame;
        if(NTSCModeInterlaced) {
            rowWithinFrame = (lineNumber % 263) * 2 + lineNumber / 263 - 22;
        } else {
            rowWithinFrame = lineNumber % 263 - 22;
        }
        if(NTSCModeFuncsValid) {
            NTSCModeFillRowBuffer(frameNumber, rowWithinFrame, NTSCAppRowSamples, rowBuffer + NTSCAppRowOffset);
        }

        if((lineNumber == 262) && NTSCModeInterlaced) {
            // interlacing, line 262 - overwrite last 405 samples with first 405 samples of EQ pulse
            memcpy(rowBuffer + NTSCRowSamples / 2, NTSCEqSyncPulseLine, NTSCRowSamples / 2);
        } else if((lineNumber == 282) && NTSCModeInterlaced) {
            // interlacing, special line 282 - write SyncPorch from BackPorch to middle of line after mode's fillRow()
            memset(rowBuffer + timing->hsync + timing->back_porch, NTSCSyncPorch, NTSCRowSamples / 2 - (timing->hsync + timing->back_porch));
        }
    }
}

void RoNTSCWaitFrame()
{
    // NTSC won't actually go lineNumber >= 525...
    int field0_vblank;
    int field1_vblank;
    do {
        field0_vblank = (NTSCRowNumber > 257) && (NTSCRowNumber < 262);
        field1_vblank = (NTSCRowNumber > 520) && (NTSCRowNumber < NTSC_FRAME_LINES);
    } while(!field0_vblank && !field1_vblank); // Wait for VBLANK; should do something smarter
}

// END copied from STM32F Rocinante Core/Src/main

int irq_dma_chan;
const void* buffer_start;

void *next_scanout_buffer;

void __isr dma_handler()
{
    dma_hw->ints0 = 1u << irq_dma_chan;
    NTSCRowNumber++;
    if(NTSCRowNumber >= 525) {
        NTSCRowNumber = 0;
        NTSCFrameNumber++;
    }
    next_scanout_buffer = NTSCRowDoubleBuffer[(NTSCRowNumber + 1) % 2];
    if(NTSCModeFuncsValid) 
    {
        NTSCFillRowBuffer(NTSCCurrentTiming, NTSCFrameNumber, NTSCRowNumber, next_scanout_buffer);
    }
}

void NTSCInit()
{
    NTSCCalculateParameters();
}

int init_video(void* buffer, uint32_t bufferSize, uint8_t blackvalue, uint8_t whitevalue)
{
    return 1;
}

void fillrow(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    // int row = rowNumber - 17;
    // re-interlace
    if(rowNumber % 2 == 0) 
    {
        rowNumber = rowNumber / 2;
    }
    else
    {
        rowNumber = 263 + rowNumber / 2;
    }
    memcpy(rowBuffer, buffer + rowNumber * 912 + 164, 704); // maxSamples);
}

int coleco_main(int argc, const char **argv);

uint32_t RoGetMillis()
{
    absolute_time_t now = get_absolute_time();
    return to_ms_since_boot (now);
}

int RoDoHousekeeping(void)
{
    return 0;
}

uint8_t RoGetJoystickState(RoControllerIndex which)
{
    return 0;
}

uint8_t RoGetKeypadState(RoControllerIndex which)
{
    return 0;
}

size_t RoAudioEnqueueSamplesBlocking(size_t writeSize /* in bytes */, uint8_t* buffer)
{
    return 0;
}

void RoAudioClear()
{
}

#define AUDIO_CHUNK_SIZE (256 * 2)

void RoAudioGetSamplingInfo(float *rate, size_t *recommendedChunkSize)
{
    *rate = 15699.76074561403508;
    *recommendedChunkSize = AUDIO_CHUNK_SIZE;
}

int main()
{
    bi_decl(bi_program_description("Rocinante on Pico."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 0, "Composite bit 0"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 1, "Composite bit 1"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 2, "Composite bit 2"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 3, "Composite bit 3"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 4, "Composite bit 4"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 5, "Composite bit 5"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 6, "Composite bit 6"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 7, "Composite bit 7"));

    const uint32_t requested_rate = 270000000; // 250000000; // 133000000;
    set_sys_clock_khz(requested_rate / 1000, 1);

    stdio_init_all();

    FILE *fp = fopen("coleco/COLECO.ROM", "rb");
    printf("fp = %p\n", fp);
    fclose(fp);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("Rocinante on Pico, %ld clock rate\n", clock_get_hz(clk_sys));

    // size_t size = 912;
    // uint32_t freq_needed = 14318180;
    size_t size = 1368;
    uint32_t freq_needed = 21477270;

    for(int i = NTSC_PIN_BASE; i < NTSC_PIN_BASE + NTSC_PIN_COUNT; i++) {
        gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_8MA);
    }

    // Set up PIO program for composite_out
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio0, true);
    uint offset = pio_add_program(pio0, &composite_out_program);
    composite_out_program_init(pio0, sm, offset, NTSC_PIN_BASE, NTSC_PIN_COUNT, freq_needed);

    // Set up DMA channel from image buffer to FIFO, paced by FIFO empty
    uint transfer_enum = DMA_SIZE_8;
    int transfer_size = 1;

    int stream_chan = dma_claim_unused_channel(true);
    int restart_chan = irq_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config stream_config = dma_channel_get_default_config(stream_chan);
    channel_config_set_transfer_data_size(&stream_config, transfer_enum);
    channel_config_set_read_increment(&stream_config, true);
    channel_config_set_write_increment(&stream_config, false);
    channel_config_set_dreq(&stream_config, pio_get_dreq(pio, sm, true));
    channel_config_set_high_priority(&stream_config, true);

    // configure DMA channel 1 to restart DMA channel 0
    dma_channel_config restart_config = dma_channel_get_default_config(restart_chan);
    channel_config_set_transfer_data_size(&restart_config, DMA_SIZE_32);
    channel_config_set_read_increment(&restart_config, false);
    channel_config_set_write_increment(&restart_config, false);
    channel_config_set_chain_to(&restart_config, stream_chan);
    if(true) channel_config_set_chain_to(&stream_config, restart_chan);

    dma_channel_configure(
        stream_chan,           // DMA channel
        &stream_config,             // channel_config
        &pio->txf[sm],  // write address
        NTSCRowDoubleBuffer[0],            // read address
        size / transfer_size,  // size of frame in transfers
        false           // don't start 
    );

    next_scanout_buffer = NTSCRowDoubleBuffer[1];

    dma_channel_configure(
        restart_chan,           // DMA channel
        &restart_config,             // channel_config
        &dma_hw->ch[stream_chan].read_addr,  // write address
        &next_scanout_buffer,            // read address
        1,  // size of frame in transfers
        false           // don't start 
    );

    dma_channel_set_irq0_enabled(restart_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    pio_sm_set_enabled(pio, sm, true);
    dma_channel_start(stream_chan);

    printf("initializing composite\n");
    NTSCInit();

    printf("launching...\n");

// need an IRQ
    int previous_frame = 0;
    absolute_time_t started = get_absolute_time();
    uint64_t started_us = to_us_since_boot (started);

    const char *args[] = {
        "emulator",
        "--playback-controllers",
        "zaxxon.txt",
        "coleco/COLECO.ROM",
        "zaxxon.col",
    };
    coleco_main(sizeof(args) / sizeof(args[0]), args); /* doesn't return */

    while(1)
    {
        sleep_ms(1);
        if(previous_frame + 30 < NTSCFrameNumber)
        {
            absolute_time_t ended = get_absolute_time();
            uint64_t ended_us = to_us_since_boot (ended);
            uint64_t us_per_frame = (ended_us - started_us) / 30;
            uint64_t ms = us_per_frame / 1000;
            uint64_t frac = us_per_frame - ms * 1000;
            printf("(%llu us) %llu.%03llu ms per frame, expected 33.44\n", us_per_frame, ms, frac);
            started = ended;
            started_us = ended_us;
            previous_frame = NTSCFrameNumber;
        }
        gpio_put(LED_PIN, (NTSCFrameNumber % 30) < 15);
    }
}
