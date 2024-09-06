#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "rocinante.pio.h"

#include "byte_queue.h"
#include "rocinante.h"
#include "text-mode.h"

extern const uint8_t buffer[];

const uint LED_PIN = 25;

const uint AUDIO_PIN = 28;

const uint32_t NTSC_PIN_BASE = 8;
const uint32_t NTSC_PIN_COUNT = 8;

enum {
    CORE1_OPERATION_SUCCEEDED = 1,
    CORE1_ENABLE_VIDEO_ISR,
    CORE1_DISABLE_VIDEO_ISR,
    CORE1_AUDIO_TEST,
};

volatile int core1_line = 0;

void core1_main()
{
    for(;;)
    {
        core1_line = __LINE__;
        uint32_t request = multicore_fifo_pop_blocking();
        core1_line = __LINE__;
        switch(request) {
            case CORE1_ENABLE_VIDEO_ISR :
                core1_line = __LINE__;
                irq_set_enabled(DMA_IRQ_0, true);
                core1_line = __LINE__;
                break;
            case CORE1_DISABLE_VIDEO_ISR :
                core1_line = __LINE__;
                irq_set_enabled(DMA_IRQ_0, false);
                core1_line = __LINE__;
                break;
            case CORE1_AUDIO_TEST : {
                // approximately 440Hz tone test
                core1_line = __LINE__;
                int foo = 0;
                for(;;) {
                    core1_line = __LINE__;
                    pwm_set_gpio_level(AUDIO_PIN, foo ? 0 : 255);
                    foo = !foo;
                    sleep_us(2272);
        }
            }
        }
        core1_line = __LINE__;
        multicore_fifo_push_blocking(CORE1_OPERATION_SUCCEEDED);
        core1_line = __LINE__;
    }
}

// Audio ----------------------------------------------------------------------

#define AUDIO_CHUNK_SIZE (256 * 2)
#define AUDIO_CHUNK_COUNT 4
#define AUDIO_BUFFER_SIZE (AUDIO_CHUNK_SIZE * AUDIO_CHUNK_COUNT)

static uint8_t audioBuffer[AUDIO_BUFFER_SIZE];

volatile size_t audioReadNext = 0;
volatile size_t audioWriteNext = AUDIO_CHUNK_SIZE * AUDIO_CHUNK_COUNT / 2;
volatile size_t missedAudioSamples = 0;

void RoAudioGetSamplingInfo(float *rate, size_t *recommendedChunkSize)
{
    // If NTSC line ISR is providing audio, we will have a sampling rate of 15.6998 KHz
    *rate = 15699.76074561403508;
    *recommendedChunkSize = AUDIO_CHUNK_SIZE;
}

size_t WriteOverlapsRead(size_t audioWriteNext, size_t writeSize, size_t audioReadNext)
{
    size_t testReadPosition = (audioReadNext < audioWriteNext) ? (audioReadNext + AUDIO_BUFFER_SIZE) : audioReadNext;
    // printf("audioReadNext %zd, testReadPosition %zd\n", audioReadNext, testReadPosition);
    if((testReadPosition > audioWriteNext) && (audioWriteNext + writeSize >= testReadPosition)) {
        return audioWriteNext + writeSize - testReadPosition;
    } else {
        return 0;
    }
}

size_t RoAudioEnqueueSamplesBlocking(size_t writeSize /* in bytes */, uint8_t* buffer)
{
#if 0
    static size_t missedPreviously = 0;
    if(missedPreviously != missedAudioSamples) {
        RoDebugOverlayPrintf("Missed %ld\n", missedAudioSamples - missedPreviously);
        missedPreviously = missedAudioSamples;
    }
#endif

    size_t waitSampleCount;

    if(writeSize > AUDIO_BUFFER_SIZE) {
        return SIZE_MAX;
    }

    waitSampleCount = WriteOverlapsRead(audioWriteNext, writeSize, audioReadNext);

    while(WriteOverlapsRead(audioWriteNext, writeSize, audioReadNext) != 0) {
        // printf("Wait... %zd %zd %zd\n", testReadPosition, audioWriteNext, writeSize);
    }

    size_t toCopy = (writeSize < (AUDIO_BUFFER_SIZE - audioWriteNext)) ? writeSize : (AUDIO_BUFFER_SIZE - audioWriteNext);
    memcpy(audioBuffer + audioWriteNext, buffer, toCopy);

    size_t remaining = writeSize - toCopy;
    if(remaining > 0) {
        memcpy(audioBuffer, buffer + toCopy, remaining);
    }

    if(audioReadNext == audioWriteNext) {
        audioReadNext = (audioReadNext + 2) % sizeof(audioBuffer);
    }

    audioWriteNext = (audioWriteNext + writeSize) % AUDIO_BUFFER_SIZE;

    return waitSampleCount;
}

void RoAudioClear()
{
    memset(audioBuffer, 128, sizeof(audioBuffer));
    audioReadNext = 0;
    audioWriteNext = AUDIO_CHUNK_SIZE * AUDIO_CHUNK_COUNT / 2;
}

void AudioStart()
{
    RoAudioClear();

    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);
    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

    pwm_config audio_pwm_config = pwm_get_default_config();
    pwm_config_set_clkdiv(&audio_pwm_config, 8); // 67.17754f); // 270M / (15.7K * 256)

    pwm_config_set_wrap(&audio_pwm_config, 255);
    pwm_init(audio_pin_slice, &audio_pwm_config, true);
}


// Video ----------------------------------------------------------------------

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

uint8_t NTSCColorburst0;
uint8_t NTSCColorburst60;
uint8_t NTSCColorburst90;
uint8_t NTSCColorburst120;
uint8_t NTSCColorburst180;
uint8_t NTSCColorburst240;
uint8_t NTSCColorburst270;
uint8_t NTSCColorburst300;

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

NTSCModeTiming NTSCTiming912, NTSCTiming1368;

void NTSCCalculateLineClocks(NTSCModeTiming* timing, int samples)
{
    timing->line_clocks = samples;
    timing->hsync = floorf(timing->line_clocks * NTSC_HOR_SYNC_DUR + 0.5);
    timing->front_porch = timing->line_clocks * NTSC_FRONTPORCH;
    timing->back_porch = timing->line_clocks * NTSC_BACKPORCH;
    timing->eq_pulse = timing->line_clocks * NTSC_EQ_PULSE_INTERVAL;
    timing->vsync = timing->line_clocks * NTSC_VSYNC_BLANK_INTERVAL;
    timing->colorburst_offset = (samples == 912) ? 76 : 114; // XXX Magic numbers preventing generalization
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

    // These are at intervals of 90 degrees - these values replicate
    // the colorburst at 14.31818MHz
    NTSCColorburst0 = NTSCSyncPorch;
    NTSCColorburst90 = NTSCSyncPorch - .6 * NTSCSyncPorch;
    NTSCColorburst180 = NTSCSyncPorch;
    NTSCColorburst270 = NTSCSyncPorch + .6 * NTSCSyncPorch;

    // These, plus the 0 and 180 degree values above, are intervals of 60 degrees
    // these values replicate the colorburst at 21.477270
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

void print_sample(uint8_t sample)
{
    if(sample <= NTSCSyncTip + 1)
    {
        putchar(' ');
    }
    else if(sample <= NTSCSyncPorch + 1)
    {
        putchar('_');
    }
    else if(sample <= NTSCBlack)
    {
        putchar('.');
    }
    else if(sample <= NTSCWhite)
    {
        putchar(".,-+^`"[6 * (sample - NTSCBlack) / (NTSCWhite - NTSCBlack)]);
    }
    else
    {
        putchar('*');
    }
}

void dump_scanline(int clocks, int chars, const uint8_t *samples)
{
    for(int i = 0; i < chars; i++)
    {
        int where = i * clocks / chars;
        print_sample(samples[where]);
    }
    puts("");
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
    // dump_scanline(timing->line_clocks, 140, NTSCEqSyncPulseLine);
    // dump_scanline(timing->line_clocks, 140, NTSCVSyncLine);
    // dump_scanline(timing->line_clocks, 140, NTSCBlankLineBW);
    // dump_scanline(timing->line_clocks, 140, NTSCBlankLineColor);
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

enum { RO_VIDEO_ROW_SAMPLES_UNINITIALIZED = 0, };

uint8_t NTSCRowDoubleBuffer[2][ROW_SAMPLE_STORAGE_MAX];
volatile int NTSCRowNumber = 0;
volatile int NTSCFrameNumber = 0;
volatile int markHandlerInSamples = 0;
int NTSCModeFuncsValid = 0;
int NTSCModeInterlaced = 1;
void* NTSCModePrivateData;
RoNTSCModeFiniFunc NTSCModeFinalize = NULL;
RoNTSCModeFillRowBufferFunc NTSCModeFillRowBuffer = DefaultFillRowBuffer;
RoNTSCModeInitFunc NTSCModeInit = NULL;
RoNTSCModeNeedsColorburstFunc NTSCModeNeedsColorburst = DefaultNeedsColorburst;
int NTSCRowConfig = RO_VIDEO_ROW_SAMPLES_UNINITIALIZED;
NTSCModeTiming *NTSCCurrentTiming;
size_t NTSCRowSamples = 0;
size_t NTSCAppRowSamples = 0;
size_t NTSCAppRowOffset = 0;
uint8_t NTSCVideoMemory[65536];

typedef struct NTSCScanoutVars
{
    int irq_dma_chan;
    void *next_scanout_buffer;
    PIO pio;
    uint sm;
    uint program_offset;
    int stream_chan;
    int restart_chan;
} NTSCScanoutVars;

NTSCScanoutVars ntsc;

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

void __isr NTSCRowISR()
{
    dma_hw->ints0 = 1u << ntsc.irq_dma_chan;

    if(audioReadNext != audioWriteNext) {
        uint16_t value = (audioBuffer[audioReadNext + 0] + audioBuffer[audioReadNext + 1]) / 2;
        pwm_set_gpio_level(AUDIO_PIN, value);
        audioReadNext = (audioReadNext + 2) % sizeof(audioBuffer);
    } else {
        missedAudioSamples++;
    }

    NTSCRowNumber++;
    if(NTSCRowNumber >= 525) {
        NTSCRowNumber = 0;
        NTSCFrameNumber++;
    }
    ntsc.next_scanout_buffer = NTSCRowDoubleBuffer[(NTSCRowNumber + 1) % 2];
    if(NTSCModeFuncsValid) 
    {
        NTSCFillRowBuffer(NTSCCurrentTiming, NTSCFrameNumber, NTSCRowNumber, ntsc.next_scanout_buffer);
    }
}

void NTSCEnableScanout()
{
    uint32_t freq_needed;

    switch(NTSCRowConfig) 
    {
        case RO_VIDEO_ROW_SAMPLES_912:
            freq_needed = 14318180;
            break;
        case RO_VIDEO_ROW_SAMPLES_1368:
            freq_needed = 21477270;
            break;
        default: case RO_VIDEO_ROW_SAMPLES_UNINITIALIZED:
            printf("ROW_SAMPLES uninitialized!\n");
            for(;;);
            break;
    }
    printf("%s %d\n", __FILE__, __LINE__);

    composite_out_program_init(ntsc.pio, ntsc.sm, ntsc.program_offset, NTSC_PIN_BASE, NTSC_PIN_COUNT, freq_needed);

    // Set up DMA channel from image buffer to FIFO, paced by FIFO empty
    const uint transfer_enum = DMA_SIZE_8;
    const int transfer_size = 1;

    dma_channel_config stream_config = dma_channel_get_default_config(ntsc.stream_chan);
    channel_config_set_transfer_data_size(&stream_config, transfer_enum);
    channel_config_set_read_increment(&stream_config, true);
    channel_config_set_write_increment(&stream_config, false);
    channel_config_set_dreq(&stream_config, pio_get_dreq(ntsc.pio, ntsc.sm, true));
    channel_config_set_high_priority(&stream_config, true);

    dma_channel_config restart_config = dma_channel_get_default_config(ntsc.restart_chan);
    channel_config_set_transfer_data_size(&restart_config, DMA_SIZE_32);
    channel_config_set_read_increment(&restart_config, false);
    channel_config_set_write_increment(&restart_config, false);
    channel_config_set_chain_to(&restart_config, ntsc.stream_chan);
    channel_config_set_chain_to(&stream_config, ntsc.restart_chan);

    dma_channel_configure(
        ntsc.stream_chan,           // DMA channel
        &stream_config,             // channel_config
        &ntsc.pio->txf[ntsc.sm],  // write address
        NTSCRowDoubleBuffer[0],            // read address
        NTSCRowSamples / transfer_size,  // size of frame in transfers
        false           // don't start 
    );
    printf("%s %d\n", __FILE__, __LINE__);

    ntsc.next_scanout_buffer = NTSCRowDoubleBuffer[1];

    dma_channel_configure(
        ntsc.restart_chan,           // DMA channel
        &restart_config,             // channel_config
        &dma_hw->ch[ntsc.stream_chan].read_addr,  // write address
        &ntsc.next_scanout_buffer,            // read address
        1,  // size of frame in transfers
        false           // don't start 
    );
    printf("%s %d\n", __FILE__, __LINE__);

    dma_channel_set_irq0_enabled(ntsc.restart_chan, true);
    printf("%s %d\n", __FILE__, __LINE__);
    irq_set_exclusive_handler(DMA_IRQ_0, NTSCRowISR);

    multicore_fifo_push_blocking(CORE1_ENABLE_VIDEO_ISR);
    uint32_t result = multicore_fifo_pop_blocking();
    if(result != CORE1_OPERATION_SUCCEEDED) {
        printf("core 1 failed ENABLE_VIDEO_ISR: %lu\n", result);
        for(;;);
    }

    pio_sm_set_enabled(ntsc.pio, ntsc.sm, true);
    dma_channel_start(ntsc.stream_chan);
}

void NTSCDisableScanout()
{
    multicore_fifo_push_blocking(CORE1_DISABLE_VIDEO_ISR);
    // uint32_t result = multicore_fifo_pop_blocking();
    uint32_t result = 0;
    bool success = multicore_fifo_pop_timeout_us(100000, &result);
    if(!success)
    {
        printf("multicore fifo pop from core 1 on core 0 timed out\n");
        printf("core1 stored that it got past line %d\n", core1_line);
        // for(;;);
    }
    else if(result != CORE1_OPERATION_SUCCEEDED)
    {
        printf("core 1 failed: %lu\n", result);
        for(;;);
    }

    pio_sm_set_enabled(ntsc.pio, ntsc.sm, false);
    dma_channel_set_irq0_enabled(ntsc.restart_chan, false);

    dma_channel_cleanup(ntsc.restart_chan);
    dma_channel_cleanup(ntsc.stream_chan);
}

void RoNTSCSetMode(int interlaced, RoRowConfig row_config, void* private_data, RoNTSCModeInitFunc initFunc, RoNTSCModeFiniFunc finiFunc, RoNTSCModeFillRowBufferFunc fillBufferFunc, RoNTSCModeNeedsColorburstFunc needsColorBurstFunc)
{
    if((NTSCModeInterlaced == interlaced) &&
        (initFunc == NTSCModeInit) &&
        (finiFunc == NTSCModeFinalize) &&
        (fillBufferFunc == NTSCModeFillRowBuffer) &&
        (needsColorBurstFunc == NTSCModeNeedsColorburst) &&
        (row_config == NTSCRowConfig))
    {
        return;
    }

    NTSCModeFuncsValid = 0;

    if(NTSCModeFinalize != NULL)
    {
        NTSCModeFinalize(NTSCModePrivateData);
    }

    if(NTSCRowConfig != RO_VIDEO_ROW_SAMPLES_UNINITIALIZED)
    {
        printf("%s %d\n", __FILE__, __LINE__);
        NTSCDisableScanout();
    }
    // XXX handle row_config != previous row_config by tearing down NTSC scanout, changing clock, and restarting NTSC scanout
    switch(row_config) 
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
    NTSCModeInit = initFunc;
    NTSCModeFillRowBuffer = fillBufferFunc;
    NTSCModeFinalize = finiFunc;
    NTSCModePrivateData = private_data;
    NTSCModeInterlaced = interlaced;
    printf("%s %d\n", __FILE__, __LINE__);
    initFunc(private_data, NTSCBlack, NTSCWhite);
    NTSCRowNumber = 0;
    NTSCFrameNumber = 0;
    NTSCRowConfig = row_config;
    printf("%s %d\n", __FILE__, __LINE__);
    NTSCEnableScanout();
    NTSCModeFuncsValid = 1;
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

void NTSCInit()
{
    NTSCCalculateParameters();
    for(int i = NTSC_PIN_BASE; i < NTSC_PIN_BASE + NTSC_PIN_COUNT; i++) {
        gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_8MA);
    }

    // Set up PIO program for composite_out
    ntsc.pio = pio0;
    ntsc.sm = pio_claim_unused_sm(ntsc.pio, true);
    ntsc.program_offset = pio_add_program(ntsc.pio, &composite_out_program);
    ntsc.stream_chan = dma_claim_unused_channel(true);
    ntsc.restart_chan = ntsc.irq_dma_chan = dma_claim_unused_channel(true);

}

uint32_t RoGetMillis()
{
    absolute_time_t now = get_absolute_time();
    return to_ms_since_boot (now);
}

void RoDelayMillis(uint32_t millis)
{
    sleep_ms(millis);
}

int RoDoHousekeeping(void)
{
    return 0;
}

const uint32_t JOYSTICK_KEYSELECT_PIN = 16;
const uint32_t JOYSTICK_JOYSELECT_PIN = 17;
const uint32_t JOYSTICK_NORTH_PIN = 18;
const uint32_t JOYSTICK_SOUTH_PIN = 19;
const uint32_t JOYSTICK_WEST_PIN = 20;
const uint32_t JOYSTICK_EAST_PIN = 21;
const uint32_t JOYSTICK_FIRE_PIN = 22;

void InitializeControllerPins()
{
    // Set SELECT pins to high-impedance
    gpio_init(JOYSTICK_KEYSELECT_PIN);
    gpio_set_dir(JOYSTICK_KEYSELECT_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_KEYSELECT_PIN);

    gpio_init(JOYSTICK_JOYSELECT_PIN);
    gpio_set_dir(JOYSTICK_JOYSELECT_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_JOYSELECT_PIN);

    // Set NSWEF pins to input with pull-up
    gpio_init(JOYSTICK_NORTH_PIN);
    gpio_set_dir(JOYSTICK_NORTH_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_NORTH_PIN);

    gpio_init(JOYSTICK_SOUTH_PIN);
    gpio_set_dir(JOYSTICK_SOUTH_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_SOUTH_PIN);

    gpio_init(JOYSTICK_WEST_PIN);
    gpio_set_dir(JOYSTICK_WEST_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_WEST_PIN);

    gpio_init(JOYSTICK_EAST_PIN);
    gpio_set_dir(JOYSTICK_EAST_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_EAST_PIN);

    gpio_init(JOYSTICK_FIRE_PIN);
    gpio_set_dir(JOYSTICK_FIRE_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_FIRE_PIN);

}

uint8_t RoGetJoystickState(RoControllerIndex which)
{
    // Set select_joystick to RESET, which grounds joystick and fire-left switches
    gpio_set_dir(JOYSTICK_JOYSELECT_PIN, GPIO_OUT);
    gpio_put(JOYSTICK_JOYSELECT_PIN, 0);

    sleep_us(10);

    // read joystick and fire-left
    unsigned int joystick_value = 0;
    
    switch(which) {
        case CONTROLLER_1:
            joystick_value = (
                ((gpio_get(JOYSTICK_NORTH_PIN) ? 1 : 0) << 0) | 
                ((gpio_get(JOYSTICK_EAST_PIN) ? 1 : 0) << 1) | 
                ((gpio_get(JOYSTICK_SOUTH_PIN) ? 1 : 0) << 2) | 
                ((gpio_get(JOYSTICK_WEST_PIN) ? 1 : 0) << 3) |
                ((gpio_get(JOYSTICK_FIRE_PIN) ? 1 : 0) << 6)
                ) ^ 0x4F;
            break;

        case CONTROLLER_2:
            joystick_value = 0;
            break;
    }

    // set select_joystick to high-impedance
    gpio_set_dir(JOYSTICK_JOYSELECT_PIN, GPIO_IN);

    // HAL_Delay(1);
    return joystick_value;
}

uint8_t RoGetKeypadState(RoControllerIndex which)
{
    // Set select_joystick to RESET, which grounds joystick and fire-left switches
    gpio_set_dir(JOYSTICK_KEYSELECT_PIN, GPIO_OUT);
    gpio_put(JOYSTICK_KEYSELECT_PIN, 0);

    sleep_us(10);

    // read joystick and fire-left
    unsigned int keypad_value = 0;
    
    switch(which) {
        case CONTROLLER_1:
            keypad_value = (
                ((gpio_get(JOYSTICK_NORTH_PIN) ? 1 : 0) << 0) | 
                ((gpio_get(JOYSTICK_EAST_PIN) ? 1 : 0) << 1) | 
                ((gpio_get(JOYSTICK_SOUTH_PIN) ? 1 : 0) << 2) | 
                ((gpio_get(JOYSTICK_WEST_PIN) ? 1 : 0) << 3) |
                ((gpio_get(JOYSTICK_FIRE_PIN) ? 1 : 0) << 6)
                ) ^ 0x4F;
            break;

        case CONTROLLER_2:
            keypad_value = 0;
            break;
    }

    // set select_joystick to high-impedance
    gpio_set_dir(JOYSTICK_KEYSELECT_PIN, GPIO_IN);

    // HAL_Delay(1);
    return keypad_value;
}

#define BAUD_RATE 115200
#define DATA_BITS 8
#define PARITY    UART_PARITY_NONE
#define STOP_BITS 1

#define UART0_TX  0
#define UART0_RX  1

// Just make press-release events separated by 100ms and let emulator Rosa shims figure out what to do.
// Do I add a separate key queue?
    // maybe need a SystemDrainEvents that gets things in the queue that is called first thing by RoGetEvent

struct queue uart_input_queue;
extern void enqueue_serial_input(uint8_t c);

void uart0_irq_routine(void)
{
    while (uart_is_readable(uart0))
    {
        uint8_t c = uart_getc(uart0);
        enqueue_serial_input(c);
    }
}

void __io_putchar( char c )
{
    uart_putc(uart0, c);
}

void uart_setup()
{
   queue_init(&uart_input_queue, QUEUE_CAPACITY);

   uart_init(uart0, BAUD_RATE);
   gpio_set_function(UART0_TX, GPIO_FUNC_UART);
   gpio_set_function(UART0_RX, GPIO_FUNC_UART);
   uart_set_format(uart0, DATA_BITS, STOP_BITS, PARITY);

   uart_set_hw_flow(uart0, false, false);
   uart_set_fifo_enabled(uart0, false);

   irq_set_exclusive_handler(UART0_IRQ, uart0_irq_routine);
   irq_set_enabled(UART0_IRQ, true);
   uart_set_irq_enables(uart0, true, false);
}

char ColecoKeypadToCharacter(uint8_t value)
{
    switch(value) {
        case 0: return '-';
        case CONTROLLER_KEYPAD_0: return '0';
        case CONTROLLER_KEYPAD_1: return '1';
        case CONTROLLER_KEYPAD_2: return '2';
        case CONTROLLER_KEYPAD_3: return '3';
        case CONTROLLER_KEYPAD_4: return '4';
        case CONTROLLER_KEYPAD_5: return '5';
        case CONTROLLER_KEYPAD_6: return '6';
        case CONTROLLER_KEYPAD_7: return '7';
        case CONTROLLER_KEYPAD_8: return '8';
        case CONTROLLER_KEYPAD_9: return '9';
        case CONTROLLER_KEYPAD_asterisk: return '*';
        case CONTROLLER_KEYPAD_pound: return '#';
        default: return '?';
    }
}

void TestControllers()
{
    while(1) {
        uint8_t joystick_1_state = RoGetJoystickState(CONTROLLER_1);
        uint8_t keypad_1_state = RoGetKeypadState(CONTROLLER_1);
        uint8_t joystick_2_state = RoGetJoystickState(CONTROLLER_2);
        uint8_t keypad_2_state = RoGetKeypadState(CONTROLLER_2);
        printf("joy 1 %c %c %c %c %c %c %c     ",
            (joystick_1_state & CONTROLLER_NORTH_BIT) ? 'N' : '-',
            (joystick_1_state & CONTROLLER_SOUTH_BIT) ? 'S' : '-',
            (joystick_1_state & CONTROLLER_WEST_BIT) ? 'W' : '-',
            (joystick_1_state & CONTROLLER_EAST_BIT) ? 'E' : '-',
            (joystick_1_state & CONTROLLER_FIRE_BIT) ? '1' : '-',
            (keypad_1_state & CONTROLLER_FIRE_BIT) ? '2' : '-',
            ColecoKeypadToCharacter(keypad_1_state & CONTROLLER_KEYPAD_MASK));
        printf("joy 2 %c %c %c %c %c %c %c\n",
            (joystick_2_state & CONTROLLER_NORTH_BIT) ? 'N' : '-',
            (joystick_2_state & CONTROLLER_SOUTH_BIT) ? 'S' : '-',
            (joystick_2_state & CONTROLLER_WEST_BIT) ? 'W' : '-',
            (joystick_2_state & CONTROLLER_EAST_BIT) ? 'E' : '-',
            (joystick_2_state & CONTROLLER_FIRE_BIT) ? '1' : '-',
            (keypad_2_state & CONTROLLER_FIRE_BIT) ? '2' : '-',
            ColecoKeypadToCharacter(keypad_2_state & CONTROLLER_KEYPAD_MASK));
        sleep_ms(100);
    }
}

int launcher_main(int argc, const char **argv);
int coleco_main(int argc, const char **argv);

extern void DoATest();

int main()
{
    bi_decl(bi_program_description("Rocinante on Pico."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    bi_decl(bi_1pin_with_name(AUDIO_PIN, "Mono audio"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 0, "Composite bit 0"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 1, "Composite bit 1"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 2, "Composite bit 2"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 3, "Composite bit 3"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 4, "Composite bit 4"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 5, "Composite bit 5"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 6, "Composite bit 6"));
    bi_decl(bi_1pin_with_name(NTSC_PIN_BASE + 7, "Composite bit 7"));
    bi_decl(bi_1pin_with_name(JOYSTICK_KEYSELECT_PIN, "Controller KEYSELECT (green) pin"));
    bi_decl(bi_1pin_with_name(JOYSTICK_JOYSELECT_PIN, "Controller JOYSELECT (gray) pin"));
    bi_decl(bi_1pin_with_name(JOYSTICK_NORTH_PIN, "Controller NORTH pin"));
    bi_decl(bi_1pin_with_name(JOYSTICK_SOUTH_PIN, "Controller SOUTH pin"));
    bi_decl(bi_1pin_with_name(JOYSTICK_WEST_PIN, "Controller WEST pin"));
    bi_decl(bi_1pin_with_name(JOYSTICK_EAST_PIN, "Controller EAST pin"));
    bi_decl(bi_1pin_with_name(JOYSTICK_FIRE_PIN, "Controller FIRE pin"));

    const uint32_t requested_rate = 270000000; // 250000000; // 133000000;
    set_sys_clock_khz(requested_rate / 1000, 1);

    multicore_launch_core1(core1_main);

    stdio_init_all(); // ? Why do I need this?  Don't do it in STM32 runtime
    uart_setup();
    InitializeControllerPins();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("Rocinante on Pico, %ld clock rate\n", clock_get_hz(clk_sys));

    AudioStart();

    if(0)
    {
        multicore_fifo_push_blocking(CORE1_AUDIO_TEST);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result != CORE1_OPERATION_SUCCEEDED) {
            printf("core 1 failed: %lu\n", result);
            for(;;);
        }
    }

    if(0)
    {
        // approximately 440Hz tone test
        int foo = 0;
        for(;;) {
            pwm_set_gpio_level(AUDIO_PIN, foo ? 0 : 255);
            foo = !foo;
            sleep_us(2272);
        }
    }

    printf("initializing composite\n");
    NTSCInit();

    printf("launching...\n");

    DoATest();

    if(0)
    {
        const char *args[] = {
            "emulator",
            "coleco/COLECO.ROM",
            "smurf.col", // "zaxxon.col",
        };
        coleco_main(sizeof(args) / sizeof(args[0]), args); /* doesn't return */
    }

    if(0)
    {
        int previous_frame = 0;
        absolute_time_t started = get_absolute_time();
        uint64_t started_us = to_us_since_boot (started);

        RoTextMode();
        RoTextModeSetLine(0, 0, 0, "Text Mode");
        RoTextModeSetLine(1, 0, 0, "event test...");

        while(1)
        {
            static int thru = 0;
            sleep_ms(1);
            extern void CheckEvents(void);
            CheckEvents();
            if(thru++ % 1000 == 0) {
                printf("through %d loops\n", thru);
            }
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

    // TestControllers();

    {
        const char *args[] = {
            "launcher",
        };
        launcher_main(sizeof(args) / sizeof(args[0]), args);
    }

}
