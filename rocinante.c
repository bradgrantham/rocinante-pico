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
#include "ntsc-kit.h"
#include "ntsc-kit-platform.h"
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
        switch(request)
        {
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
            case CORE1_AUDIO_TEST :
            {
                // approximately 440Hz tone test
                core1_line = __LINE__;
                int foo = 0;
                for(;;)
                {
                    core1_line = __LINE__;
                    pwm_set_gpio_level(AUDIO_PIN, foo ? 0 : 255);
                    foo = !foo;
                    sleep_us(2272);
                }
            }
            default:
            {
                core1_line = 666666;
                for(;;);
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

#define DAC_VALUE_LIMIT 0xFF
#define MAX_DAC_VOLTAGE 1.18

uint8_t PlatformVoltageToDACValue(float voltage)
{
    if(voltage < 0.0f) {
        return 0x0;
    }
    uint32_t value = (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
    if(value >= DAC_VALUE_LIMIT) {
        return DAC_VALUE_LIMIT;
    }
    return value;
}

#define PLACEMENT_FAST_RAM

volatile bool markHandlerInSamples = 0;

typedef struct NTSCScanoutVars
{
    int irq_dma_chan;
    void *next_scanout_buffer;
    PIO pio;
    uint sm;
    uint program_offset;
    int stream_chan;
    int restart_chan;
    size_t lineSamples;
    int lineNumber;
    int frameNumber;
} NTSCScanoutVars;

NTSCScanoutVars ntsc;
NTSCLineConfig videoLineConfig;
bool videoInterlaced;
uint8_t videoLineBuffers[2][1368];

void __isr NTSCLineISR()
{
    dma_hw->ints0 = 1u << ntsc.irq_dma_chan;

    if(audioReadNext != audioWriteNext) {
        uint16_t value = (audioBuffer[audioReadNext + 0] + audioBuffer[audioReadNext + 1]) / 2;
        pwm_set_gpio_level(AUDIO_PIN, value);
        audioReadNext = (audioReadNext + 2) % sizeof(audioBuffer);
    } else {
        missedAudioSamples++;
    }

    ntsc.lineNumber = ntsc.lineNumber + 1;
    if(videoInterlaced)
    {
        if(ntsc.lineNumber == 525)
        {
            ntsc.lineNumber = 0;
        }
    }
    else
    {
        if(ntsc.lineNumber == 262)
        {
            ntsc.lineNumber = 0;
        }
    }
    if(ntsc.lineNumber == 0) {
        ntsc.frameNumber ++;
    }

    ntsc.next_scanout_buffer = videoLineBuffers[(ntsc.lineNumber + 1) % 2];
    NTSCFillLineBuffer(ntsc.frameNumber, ntsc.lineNumber, ntsc.next_scanout_buffer);
    if(markHandlerInSamples)
    {
        if( (ntsc.lineNumber > 30 && ntsc.lineNumber < 262) ||
            (ntsc.lineNumber > 262+30 && ntsc.lineNumber < 262+262))
        {
            uint8_t blackValue = PlatformVoltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE);
            uint8_t whiteValue = PlatformVoltageToDACValue(NTSC_SYNC_WHITE_VOLTAGE);
            int offset = (ntsc.lineSamples == 1368) ? 240 : 160;
            int count = (ntsc.lineSamples == 1368) ? 1056 : 704;

            memset(videoLineBuffers[ntsc.lineNumber % 2] + offset, (blackValue + whiteValue) / 2, count);
        }
    }
}

int PlatformGetNTSCLineNumber()
{
    return ntsc.lineNumber;
}

void PlatformEnableNTSCScanout(NTSCLineConfig line_config, bool interlaced)
{
    uint32_t dma_freq_needed;
    uint32_t system_freq_needed;

    videoLineConfig = line_config;
    videoInterlaced = interlaced;

    switch(line_config) 
    {
        case NTSC_LINE_SAMPLES_910:
            dma_freq_needed = 14318180;
            system_freq_needed = 267000000;
            ntsc.lineSamples = 910;
            break;
        case NTSC_LINE_SAMPLES_912:
            dma_freq_needed = 14318180;
            system_freq_needed = 267000000;
            ntsc.lineSamples = 912;
            break;
        case NTSC_LINE_SAMPLES_1368:
            dma_freq_needed = 21477270;
            system_freq_needed = 249000000; // XXX would be nice to go higher
            ntsc.lineSamples = 1368;
            break;
        default:
            dma_freq_needed = 0;
            system_freq_needed = 0;
            printf("unexpected line config %d\n", line_config);
            panic("unexpected line config");
            break;
    }

    bool succeeded = set_sys_clock_khz(system_freq_needed / 1000, 0);
    if(!succeeded)
    {
        printf("Failed to set clock to requested rate %ld for video sampling.\n", system_freq_needed);
        printf("Attempting to set fallback clock, color may not work.\n");
        bool succeeded = set_sys_clock_khz(dma_freq_needed * 12, 0);
        if(!succeeded)
        {
            printf("Also failed to set fallback clock.  Will use 262MHz or hang.\n");
            set_sys_clock_khz(262000000, 1);
        }
    }

    ntsc.lineNumber = 0;
    ntsc.frameNumber = 0;

    composite_out_program_init(ntsc.pio, ntsc.sm, ntsc.program_offset, NTSC_PIN_BASE, NTSC_PIN_COUNT, dma_freq_needed);

    for(int i = NTSC_PIN_BASE; i < NTSC_PIN_BASE + NTSC_PIN_COUNT; i++) {
        gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_8MA);
    }

    // Set up DMA channel from image buffer to FIFO, paced by FIFO empty
    const uint transfer_enum = DMA_SIZE_8;
    const int transfer_size = 1; // Why doesn't 4 work for this?

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
        videoLineBuffers[0],            // read address
        ntsc.lineSamples / transfer_size,  // size of frame in transfers
        false           // don't start 
    );

    ntsc.next_scanout_buffer = videoLineBuffers[1];

    dma_channel_configure(
        ntsc.restart_chan,           // DMA channel
        &restart_config,             // channel_config
        &dma_hw->ch[ntsc.stream_chan].read_addr,  // write address
        &ntsc.next_scanout_buffer,            // read address
        1,  // size of frame in transfers
        false           // don't start 
    );

    dma_channel_set_irq0_enabled(ntsc.restart_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, NTSCLineISR);

    multicore_fifo_push_blocking(CORE1_ENABLE_VIDEO_ISR);
    uint32_t result = multicore_fifo_pop_blocking();
    if(result != CORE1_OPERATION_SUCCEEDED) {
        printf("core 1 failed ENABLE_VIDEO_ISR: %lu\n", result);
        for(;;);
    }

    pio_sm_set_enabled(ntsc.pio, ntsc.sm, true);
    dma_channel_start(ntsc.stream_chan);
}

void PlatformDisableNTSCScanout()
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

void InitializeVideo()
{
    NTSCInitialize();

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

// https://stackoverflow.com/a/34571089/211234
// Modified to C 11/25/2024
static const char *BASE64_ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
void base64Encode(const uint8_t* in, size_t size)
{
    int val = 0;
    int valb = -6;
    int outsize = 0;

    for (size_t s = 0; s < size; s++)
    {
        uint8_t c = in[s];
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0) {
            putchar(BASE64_ALPHABET[(val >> valb) & 0x3F]);
            outsize ++;
            valb -= 6;
        }
    }
    if (valb > -6)
    {
        putchar(BASE64_ALPHABET[((val << 8) >> (valb + 8)) & 0x3F]);
        outsize ++;
    }
    while (outsize % 4 != 0)
    {
        putchar('=');
        outsize ++;
    }
}

void display_test_image()
{
    const int width = 448;
    const int height = 240;
    const int comp = 1;
    uint8_t img[20 + width * height * comp];
    uint8_t *p = img + sprintf(img, "%s %d %d 255\n", (comp == 1) ? "P5" : "P6", width, height);
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            if(comp == 1) 
            {
                p[x + y * width] = 128;
            }
            else
            {
                p[(x + y * width) * comp + 0] = 255;
                p[(x + y * width) * comp + 1] = 0;
                p[(x + y * width) * comp + 2] = 0;
            }
        }
    }
    printf("\033]1337;File=width=%dpx;height=%dpx;inline=1:", width, height);
    base64Encode(img, (p - img) + width * height * comp);
    printf("\007\n");
}

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

    const uint32_t requested_rate = 262000000; // 250000000; // 133000000;
    set_sys_clock_hz(requested_rate, 1);

    stdio_init_all();
    uart_setup();

    sleep_us(1500000);
    printf("Rocinante on Pico, %ld clock rate\n", clock_get_hz(clk_sys));

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_main);

    InitializeControllerPins();

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
    InitializeVideo();

    // display_test_image();

    DoATest();

    printf("launching...\n");

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
            if(previous_frame + 30 < ntsc.frameNumber)
            {
                absolute_time_t ended = get_absolute_time();
                uint64_t ended_us = to_us_since_boot (ended);
                uint64_t us_per_frame = (ended_us - started_us) / 30;
                uint64_t ms = us_per_frame / 1000;
                uint64_t frac = us_per_frame - ms * 1000;
                printf("(%llu us) %llu.%03llu ms per frame, expected 33.44\n", us_per_frame, ms, frac);
                started = ended;
                started_us = ended_us;
                previous_frame = ntsc.frameNumber;
            }
            gpio_put(LED_PIN, (ntsc.frameNumber % 30) < 15);
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
