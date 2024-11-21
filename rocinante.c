#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include "rocinante.pio.h"

#include "frame.c"

const uint LED_PIN = 25;

const uint32_t NTSC_PIN_BASE = 8;
const uint32_t NTSC_PIN_COUNT = 8;

volatile int frame_number = 0;
volatile int row_number = 0;
int irq_dma_chan;
const void* buffer_start;

uint8_t eq_line[1368];
uint8_t vsync_line[1368];
uint8_t blank_line[1368];
#define COLOR_LINE_COUNT 128
uint8_t color_lines[COLOR_LINE_COUNT][1368];

uint8_t rowsamples[2][1368];
size_t samples;
size_t lines;
int interlace;
void *next_scanout_buffer;

void __isr dma_handler()
{
    dma_hw->ints0 = 1u << irq_dma_chan;
    row_number++;
    if(row_number >= lines) {
        row_number = 0;
        frame_number++;
    }
    next_scanout_buffer = rowsamples[(row_number + 1) % 2];
    if(row_number < 3)
    {
        memcpy(next_scanout_buffer, eq_line, samples);
    }
    else if(row_number < 6)
    {
        memcpy(next_scanout_buffer, vsync_line, samples);
    }
    else if(row_number < 9)
    {
        memcpy(next_scanout_buffer, eq_line, samples);
    }
    else if(row_number < 21)
    {
        memcpy(next_scanout_buffer, blank_line, samples);
    }
    else
    {
        memcpy(next_scanout_buffer, color_lines[row_number * COLOR_LINE_COUNT / lines], samples);
    }
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

    const uint32_t requested_rate = 224000000; // 232000000; // 267000000; // 272000000; // 270000000;
    set_sys_clock_hz(requested_rate, 0);

    switch(sizeof(buffer))
    {
        case 910 * 262:
            samples = 910;
            lines = 262;
            interlace = 0;
            break;
        case 912 * 262:
            samples = 912;
            lines = 262;
            interlace = 0;
            break;
        case 1368 * 262:
            samples = 1368;
            lines = 262;
            interlace = 0;
            break;
        case 910 * 525:
            samples = 910;
            lines = 525;
            interlace = 1;
            break;
        case 912 * 525:
            samples = 912;
            lines = 525;
            interlace = 1;
            break;
        case 1368 * 525:
            samples = 1368;
            interlace = 1;
            lines = 525;
            break;
        default:
            printf("unexpected image size %zd\n", sizeof(buffer));
            panic("data error");
            break;
    }

    memcpy(eq_line, buffer + samples * 0, samples);
    memcpy(vsync_line, buffer + samples * 3, samples);
    memcpy(blank_line, buffer + samples * 9, samples);
    for(int i = 0; i < COLOR_LINE_COUNT; i++)
    {
        memcpy(color_lines[i], buffer + i * lines / COLOR_LINE_COUNT * samples, samples);
    }

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("Rocinante on Pico, %ld clock rate\n", clock_get_hz(clk_sys));

    size_t size = samples;
    uint32_t freq_needed = (samples == 1368) ? 21477270 : 14318180;
    printf("frame is %zd bytes at %ld Hz\n", size, freq_needed);

    // Set processor clock to 128.863620 and then clock out a value
    // every 9 cycles?  O_o  Probably no better than setting PIO to 14.31818 * 2
    // set timer to 9 cycles somehow?

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
    channel_config_set_chain_to(&stream_config, restart_chan);

    dma_channel_configure(
        stream_chan,           // DMA channel
        &stream_config,             // channel_config
        &pio->txf[sm],  // write address
        rowsamples[0],            // read address
        size / transfer_size,  // size of frame in transfers
        false           // don't start 
    );

    next_scanout_buffer = rowsamples[1];

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

    printf("streaming...\n");
    sleep_ms(1000 * 1000);
}
