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

extern const uint8_t buffer[];

const uint LED_PIN = 25;

const uint32_t NTSC_PIN_BASE = 8;
const uint32_t NTSC_PIN_COUNT = 8;

volatile int frame_number = 0;
volatile int row_number = 0;
int irq_dma_chan;
const void* buffer_start;

uint8_t rowsamples[2][912];
void *next_scanout_buffer;

void __isr dma_handler()
{
    dma_hw->ints0 = 1u << irq_dma_chan;
    row_number++;
    if(row_number >= 525) {
        row_number = 0;
        frame_number++;
    }
    next_scanout_buffer = rowsamples[(row_number + 1) % 2];
    memcpy(next_scanout_buffer, buffer + row_number * 912, 912);
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

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("Rocinante on Pico, %ld clock rate\n", clock_get_hz(clk_sys));

    uint8_t *scaled = malloc(228 * 525);
    if(scaled == NULL) {
        printf("Couldn't allocate\n");
        while(1);
    }
    for(int row = 0; row < 525; row++)
    {
        for(int col = 0; col < 228; col++)
        {
            const uint8_t *bufferp = buffer + 4 * col + row * 912;
            uint8_t *scaledp = scaled + col + row * 228;
            uint8_t v = (bufferp[0] + bufferp[1] + bufferp[2] + buffer[3]) / 4;
            *scaledp = v;
            if((col % 3 == 0) && (row % 20 == 0))
            {
                uint8_t c = (v > 127) ? 127 : v;
                putchar(" .-+*#"[c * 6 / 128]);
            }
        }
        if(row % 20 == 0)
        {
            puts("");
        }
    }

    size_t size = 912;
    uint32_t freq_needed = 14318180 ; // 14493570 ; // 14794309; // correction for weird timing I see, was // 14318180;

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
    if(true) channel_config_set_chain_to(&stream_config, restart_chan);

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

// need an IRQ
    int previous_frame = 0;
    absolute_time_t started = get_absolute_time();
    uint64_t started_us = to_us_since_boot (started);
    while(1)
    {
        sleep_ms(1);
        if(previous_frame + 30 < frame_number)
        {
            absolute_time_t ended = get_absolute_time();
            uint64_t ended_us = to_us_since_boot (ended);
            uint64_t us_per_frame = (ended_us - started_us) / 30;
            uint64_t ms = us_per_frame / 1000;
            uint64_t frac = us_per_frame - ms * 1000;
            printf("(%llu us) %llu.%03llu ms per frame, expected 33.44\n", us_per_frame, ms, frac);
            started = ended;
            started_us = ended_us;
            previous_frame = frame_number;
        }
        gpio_put(LED_PIN, (frame_number % 30) < 15);
    }
}
