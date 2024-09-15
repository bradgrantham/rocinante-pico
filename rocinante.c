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

    const uint32_t requested_rate = 267000000; // 232000000; // 272000000; // 270000000;
    set_sys_clock_hz(requested_rate, 0);

    int samples, lines;

    switch(sizeof(buffer))
    {
        case 910 * 262:
            samples = 910;
            lines = 262;
            break;
        case 912 * 262:
            samples = 912;
            lines = 262;
            break;
        case 1368 * 262:
            samples = 1368;
            lines = 262;
            break;
        case 910 * 525:
            samples = 910;
            lines = 525;
            break;
        case 912 * 525:
            samples = 912;
            lines = 525;
            break;
        case 1368 * 525:
            samples = 1368;
            lines = 525;
            break;
        default:
            printf("unexpected image size %zd\n", sizeof(buffer));
            panic("data error");
            break;
    }

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("Rocinante on Pico, %ld clock rate\n", clock_get_hz(clk_sys));

    size_t size = samples * lines;
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
    int restart_chan = dma_claim_unused_channel(true);

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
        buffer,            // read address
        size / transfer_size,  // size of frame in transfers
        false           // start 
    );

    const void *buffer_address = buffer;

    dma_channel_configure(
        restart_chan,           // DMA channel
        &restart_config,             // channel_config
        &dma_hw->ch[stream_chan].read_addr,  // write address
        &buffer_address,            // read address
        1,  // size of frame in transfers
        false           // don't start 
    );

    pio_sm_set_enabled(pio, sm, true);
    dma_channel_start(stream_chan);

    printf("streaming...\n");
    sleep_ms(1000 * 1000);
}
