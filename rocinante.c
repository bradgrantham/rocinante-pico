#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "rocinante.pio.h"

extern const uint8_t buffer[];

const uint LED_PIN = 25;

const uint32_t NTSC_PIN_BASE = 2;
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

    const uint32_t requested_rate = 250000000; // 133000000;
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

#if 1
    const void* src = buffer;
    size_t size = 912 * 525;
    uint32_t freq_needed = 14318180;
#else
    const void* src = scaled;
    size_t size = 228 * 525;
    uint32_t freq_needed = 3579545;
#endif

    // Set processor clock to 128.863620 and then clock out a value
    // every 9 cycles?  O_o  Probably no better than setting PIO to 14.31818 * 2
    // set timer to 9 cycles somehow?

    // Set up PIO program for composite_out
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio0, true);
    uint offset = pio_add_program(pio0, &composite_out_program);
    composite_out_program_init(pio0, sm, offset, NTSC_PIN_BASE, NTSC_PIN_COUNT, freq_needed);

    // Set up DMA channel from image buffer to FIFO, paced by FIFO empty
    uint transfer_enum = DMA_SIZE_32;
    int transfer_size = 4;

    int chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, transfer_enum);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        chan,           // DMA channel
        &c,             // channel_config
        &pio->txf[sm],  // write address
        src,            // read address
        size / transfer_size,  // size of frame in transfers
        false           // don't start 
    );

    pio_sm_set_enabled(pio, sm, true);

    printf("streaming frames\n");
    const int frames = 100; // 10 * 60;
    absolute_time_t started = get_absolute_time();
    uint64_t started_us = to_us_since_boot (started);
    for(int frame = 0; frame < frames; frame++)
    {
        dma_channel_start(chan);
        dma_channel_wait_for_finish_blocking(chan);
        (void)*(io_ro_32*)XIP_NOCACHE_NOALLOC_BASE;
        gpio_put(LED_PIN, frame & 16);
    }
    absolute_time_t ended = get_absolute_time();
    uint64_t ended_us = to_us_since_boot (ended);
    printf("done streaming frames, %llu ms per frame\n", (ended_us - started_us) / 1000 / frames);
}
