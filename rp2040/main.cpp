#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/bootrom.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "tusb.h"
#include "driver.pio.h"
#include "lib/font.h"

unsigned static const int displayXsize = 320;
unsigned static const int displayYsize = 240;

unsigned static const int displayXsizeDiv32 = displayXsize / 32;
unsigned static const int displayYsizeDiv32 = displayYsize / 32;

const uint8_t brRollover = 2;

// pinout:
// Data pins: 2-5
// VCLK - 6
// HS - 7
// VS - 8

unsigned static int dataStartPin = 2;       // uses the next 4 pins for data
unsigned static int controlStartPin = 6;    // uses 3 pins for control

uint8_t frameBuffer[displayYsize][displayXsize] = {0};
uint8_t tmpframeBuffer[displayYsize][displayXsize] = {0};
volatile uint32_t intermediaryFrameBuffer1[displayYsize * displayXsizeDiv32] = {0};
volatile uint32_t intermediaryFrameBuffer2[displayYsize * displayXsizeDiv32] = {0};
volatile bool currentBuffer = true;
volatile bool computeComplete = false;
volatile uint8_t iteration = 0;


// internal use variables
int dma_chan1;
int dma_chan2;
dma_channel_config dma_conf1;
dma_channel_config dma_conf2;
PIO pio;
uint smData;
uint smClock;
const uint ledPin = 25;

void __time_critical_func(compute_framebuffer)(bool currBuff) {
    // check each pixel value in frameBuffer and the value of iteration, if the pixel is less than iteration, set it to 0 else set it to 1
    for (int y = 0; y < displayYsize; ++y) {
        for (int x = 0; x < displayXsize; ++x) {
            if (frameBuffer[y][x] <= iteration) {
                if(currBuff){
                    // x >> 5 is the same as x / 32
                    // x & 0b0011111 is the same as x % 32
                    intermediaryFrameBuffer1[y * displayXsizeDiv32 + (x >> 5)] &= ~(1 << (x & 0b0011111));
                }else{
                    intermediaryFrameBuffer2[y * displayXsizeDiv32 + (x >> 5)] &= ~(1 << (x & 0b0011111));
                }
            } else {
                if(currBuff){
                    intermediaryFrameBuffer1[y * displayXsizeDiv32 + (x >> 5)] |= 1 << (x & 0b0011111);
                }else{
                    intermediaryFrameBuffer2[y * displayXsizeDiv32 + (x >> 5)] |= 1 << (x & 0b0011111);
                }
            }
        }
    }
}

void fill_buffer(uint type = 0){
    static uint16_t offset = 0;
    if(type == 1){
        for(int y = 0; y < displayYsize; y++){
            for(int x = 0; x < displayXsize; x++){
                frameBuffer[y][x] = 1;
            }
        }
    }else{
        for(int y = 0; y < displayYsize; y++){
            for(int x = 0; x < displayXsize; x++){
                frameBuffer[y][x] = 0x00;
            }
        }
    }
}

void __time_critical_func(configureDMA)(bool currBuff);

void __time_critical_func(dma_irq_handler)(){
    // acknowledge the interrupt
    dma_hw->ints0 = (1u << dma_chan1) | (1u << dma_chan2);
    if(computeComplete){
        iteration++;
        currentBuffer = !currentBuffer;
        computeComplete = false;
    }
    if(iteration >= brRollover){
        iteration = 0;
    }
    configureDMA(currentBuffer);    
}

void __time_critical_func(configureDMA)(bool currBuff){
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    if(currBuff){
        dma_channel_configure(dma_chan1, &dma_conf1,
                              &pio->txf[smData],                    // Destinatinon pointer
                              &intermediaryFrameBuffer1,            // Source pointer
                              (displayYsize * displayXsizeDiv32),   // Number of transfers
                              false                                 // Start immediately
        );
        dma_channel_set_irq0_enabled(dma_chan1, true);
        dma_channel_start(dma_chan1);
    }else{
        dma_channel_configure(dma_chan2, &dma_conf2,
                              &pio->txf[smData],                    // Destinatinon pointer
                              &intermediaryFrameBuffer2,            // Source pointer
                              (displayYsize * displayXsizeDiv32),   // Number of transfers
                              false                                 // Start immediately
        );
        dma_channel_set_irq0_enabled(dma_chan2, true);
        dma_channel_start(dma_chan2);
    }
}

void __time_critical_func(core1)(){
    bool oldBufferState = false;
    while(1){
        if(oldBufferState != currentBuffer && !computeComplete){
            compute_framebuffer(!currentBuffer);
            oldBufferState = currentBuffer;
            computeComplete = true;
        }
    }
}

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_MAX);
    set_sys_clock_khz(400000, true);
    //set_sys_clock_khz(100000, true);
    stdio_init_all();

    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);

    static const uint led_pin = 0;
    static const float pio_freq = 30000000;

    pio = pio0;
    smData = pio_claim_unused_sm(pio, true);
    uint offsetData = pio_add_program(pio, &driver_program);
    float divData = (float)clock_get_hz(clk_sys) / pio_freq;
    //float divData = 1;
    driver_program_init(pio, smData, offsetData, dataStartPin, controlStartPin, divData);
    
    smClock = pio_claim_unused_sm(pio, true);
    uint offsetClock = pio_add_program(pio, &clock_program);
    float divClock = (float)clock_get_hz(clk_sys) / pio_freq;
    //float divClock = 1;
    clock_program_init(pio, smClock, offsetClock, 8, divClock);
    

    pio_sm_put_blocking(pio, smData, (displayXsize/4));
    pio_sm_set_enabled(pio, smData, true);
    pio_sm_put_blocking(pio, smClock, (displayYsize));
    pio_sm_set_enabled(pio, smClock, true);
    fill_buffer(1);

    // Get a free channel, panic() if there are none
    dma_chan1 = dma_claim_unused_channel(true);
    dma_chan2 = dma_claim_unused_channel(true);

    dma_conf1 = dma_channel_get_default_config(dma_chan1);
    channel_config_set_transfer_data_size(&dma_conf1, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_conf1, true);
    channel_config_set_write_increment(&dma_conf1, false);
    channel_config_set_dreq(&dma_conf1, pio_get_dreq(pio, smData, true));

    dma_conf2 = dma_channel_get_default_config(dma_chan2);
    channel_config_set_transfer_data_size(&dma_conf2, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_conf2, true);
    channel_config_set_write_increment(&dma_conf2, false);
    channel_config_set_dreq(&dma_conf2, pio_get_dreq(pio, smData, true));

    compute_framebuffer(currentBuffer);
    compute_framebuffer(!currentBuffer);

    configureDMA(currentBuffer);

    int dma_chan3 = dma_claim_unused_channel(true);
    dma_channel_config dma_conf_memcpy = dma_channel_get_default_config(dma_chan3);
    channel_config_set_transfer_data_size(&dma_conf_memcpy, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_conf_memcpy, true);
    channel_config_set_write_increment(&dma_conf_memcpy, true);

    gpio_put(ledPin, 1);
    multicore_launch_core1(core1);

    // wait for USB to connect
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    while(true)
    {

        puts("A");
        bool broke = false;
        
        for (uint32_t y = 0; y < displayYsize; y++) {
            for (uint32_t x = 0; x < displayXsize/8; x++){
                char buf[2];
                bool timeout = false;
                uint32_t start = time_us_32();
                while(!timeout && !tud_cdc_available()){
                    if(time_us_32() - start > 100000){
                        timeout = true;
                    }
                }
                if(!timeout){
                    tud_cdc_read(buf, sizeof(buf));
                }else{
                    broke = true;
                    break;
                }
                // we need to read two bytes at a time to avoid overhead
                uint8_t c1 = (uint8_t)buf[0];
                uint8_t c2 = (uint8_t)buf[1];
                // byte structure:
                // 00 00 00 00 - 1 byte
                // each collection of 2 bits is a brightness value
                // 00 = black
                // 01 = 1 - 50% in our case
                // 10 || 11 = 2 - 100% in our case
                uint8_t pixels1[4] = {0};
                uint8_t pixels2[4] = {0};
                for(int j = 0; j < 4; j++){
                    // since this is little endian the encoding will be backwards
                    pixels1[j] = (c1 >> (j*2)) & 0b11;
                    pixels2[j] = (c2 >> (j*2)) & 0b11;
                }
                // fill the frame buffer
                tmpframeBuffer[y][x*8] = pixels1[0];
                tmpframeBuffer[y][x*8+1] = pixels1[1];
                tmpframeBuffer[y][x*8+2] = pixels1[2];
                tmpframeBuffer[y][x*8+3] = pixels1[3];
                tmpframeBuffer[y][x*8+4] = pixels2[0];
                tmpframeBuffer[y][x*8+5] = pixels2[1];
                tmpframeBuffer[y][x*8+6] = pixels2[2];
                tmpframeBuffer[y][x*8+7] = pixels2[3];
            }
            if(broke){
                break;
            }
        }
        gpio_put(ledPin, broke);

        // copy tmpframeBuffer into the real frame buffer (this is needed to avoid tearing)
        if(!broke){
            dma_channel_configure(dma_chan3, &dma_conf_memcpy,
                frameBuffer,            // dst
                tmpframeBuffer,         // src
                sizeof(frameBuffer),    // transfer count
                true                    // start immediately
            );
        }
    }
    return 0;
}