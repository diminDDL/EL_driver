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
#include "driver.pio.h"

unsigned static const int displayXsize = 320;
unsigned static const int displayYsize = 240;

// pinout:
// Data pins: 1-4
// VCLK - 6
// HS - 7
// VS - 8

unsigned static int dataStartPin = 2;       // uses the next 4 pins for data
unsigned static int controlStartPin = 6;    // uses 3 pins for control

uint8_t frameBuffer[displayYsize][displayXsize];
volatile uint32_t intermediaryFrameBuffer1[displayYsize * displayXsize / 32];
volatile uint32_t intermediaryFrameBuffer2[displayYsize * displayXsize / 32];
volatile bool currentBuffer = true;
volatile uint8_t iteration = 0;

// internal use variables
int dma_chan;
PIO pio;
uint smData;
uint smClock;
const uint ledPin = 25;

// function that allows us to use the BOOTSEL button as user input
bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;
    uint32_t flags = save_and_disable_interrupts();
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    for (volatile int i = 0; i < 1000; ++i);
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    restore_interrupts(flags);
    return button_state;
}

void __no_inline_not_in_flash_func(compute_framebuffer)(bool currBuff) {    // <<<--- this is wrong, it is computing it vertically and not horizontally
    // check each pixel value in frameBuffer and the value of iteration, if the pixel is less than iteration, set it to 0 else set it to 1
    for (int y = 0; y < displayYsize; ++y) {
        for (int x = 0; x < displayXsize; ++x) {
            if (frameBuffer[y][x] <= iteration) {
                if(currBuff){
                    intermediaryFrameBuffer1[y * displayXsize / 32 + x / 32] &= ~(1 << (x % 32));
                }else{
                    intermediaryFrameBuffer2[y * displayXsize / 32 + x / 32] &= ~(1 << (x % 32));
                }
            } else {
                if(currBuff){
                    intermediaryFrameBuffer1[y * displayXsize / 32 + x / 32] |= 1 << (x % 32);
                }else{
                    intermediaryFrameBuffer2[y * displayXsize / 32 + x / 32] |= 1 << (x % 32);
                }
            }
        }
    }
}

void fill_buffer(uint type = 0){
    if(type == 0){
        for(int y = 0; y < displayYsize; y++){
            for(int x = 0; x < displayXsize; x++){
                frameBuffer[y][x] = 0x7F;
            }
        }
    }else if(type == 1){
       for(int y = 0; y < displayYsize; y++){
            for(int x = 0; x < displayXsize; x++){
                // if x is even then set to 0xFF else 0x00
                frameBuffer[y][x] = (x % 2 == 0) ? 0xFF : 0x00;
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

void configureDMA(bool currBuff);

void __no_inline_not_in_flash_func(dma_irq_handler)(){
    // acknowledge the interrupt
    dma_hw->ints0 = (1u << dma_chan);
    currentBuffer = !currentBuffer;
    iteration++;
    configureDMA(currentBuffer);
}

void configureDMA(bool currBuff){
    // make a dma config
    dma_channel_config dma_conf = dma_channel_get_default_config(dma_chan);
    // transfer uint32_t
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
    // an array is read -> increment of read pointer
    channel_config_set_read_increment(&dma_conf, true);
    // a FIFO is written to -> no increment of write pointer
    channel_config_set_write_increment(&dma_conf, false);
    // let writing to the SM determine the speed of DMA
    channel_config_set_dreq(&dma_conf, pio_get_dreq(pio, smData, true));
    // set the source and destination address
    if(currBuff){
        dma_channel_configure(dma_chan, &dma_conf,
                              &pio->txf[smData],                    // Destinatinon pointer
                              &intermediaryFrameBuffer1,            // Source pointer
                              (displayYsize * displayXsize / 32),   // Number of transfers
                              false                                 // Start immediately
        );
    }else{
        dma_channel_configure(dma_chan, &dma_conf,
                              &pio->txf[smData],                    // Destinatinon pointer
                              &intermediaryFrameBuffer2,            // Source pointer
                              (displayYsize * displayXsize / 32),   // Number of transfers
                              false                                 // Start immediately
        );
    }
    // configure DMA interrupts
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_start(dma_chan);
}

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_khz(400000, true);
    //set_sys_clock_khz(100000, true);
    stdio_init_all();

    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);

    static const uint led_pin = 0;
    static const float pio_freq = 80000000;

    pio = pio0;
    smData = pio_claim_unused_sm(pio, true);
    uint offsetData = pio_add_program(pio, &driver_program);
    float divData = (float)clock_get_hz(clk_sys) / pio_freq;
    //float divData = 1;
    driver_program_init(pio, smData, offsetData, dataStartPin, controlStartPin, divData);
    pio_sm_set_enabled(pio, smData, true);
    smClock = pio_claim_unused_sm(pio, true);
    uint offsetClock = pio_add_program(pio, &clock_program);
    float divClock = (float)clock_get_hz(clk_sys) / pio_freq;
    //float divClock = 1;
    clock_program_init(pio, smClock, offsetClock, 8, divClock);
    pio_sm_set_enabled(pio, smClock, true);

    pio_sm_put_blocking(pio, smData, (displayXsize/4));
    pio_sm_put_blocking(pio, smClock, (displayYsize));
    fill_buffer(1);

    // Get a free channel, panic() if there are none
    dma_chan = dma_claim_unused_channel(true);
    compute_framebuffer(currentBuffer);
    compute_framebuffer(!currentBuffer);
    configureDMA(currentBuffer);
    bool oldBufferState = false;
    while(true)
    {
        // if(get_bootsel_button()){
        //     reset_usb_boot(0,0);
        // }
        if(oldBufferState != currentBuffer){
            compute_framebuffer(!currentBuffer);
            oldBufferState = currentBuffer;
        }
    }
    return 0;
}