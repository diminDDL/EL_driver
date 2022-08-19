#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/bootrom.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
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
volatile uint32_t intermediaryFrameBuffer[displayYsize * displayXsize / 32];
volatile uint8_t iteration = 0;

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

void compute_framebuffer() {
    // check each pixel value in frameBuffer and the value of iteration, if the pixel is less than iteration, set it to 0 else set it to 1
    for (int y = 0; y < displayYsize; ++y) {
        for (int x = 0; x < displayXsize; ++x) {
            if (frameBuffer[y][x] <= iteration) {
                intermediaryFrameBuffer[y * displayXsize / 32 + x / 32] &= ~(1 << (x % 32));
            } else {
                intermediaryFrameBuffer[y * displayXsize / 32 + x / 32] |= 1 << (x % 32);
            }
        }
    }
}

void fill_buffer(uint type = 0){
    if(type == 0){
        for(int y = 0; y < displayYsize; y++){
            for(int x = 0; x < displayXsize; x++){
                frameBuffer[y][x] = 0xFF;
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

int main()
{
    stdio_init_all();

    const uint ledPin = 25;
    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);

    static const uint led_pin = 0;
    // static const float pio_freq = 2000;

    PIO pio = pio0;
    uint smData = pio_claim_unused_sm(pio, true);
    uint offsetData = pio_add_program(pio, &driver_program);
    float divData = 1;
    driver_program_init(pio, smData, offsetData, dataStartPin, controlStartPin, divData);
    pio_sm_set_enabled(pio, smData, true);
    uint smClock = pio_claim_unused_sm(pio, true);
    uint offsetClock = pio_add_program(pio, &clock_program);
    float divClock = 1;
    clock_program_init(pio, smClock, offsetClock, 8, divClock);
    pio_sm_set_enabled(pio, smClock, true);



    sleep_ms(100);
    // printf the frame buffer
    // printf("----------------------------------------------------\n|");
    // for(int y = 0; y < displayYsize; y++){
    //     for(int x = 0; x < displayXsize; x++){
    //         printf("%d, ", frameBuffer[y][x]);
    //     }
    //     printf("|\n|");
    // }
    // printf("----------------------------------------------------\n");
    
    pio_sm_put_blocking(pio, smData, 16);
    pio_sm_put_blocking(pio, smClock, 4);
    while(true)
    {
        if(get_bootsel_button()){
            reset_usb_boot(0,0);
        }
        pio_sm_put_blocking(pio, smData, 0x55555555);
        pio_sm_put_blocking(pio, smData, 0x55555555);
        pio_sm_put_blocking(pio, smData, 0x55555555);
        pio_sm_put_blocking(pio, smData, 0x55555555);
        
    }
    return 0;
}