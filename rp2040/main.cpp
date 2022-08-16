#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/pio.h"
#include "driver.pio.h"

int main()
{
    stdio_init_all();
    
    const uint ledPin = 25;
    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);

    static const uint led_pin = 0;
    // static const float pio_freq = 2000;

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &pulse_program);
    float div = 1;
    pulse_program_init(pio, sm, offset, led_pin, div);
    pio_sm_set_enabled(pio, sm, true);

    sleep_ms(100);
    for(;;)
    {
        gpio_put(ledPin, true);
        sleep_ms(500);
        gpio_put(ledPin, false);
        sleep_ms(500);
    }
    return 0;
}
