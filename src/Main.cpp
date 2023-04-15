#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/time.h"

// ***
// *** NOTE: Just hacking the shift register code to get the form laid out. Will
// *** need major changes before this executes.
// ***

int main(void) {
  const uint kLEDPin = PICO_DEFAULT_LED_PIN;
  const uint kClockPin = 3;
  const uint kData01Pin = 4;
  const uint kData02Pin = 5;
  const uint kData03Pin = 6;
  const uint kData04Pin = 7;

  // Init the USB / UART IO.
  stdio_init_all();
  adc_init();

  // Init the uart using our preferred settings.
  stdio_uart_init_full(uart0, 115200, 16, 17);

  printf("\n\nIO Expander Test\n\n");

  // Control the on-board LED so we have a heartbeat to confirm operation.
  gpio_init(kLEDPin);
  gpio_set_dir(kLEDPin, GPIO_OUT);

  // Set the pins.
  gpio_init(kClockPin);
  gpio_set_dir(kClockPin, GPIO_OUT);

  gpio_init(kData01Pin);
  gpio_set_dir(kData01Pin, GPIO_IN);

  gpio_init(kData02Pin);
  gpio_set_dir(kData02Pin, GPIO_IN);

  gpio_init(kData03Pin);
  gpio_set_dir(kData03Pin, GPIO_IN);

  gpio_init(kData04Pin);
  gpio_set_dir(kData04Pin, GPIO_IN);

  // Set the initial state.
  gpio_put(kClockPin, 0);
  sleep_us(1);

  while (true) {
    uint8_t data01{0};
    uint8_t data02{0};
    uint8_t data03{0};
    uint8_t data04{0};

    // Sample the pins.
    //???
    sleep_us(1);
    //???
    sleep_us(1);

    for (int i = 0; i < 8; i++) {
      // Read data.
      bool q1 = gpio_get(kData01Pin);
      data01 = data01 << 1;
      data01 += q1 ? 1 : 0;

      // Read data.
      bool q2 = gpio_get(kData02Pin);
      data02 = data02 << 1;
      data02 += q2 ? 1 : 0;

      // Read data.
      bool q3 = gpio_get(kData03Pin);
      data03 = data03 << 1;
      data03 += q3 ? 1 : 0;

      // Read data.
      bool q4 = gpio_get(kData04Pin);
      data04 = data04 << 1;
      data04 += q4 ? 1 : 0;

      sleep_us(1);

      // Tick tock.
      gpio_put(kClockPin, 1);
      sleep_us(1);
      gpio_put(kClockPin, 0);
      sleep_us(1);
    }

    // Get the results in binary.
    char buffer01[10];
    char buffer02[10];
    char buffer03[10];
    char buffer04[10];
    itoa(data01, buffer01, 2);
    itoa(data02, buffer02, 2);
    itoa(data03, buffer03, 2);
    itoa(data04, buffer04, 2);

    printf("01: 0x%0X %s    02: 0x%0X %s    03: 0x%0X %s    04: 0x%0X %s\n",
           data01, buffer01, data02, buffer02, data03, buffer03, data04,
           buffer04);

    // Slow it all down and give me a chance to read the output.
    gpio_put(kLEDPin, 1);
    sleep_ms(100);
    gpio_put(kLEDPin, 0);
    sleep_ms(100);
  }

  return 0;
}
