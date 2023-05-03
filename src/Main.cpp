
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "MCP23017.h"

// I2C addresses. Assumption: we have four units on a board. The limit is eight,
// but we don't have a board for that yet.
static const uint8_t kBaseAddress = 0x20;
static const uint8_t kAddressFirst = kBaseAddress;
static const uint8_t kAddressSecond = kBaseAddress + 1;
static const uint8_t kAddressThird = kBaseAddress + 2;
static const uint8_t kAddressFourth = kBaseAddress + 3;

// Ports
i2c_inst_t *i2c = i2c0;

// Buffer to store raw reads
uint8_t data[6];

void Blink() {
  // Simple method to blink the default LED.
  // FIXME: This is completely wasteful.
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(100);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  sleep_ms(100);
}

// Write 1 byte to the specified register
int reg_write(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
              const uint8_t nbytes) {

  int num_bytes_read = 0;
  uint8_t msg[nbytes + 1];

  // Check to make sure caller is sending 1 or more bytes
  if (nbytes < 1) {
    return 0;
  }

  // Append register address to front of data packet
  msg[0] = reg;
  for (int i = 0; i < nbytes; i++) {
    msg[i + 1] = buf[i];
  }

  // Write data to register(s) over I2C
  i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

  return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
             const uint8_t nbytes) {

  int num_bytes_read = 0;

  // Check to make sure caller is asking for 1 or more bytes
  if (nbytes < 1) {
    return 0;
  }

  // Read data from register(s) over I2C
  i2c_write_blocking(i2c, addr, &reg, 1, true);
  num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

  return num_bytes_read;
}

void Loop() {
  // Read from PORTB.
  data[0] = 0x13;
  data[1] = 0xAA; // TEST: random garbage
  i2c_write_blocking(i2c, kAddressFirst, data, 1, true);
  i2c_read_blocking(i2c, kAddressFirst, data, 1, false);
  printf("value = 0x%0X\r\n", data[0]);
  data[1] = data[0]; // Copy the returned result for output.

  // Write the previously read value to PORTA.
  data[0] = 0x12;
  i2c_write_blocking(i2c, kAddressFirst, data, 2, false);
}

void Initialise() {
  // Init the USB / UART IO.
  stdio_init_all();

  // Init the uart using our preferred settings.
  stdio_uart_init_full(uart0, 115200, 16, 17);

  // Initialize I2C port at 400 kHz.
  uint actualRate = i2c_init(i2c, 400 * 1000);

  printf("\n\nIO Expander: Rate = %u\n\n", actualRate);

  // Control the on-board LED so we have a heartbeat to confirm operation.
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  // Initialize I2C pins
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  printf("\n\nPre-write\n\n");

  // Set PORTA to output mode.
  data[0] = 0x00; // MCP23017::ControlRegister::IODirectionB;
  data[1] = 0x00;
  i2c_write_blocking(i2c, kAddressFirst, data, 2, false);

  // Set PORTA pullups to OFF.
  data[0] = 0x0C;
  data[1] = 0x00;
  i2c_write_blocking(i2c, kAddressFirst, data, 2, false);

  // Set PORTB to input mode.
  data[0] = 0x01;
  data[1] = 0xFF;
  i2c_write_blocking(i2c, kAddressFirst, data, 2, false);

  // Set PORTB pullups to ON.
  data[0] = 0x0D;
  data[1] = 0xFF;
  i2c_write_blocking(i2c, kAddressFirst, data, 2, false);

  printf("\n\nInit complete!\n\n");
}

int main() {

  Initialise();

  // Loop forever
  while (true) {
    Loop();
    Blink();
  }
}
