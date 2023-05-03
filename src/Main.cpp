
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include <MCP23017.h>

const uint kClockPin = 3;
const uint kData01Pin = 4;
const uint kData02Pin = 5;
const uint kData03Pin = 6;
const uint kData04Pin = 7;

// I2C address
static const uint8_t BASE_ADDRESS = 0x20;
static const uint8_t ADDRESS_FIRST = BASE_ADDRESS;
static const uint8_t ADDRESS_SECOND = BASE_ADDRESS + 1;
static const uint8_t ADDRESS_THIRD = BASE_ADDRESS + 2;
static const uint8_t ADDRESS_FOURTH = BASE_ADDRESS + 3;

// Registers
static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;

// Other constants
static const uint8_t DEVID = 0xE5;
static const float SENSITIVITY_2G = 1.0 / 256; // (g/LSB)
static const float EARTH_GRAVITY = 9.80665;    // Earth's gravity in [m/s^2]

// Ports
i2c_inst_t *i2c = i2c0;

// Buffer to store raw reads
uint8_t data[6];

void Blink() {
  // Simple method to blink the default LED.
  // FIXME: This is completely wasteful.
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(50);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  sleep_ms(50);
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

void Loop2() {
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  float acc_x_f;
  float acc_y_f;
  float acc_z_f;

  // Read X, Y, and Z values from registers (16 bits each)
  reg_read(i2c, BASE_ADDRESS, REG_DATAX0, data, 6);

  // Convert 2 bytes (little-endian) into 16-bit integer (signed)
  acc_x = (int16_t)((data[1] << 8) | data[0]);
  acc_y = (int16_t)((data[3] << 8) | data[2]);
  acc_z = (int16_t)((data[5] << 8) | data[4]);

  // Convert measurements to [m/s^2]
  acc_x_f = acc_x * SENSITIVITY_2G * EARTH_GRAVITY;
  acc_y_f = acc_y * SENSITIVITY_2G * EARTH_GRAVITY;
  acc_z_f = acc_z * SENSITIVITY_2G * EARTH_GRAVITY;

  // Print results
  printf("X: %.2f | Y: %.2f | Z: %.2f\r\n", acc_x_f, acc_y_f, acc_z_f);

  sleep_ms(100);
}

// ***
// *** main
// ***

int main() {

  // Init the USB / UART IO.
  stdio_init_all();

  // Init the uart using our preferred settings.
  stdio_uart_init_full(uart0, 115200, 16, 17);

  // Initialize I2C port at 400 kHz.
  uint actualRate = i2c_init(i2c, 100 * 1000);

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

  // Set PORTA to input mode.
  data[0] = 0x00; // MCP23017::ControlRegister::IODirectionB;
  data[1] = 0x00;
  i2c_write_blocking(i2c, ADDRESS_FIRST, data, 2, false);

  // Set PORTA pullups to ON.
  data[0] = 0x0C;
  data[1] = 0xFF;
  i2c_write_blocking(i2c, ADDRESS_FIRST, data, 2, false);

  // Set PORTB to output mode.
  data[0] = 0x01;
  data[1] = 0xFF;
  i2c_write_blocking(i2c, ADDRESS_FIRST, data, 2, false);

  // Read from PORTA.
  data[0] = 0x12;
  data[1] = 0xAA;
  i2c_write_blocking(i2c, ADDRESS_FIRST, data, 2, true);
  int num_bytes_read = i2c_read_blocking(i2c, ADDRESS_FIRST, data, 1, false);
  printf("Return = %d bytes, value = %u\r\n", num_bytes_read, data[0]);

  // Write to PORTB.
  data[0] = 0x13;
  data[1] = 0x0F;
  i2c_write_blocking(i2c, ADDRESS_FIRST, data, 2, false);

  printf("\n\nMade it through!\n\n");

  // Wait before taking measurements
  sleep_ms(2000);

  // Loop forever
  while (true) {

    // Loop2();
    //    printf("...\n");

    Blink();
  }
}
