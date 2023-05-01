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

// Pins
const uint kSDAPin = 4;
const uint kSCLPin = 5;

// I2C address
static const uint8_t BASE_ADDRESS = 0x20;

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
  i2c_init(i2c, 400 * 1000);

  printf("\n\nIO Expander Test newt\n\n");

  // Control the on-board LED so we have a heartbeat to confirm operation.
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  // Initialize I2C pins
  gpio_set_function(kSDAPin, GPIO_FUNC_I2C);
  gpio_set_function(kSCLPin, GPIO_FUNC_I2C);

  printf("\n\nWTF\n\n");
  // Read device ID to make sure that we can communicate with the chip.
  reg_read(i2c, BASE_ADDRESS, REG_DEVID, data, 1);
  if (data[0] != DEVID) {
    printf("ERROR: Could not communicate with IO Expander\r\n");
    while (true)
      ;
  }

  printf("DEVID: %i\r\n", data[0]);

  // // Read Power Control register
  // reg_read(i2c, BASE_ADDRESS, REG_POWER_CTL, data, 1);
  // printf("0x%X\r\n", data[0]);

  // // Tell ADXL343 to start taking measurements by setting Measure bit to high
  // data[0] |= (1 << 3);
  // reg_write(i2c, BASE_ADDRESS, REG_POWER_CTL, &data[0], 1);

  // // Test: read Power Control register back to make sure Measure bit was set
  // reg_read(i2c, BASE_ADDRESS, REG_POWER_CTL, data, 1);
  // printf("0x%X\r\n", data[0]);

  // Wait before taking measurements
  sleep_ms(2000);

  // Loop forever
  while (true) {

    // Loop2();

    Blink();
  }
}

// void Loop() {
//   uint8_t data01{0};
//   uint8_t data02{0};
//   uint8_t data03{0};
//   uint8_t data04{0};

//   // Sample the pins.
//   //???
//   sleep_us(1);
//   //???
//   sleep_us(1);

//   for (int i = 0; i < 8; i++) {
//     // Read data.
//     bool q1 = gpio_get(kData01Pin);
//     data01 = data01 << 1;
//     data01 += q1 ? 1 : 0;

//     // Read data.
//     bool q2 = gpio_get(kData02Pin);
//     data02 = data02 << 1;
//     data02 += q2 ? 1 : 0;

//     // Read data.
//     bool q3 = gpio_get(kData03Pin);
//     data03 = data03 << 1;
//     data03 += q3 ? 1 : 0;

//     // Read data.
//     bool q4 = gpio_get(kData04Pin);
//     data04 = data04 << 1;
//     data04 += q4 ? 1 : 0;

//     sleep_us(1);

//     // Tick tock.
//     gpio_put(kClockPin, 1);
//     sleep_us(1);
//     gpio_put(kClockPin, 0);
//     sleep_us(1);
//   }

//   // Get the results in binary.
//   char buffer01[10];
//   char buffer02[10];
//   char buffer03[10];
//   char buffer04[10];
//   itoa(data01, buffer01, 2);
//   itoa(data02, buffer02, 2);
//   itoa(data03, buffer03, 2);
//   itoa(data04, buffer04, 2);

//   printf("01: 0x%0X %s    02: 0x%0X %s    03: 0x%0X %s    04: 0x%0X %s\n",
//          data01, buffer01, data02, buffer02, data03, buffer03, data04,
//          buffer04);
// }

// ***
// *** NOTE: Just hacking the shift register code to get the form laid out. Will
// *** need major changes before this executes.
// ***

// int mainOld(void) {
//   // Init the USB / UART IO.
//   stdio_init_all();

//   // Init the uart using our preferred settings.
//   stdio_uart_init_full(uart0, 115200, 16, 17);

//   printf("\n\nIO Expander Test old\n\n");

//   // Control the on-board LED so we have a heartbeat to confirm operation.
//   gpio_init(PICO_DEFAULT_LED_PIN);
//   gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

//   // Set the pins.
//   gpio_init(kClockPin);
//   gpio_set_dir(kClockPin, GPIO_OUT);

//   gpio_init(kData01Pin);
//   gpio_set_dir(kData01Pin, GPIO_IN);

//   gpio_init(kData02Pin);
//   gpio_set_dir(kData02Pin, GPIO_IN);

//   gpio_init(kData03Pin);
//   gpio_set_dir(kData03Pin, GPIO_IN);

//   gpio_init(kData04Pin);
//   gpio_set_dir(kData04Pin, GPIO_IN);

//   // Set the initial state.
//   gpio_put(kClockPin, 0);
//   sleep_us(1);

//   while (true) {
//     Loop();

//     Blink();
//   }

//   return 0;
// }
