#pragma once

#include <cstdint>

namespace MCP23017 {

/**
 * Control register values when using bank 0 mode.
 */

enum class ControlRegister : uint8_t {
  IODirectionA = 0x00,       // I/O direction register.
  IODirectionB = 0x01,       // I/O direction register.
  IPolarityA = 0x02,         // Input polarity register.
  IPolarityB = 0x03,         // Input polarity register.
  InterruptOnChangeA = 0x04, // Interrupt-on-change control register.
  InterruptOnChangeB = 0x05, // Interrupt-on-change control register.
  DefaultCompareA = 0x06,   // Default compare register for interrupt-on-change.
  DefaultCompareB = 0x07,   // Default compare register for interrupt-on-change.
  InterruptControlA = 0x08, // Interrupt control register.
  InterruptControlB = 0x09, // Interrupt control register.
  IOConfigurationA = 0x0A,  // Configuration register.
  IOConfigurationB = 0x0B,  // Configuration register.
  PullUpResistorA = 0x0C,   // Pull-up resistor configuration register.
  PullUpResistorB = 0x0D,   // Pull-up resistor configuration register.
  InterruptFlagA = 0x0E,    // Interrupt flag register.
  InterruptFlagB = 0x0F,    // Interrupt flag register.
  InterruptCapturedA = 0x10, // Interrupt captured register.
  InterruptCapturedB = 0x11, // Interrupt captured register.
  GPIOA = 0x12,              // General purpose I/O port register.
  GPIOB = 0x13,              // General purpose I/O port register.
  OutputLatchA = 0x14,       // Output latch register.
  OutputLatchB = 0x15        // Output latch register.
};

/**
 * Control register values when using bank 1 mode.
 */

enum class ControlRegisterBank1 : uint8_t {
  IODirectionA = 0x00,       // I/O direction register.
  IODirectionB = 0x10,       // I/O direction register.
  IPolarityA = 0x01,         // Input polarity register.
  IPolarityB = 0x11,         // Input polarity register.
  InterruptOnChangeA = 0x02, // Interrupt-on-change control register.
  InterruptOnChangeB = 0x12, // Interrupt-on-change control register.
  DefaultCompareA = 0x03,   // Default compare register for interrupt-on-change.
  DefaultCompareB = 0x13,   // Default compare register for interrupt-on-change.
  InterruptControlA = 0x04, // Interrupt control register.
  InterruptControlB = 0x14, // Interrupt control register.
  IOConfigurationA = 0x05,  // Configuration register.
  IOConfigurationB = 0x15,  // Configuration register.
  PullUpResistorA = 0x06,   // Pull-up resistor configuration register.
  PullUpResistorB = 0x16,   // Pull-up resistor configuration register.
  InterruptFlagA = 0x07,    // Interrupt flag register.
  InterruptFlagB = 0x17,    // Interrupt flag register.
  InterruptCapturedA = 0x08, // Interrupt captured register.
  InterruptCapturedB = 0x18, // Interrupt captured register.
  GPIOA = 0x09,              // General purpose I/O port register.
  GPIOB = 0x19,              // General purpose I/O port register.
  OutputLatchA = 0x0A,       // Output latch register.
  OutputLatchB = 0x1A        // Output latch register.
};

} // namespace MCP23017