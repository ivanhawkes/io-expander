# IO Expander Test

Simple code to test use of four IO expanders (Microchip MCP23017). All expanders share a connection to the latch and clock lines.
There is a separate data line for each shift register.

Wiring requirements.

* GPIO 02     Clock
* GPIO 03     Data 01   (QH on Shift Register 1)

NOTE: More requirements to come as I hook it up.