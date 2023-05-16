# IO Expander Test

Simple code to test use of four IO expanders (Microchip MCP23017). All expanders share a connection to the latch and clock lines. Set the chip IDs to 0x00, 0x01, 0x02, 0x03 respectively. This will make the actual chip addresses 0x20, 0x21, 0x22, 0x23

Wiring requirements.

* GPIO 04     Data
* GPIO 05     Clock

NOTE: More requirements to come as I hook it up.