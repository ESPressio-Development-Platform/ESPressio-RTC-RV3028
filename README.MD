# ESPressio-RTC-RV3028

Platform-independent Micro Crystal RV-3028-C7 implementation for `ESPressio-RTC`.

Implemented facilities include calendar read/write for 2000-2099, power-on-reset validity reporting/clearing, the hardware 32-bit Unix-time counter with bounded repeated-read consistency checking, EEPROM-busy observation, runtime RAM-mirror CLKOUT configuration, and fixed-frequency ISR-driven `ESPressio-Platform::Clock` aliases.

`ConfigureClockOutput()` intentionally changes only the active RAM mirror at register `0x35`; it does not silently perform EEPROM writes or password operations.

## CLKOUT and backup operation

Fixed-rate Platform Clock aliases are provided for 1 Hz, 32 Hz, 64 Hz, 1.024 kHz, 8.192 kHz and 32.768 kHz. `TimerInterrupt` is intentionally not exposed as a static Platform Clock source. The physical CLKOUT must be configured to match the selected alias. Applications must also account for the RV-3028 power/backup configuration: a clock output that is not physically driven during a supply state cannot provide monotonic ticks during that interval.

Clock-output configuration is changed in the active RAM mirror only; this implementation does not silently burn EEPROM configuration cells.
