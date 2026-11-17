# To do

## Before closing up the case

- [ ] **Paddles**: add 2600-style paddle circuitry and software support.
  - Hardware: 3.3 V to DE-9 pin 7; each pot pin (5 and 9) through ~1 k to a
    GPIO with ~10 nF from the GPIO to ground. Pin 5 shares GP16 with the
    keypad select; pin 9 needs a GPIO of its own.
  - Firmware: a paddle mode that discharges the cap, floats the pin with
    pulls disabled, and times the rise. Needs a platform call to return the
    values and per-unit calibration.
- [ ] **Audio**: upgrade the mono audio output (currently 8-bit PWM on GP28).

## Later

- [ ] **USB mass storage**: expose the SD card as a USB drive when fire is
  held at power-on (TinyUSB MSC on top of the `diskio.c` block functions;
  FatFs must not have the card mounted in that mode).
