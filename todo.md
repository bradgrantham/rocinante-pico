# To do

## Before closing up the case

- [x] **Panel LED**: wire the lid LED (circa-1985 green) to a spare GPIO
  (GP6 or GP7) through 150-330 ohm, cathode to ground, and mirror the
  on-board LED heartbeat on it: initialise the pin next to `LED_PIN` and add
  a second `gpio_put` where `rocinante.c` blinks GP25. Test with a regular
  LED first.
- [ ] **Paddles**: add 2600-style paddle circuitry and software support.
  - Hardware: 3.3 V to DE-9 pin 7; each pot pin (5 and 9) through ~1 k to a
    GPIO with ~10 nF from the GPIO to ground. Pin 5 shares GP16 with the
    keypad select; pin 9 needs a GPIO of its own.
  - Firmware: a paddle mode that discharges the cap, floats the pin with
    pulls disabled, and times the rise. Needs a platform call to return the
    values and per-unit calibration.
- [ ] **Audio**: upgrade the mono audio output (currently 8-bit PWM on GP28).
- [ ] **Lid d-pad**: after the audio upgrade, wire the four direction buttons
  and the fire button on the lid. Each button goes from its data line
  (GP18 N, GP19 S, GP20 W, GP21 E, GP22 fire) through a series diode to the
  joystick select line (GP17), anode on the data-line side.

## Later

- [ ] **USB mass storage**: expose the SD card as a USB drive when fire is
  held at power-on (TinyUSB MSC on top of the `diskio.c` block functions;
  FatFs must not have the card mounted in that mode).
