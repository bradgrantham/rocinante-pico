# To do

## Before closing up the case

- [x] **Panel LED**: wire the lid LED (circa-1985 green) to a spare GPIO
  (GP6 or GP7) through 150-330 ohm, cathode to ground, and mirror the
  on-board LED heartbeat on it: initialise the pin next to `LED_PIN` and add
  a second `gpio_put` where `rocinante.c` blinks GP25. Test with a regular
  LED first.
- [ ] **Paddles**: add 2600-style paddle software support.
  - Hardware (done): 3.3 V on DE-9 pin 7; pin 5 through 1 k to GP27 and
    pin 9 through 1 k to GP26, each with 10 nF from the GPIO to ground.
    GP27 doubles as the ColecoVision keypad select.
  - Firmware: discharge the cap, float the pin with pulls disabled, and
    time the rise; `RoGetPaddleValue` returns 0..65535. Calibration is
    private to this project, stored in the last flash sector, with a
    calibration app registered with the launcher.
- [ ] **Audio**: upgrade the mono audio output (currently 8-bit PWM on GP28).
- [ ] **Lid d-pad**: after the audio upgrade, wire the four direction buttons
  and the fire button on the lid. Each button goes from its data line
  (GP18 N, GP19 S, GP20 W, GP21 E, GP22 fire) through a series diode to the
  joystick select line (GP17), anode on the data-line side.
- [ ] **Controller switching**: check the pin 8 transistor (base through
  1 k to GP16, collector to DE-9 pin 8, emitter to ground) and try a
  ColecoVision controller in the ColecoVision emulator.
- [ ] **Joystick tester at boot**: remove the temporary launch from `main()`
  once the lid buttons check out.

## Later

- [ ] **USB mass storage**: expose the SD card as a USB drive when fire is
  held at power-on (TinyUSB MSC on top of the `diskio.c` block functions;
  FatFs must not have the card mounted in that mode).
- [ ] **Original Rocinante**: it needs `RoSetControllerType` and
  `RoGetControllerType` to build against the current Rosa API, defaulting
  to an Atari joystick. Deferred.
