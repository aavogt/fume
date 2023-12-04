# fume hood fan control

The ESP32C3 will decide how fast or how long to run the fan.

## TODO

- AGS02MA
  - [x] reads 32ppb inside and >100ppb when above isopropyl
  - [x] use old i2c.h instead to match espressif SSD1306 driver
  - [ ] redo wiring
- SSD1306 need layout for the following:
   - [x] PPB digits
   - [ ] graph PPB
   - [ ] setpoint digits
   - [ ] RPM digits
- XL4015 to 3 pin fan
  - [x] yellow tachometer wire GPIO interrupt (no PCNT available)
  - [x] PID: LEDC changes the voltage at (which part of?) the trim pot voltage divider
- EC11 to adjust PPB setpoint or threshold
  - [ ] driver
  - [ ] decide what the button does
