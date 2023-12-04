# fume hood fan control

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
- [ ] XL4015 to 3 pin fan
  - [x] yellow tachometer wire GPIO interrupt (no PCNT available)
  - [x] PID: LEDC changes the voltage at the trim pot voltage divider
- [ ] EC11 to adjust PPB setpoint
