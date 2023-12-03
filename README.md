# fume hood fan control

## TODO

- AGS02MA
  - [x] reads 32ppb inside and >100ppb when above isopropyl
  - [ ] redo wiring
  - [ ] use i2c.h instead to match espressif SSD1306 driver
- [ ] SSD1306 draw PPB graph, setpoints, or RPM
- [ ] XL4015 to 3 pin fan
  - [ ] yellow tachometer wire GPIO interrupt (no PCNT available)
  - [ ] PID: LEDC changes the voltage at the trim pot voltage divider
- [ ] EC11 to adjust PPB setpoint 
