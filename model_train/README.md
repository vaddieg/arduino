# Model Train Controller

Arduino Nano project for controlling a model train.

## Requirements

### Libraries
- LiquidCrystal (built-in to Arduino IDE)

### Hardware
- Arduino Nano
- 1602 LCD (4-bit mode, no I2C)
- Motor driver with relay for polarity
- Speaker (active or passive)
- 4 pushbuttons
- 2 LEDs with current-limiting resistors (220Ω)
- TOS xxx IR sensor for 36KHz
- IR LED

## Assembly Notes

1. **LCD 4-bit mode**: Use 4 data pins (D4-D7) + RS + EN
2. **Motor relay**: Connect to H-bridge or dual relay module
3. **Speaker**: Simple PWM output, no external DAC needed
4. **Buttons**: Internal pullup resistors used (no external resistors)
5. **Power**: Ensure separate power for motor (Arduino can't drive motor directly)
