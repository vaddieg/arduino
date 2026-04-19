# Model Train Controller

Arduino Nano project for controlling analog model trains. Demo mode is tuned for my specific track setup, but manual mode is useful with any custom configuration.

## Features
- Precise manual train control using PWM power supply via rails
- Detecting train and its speed
- Kiosk/demo mode
- Simulated station whistle sound

### Libraries
- LiquidCrystal (built-in to Arduino IDE)

### Hardware
- Arduino Nano
- 1602 LCD (4-bit parallel mode)
- P5 5V 2-channel relay
- buzzer
- 2 pushbuttons
- 1 auto-centering switch for up/down
- 3 LEDs with current-limiting resistors (220Ω)
- TOS xxx IR sensor for 36KHz
- IR LED
- 5.5mm DC power jack
- 3x 330 Ohm
- 3x 680 Ohm
- 3x 2N2222
- 1x 470uF 25V
- 1x 100uF 16V
- 3x 1N4001 diodes
- 1x 10k potentiometer
- header pins
- 9-15V DC power supply
- wires

##Supported equipment:
- Analog trains in scale HO, TT or N
- Electromagnetic rail switchers
- LED traffic lights