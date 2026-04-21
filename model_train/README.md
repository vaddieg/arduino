# Model Train Controller

Arduino Nano project for controlling analog model trains. Nearly all components are included in basic Arduino kits.
Demo mode is tuned for my specific track setup, but manual mode is useful with any custom configuration.

## Features
- Precise manual train control using PWM power supply via rails
- Detecting train and its speed
- Detecting rail current and short circuit protection
- Kiosk/demo mode
- Simulated station whistle sound
- 9-16V DC power supply

## Hardware
### Main board:
- Arduino Nano
- 1602 LCD (4-bit parallel mode)
- P5 5V 2-channel relay
- 5V or 3V buzzer
- 2 pushbuttons
- 1 auto-centering switch for up/down OR a 10kΩ sliding potentiometer
- 3 LEDs with current-limiting resistors (220Ω)
- 1x 0.5Ω 1W SMD 
- 3x 330Ω
- 3x 680Ω
- 3x 2N2222
- 1x 470uF 25V
- 1x 100uF 16V
- 1x 5nF
- 3x 1N4001 diodes
- 1x 10k potentiometer
- header pins
- 1x 2.54mm 2 pin screw terminal connector
- 5.5mm DC power jack
- wires

### Sensor board:
- TOS xxx IR sensor for 36kHz or 38kHz
- IR LED
- 15kΩ

## Supported equipment:
- Analog trains in scale HO, TT or N
- Electromagnetic rail switchers
- LED traffic lights