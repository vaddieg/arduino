#include "PortDLCD.h"


#include <inttypes.h>
#include "Arduino.h"

// portd bit
#define RS_PIN 4
#define EN_PIN 5

// Attempt to get rid of high-level arduino functuions and connect LCD to single PORTD


// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).



PortDLCD::PortDLCD() {
//	_rs_pin = 4;
//    _enable_pin = 5;
    
//     _data_pins[0] = 0;
//     _data_pins[1] = 1;
//     _data_pins[2] = 2;
//     _data_pins[3] = 3;
    
//     pinMode(_rs_pin, OUTPUT);
//     pinMode(_enable_pin, OUTPUT);
//     for (int i = 0; i < 4; i++) 
//     	pinMode(i, OUTPUT);

	DDRD |= 0b00111111;
    
    _displayfunction = LCD_4BITMODE | LCD_5x8DOTS;
}


void PortDLCD::wait4_1ms() {
    write4bits(0x03);
    delayMicroseconds(4500); // wait min 4.1ms

}

void PortDLCD::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;
  _currline = 0;

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  delayMicroseconds(50000); 
  // Now we pull both RS and R/W low to begin commands
  //digitalWrite(_rs_pin, LOW);
  PORTD &= ~(1 << RS_PIN);
  //digitalWrite(_enable_pin, LOW);
  PORTD &= ~(1 << EN_PIN);
   
  //put the LCD into 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    wait4_1ms();
    
    // second try
    wait4_1ms();
    
    // third go!
//    write4bits(0x03); 
//    delayMicroseconds(150);
    wait4_1ms();
    
    // finally, set to 4-bit interface
    write4bits(0x02); 
  
  // finally, set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);  

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

}

/********** high level commands, for the user! */
void PortDLCD::clear()
{
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void PortDLCD::home()
{
  command(LCD_RETURNHOME);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void PortDLCD::setCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row >= _numlines ) {
    row = _numlines-1;    // we count rows starting w/0
  }
  
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void PortDLCD::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void PortDLCD::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void PortDLCD::noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void PortDLCD::cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void PortDLCD::noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void PortDLCD::blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void PortDLCD::scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void PortDLCD::scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void PortDLCD::leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void PortDLCD::rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void PortDLCD::autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void PortDLCD::noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void PortDLCD::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}

/*********** mid level commands, for sending data/cmds */

inline void PortDLCD::command(uint8_t value) {
  send(value, LOW);
}

uint8_t PortDLCD::write(uint8_t value) {
  send(value, HIGH);
  return 1; // assume sucess
}

uint8_t PortDLCD::write(const char *str) {
    uint8_t ptr=0;
    while (str[ptr] != '\0' ) {
        write(str[ptr++]);
    }
    return ptr;
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void PortDLCD::send(uint8_t value, uint8_t mode) {
  //digitalWrite(_rs_pin, mode);
	if (mode) {
		PORTD |= (1 << RS_PIN);
	} else {
		PORTD &= ~(1 << RS_PIN);
	}
	
    write4bits(value>>4);
    write4bits(value);

}

void PortDLCD::pulseEnable(void) {
  //digitalWrite(_enable_pin, LOW);
  PORTD &= ~(1 << EN_PIN);
  delayMicroseconds(5);    
  //digitalWrite(_enable_pin, HIGH);
  PORTD |= (1 << EN_PIN);
  delayMicroseconds(5);    // enable pulse must be >450ns
  //digitalWrite(_enable_pin, LOW);
  PORTD &= ~(1 << EN_PIN);
  delayMicroseconds(100);   // commands need > 37us to settle
}

void PortDLCD::write4bits(uint8_t value) {
//   for (int i = 0; i < 4; i++) {
//     
//     digitalWrite(_data_pins[i], (value >> i) & 0x01);
//   }
// 
	PORTD = (PORTD & 0b11110000) | (value & 0b00001111);

  pulseEnable();
}


