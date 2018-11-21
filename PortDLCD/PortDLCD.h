#ifndef PORTDLCD_h
#define PORTDLCD_h

//  Connection to LCD:
//  avr	a-ino  lcd
//  PD0 (0) -> DB4
//  PD1 (1) -> DB5
//  PD2 (2) -> DB6
//  PD3 (3) -> DB7
//  PD4 (4) -> RS
//  PD5 (5) -> enable


// Shrinked version of LCD lib
// by vaddieg
//
// Only 4-bit write-only mode

#include <inttypes.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

class PortDLCD {
public:
//	PortDLCD(int a);
 // PortDLCD(uint8_t rs, uint8_t enable,
//		uint8_t db4, uint8_t db5, uint8_t db6, uint8_t db7); // LCD's pins DB4-7 used for 4-bit transfer
  PortDLCD();
    
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);

  void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void autoscroll();
  void noAutoscroll();

  void createChar(uint8_t, uint8_t[]);
  void setCursor(uint8_t, uint8_t); 
  void command(uint8_t);
  
  uint8_t write(uint8_t value);
  uint8_t write(const char *str);
private:
  void wait4_1ms();
  void send(uint8_t, uint8_t);
  void write4bits(uint8_t);
//  void write8bits(uint8_t);
  void pulseEnable();

//  uint8_t _rs_pin; // LOW: command.  HIGH: character.
//  uint8_t _rw_pin; // LOW: write to LCD.  HIGH: read from LCD.
//  uint8_t _enable_pin; // activated by a HIGH pulse.
  //uint8_t _data_pins[4];

  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  //uint8_t _initialized;

  uint8_t _numlines,_currline;
};

#endif
