#ifndef MiniLCD_h
#define MiniLCD_h

// Shrinked version of LiquidCrystal arduino library
//
// Only 4-bit parallel write-only mode supported

#include <inttypes.h>

class MiniLCD {
public:

  MiniLCD(uint8_t rs, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);

    
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = 0 /*LCD_5x8DOTS*/);

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
  
  uint8_t print(const char *str);
  uint8_t print(long value);
private:
  uint8_t write(uint8_t value);
  void wait4_1ms();
  void send(uint8_t, uint8_t);
  void write4bits(uint8_t);
//  void write8bits(uint8_t);
  void pulseEnable();

  uint8_t _rs_pin; // LOW: command.  HIGH: character.
  uint8_t _enable_pin; // activated by a HIGH pulse.
  uint8_t _data_pins[4];

  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _numlines,_currline;
};

#endif
