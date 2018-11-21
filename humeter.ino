
// Designed for Atmega48 and higher
// Read Rh and t. Collect t and calculate average, min and max value
// Uses sleep mode, and wake-up by watchdog every 8 seconds
// Draw graph of t and Rh

// uploaded at 3.25, atmega48

// Now compiles to 3.8k
// LCD1602 connected to pins 0-5
// DHT11 to pin 10


//#include <dht11.h>
//#include <MiniLCD.h>
#include <PortDLCD.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// points to store per 24h
#define STORE_POINTS 24

#define CELSIUS_CHAR 5
#define DEW_CHAR 6
#define RH_CHAR 7

char strBuff[5]; // for converting integers
uint8_t avgBuf[2][STORE_POINTS+1]; // last value is current one

#define CUR_TEMP avgBuf[0][STORE_POINTS]
#define CUR_HUM avgBuf[1][STORE_POINTS]

bool avgEnough; // true after 1st 24h

uint8_t point; // 24h buffer pointer
unsigned long hour_ts;
unsigned long man_ms; // manually driven'timer'
bool avgTempFilled;
bool modeTrigger; // switch between T average and R average modes for 2nd line

uint8_t const celsiusChar[8] = {
  0b11000,
  0b11000,
  0b00000,
  0b00011,
  0b00100,
  0b00100,
  0b00011,
  0b00000
};

//uint8_t const dewChar[8] = {
//  0b00100,
//  0b00100,
//  0b01010,
//  0b10001,
//  0b10001,
//  0b10001,
//  0b01110,
//  0b00000
//};
// dark version
uint8_t const dewChar[8] = {
  0b00100,
  0b00100,
  0b01110,
  0b11111,
  0b11101,
  0b11101,
  0b01110,
  0b00000
};


uint8_t const RH[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b10100,
  0b11100,
  0b10100
};


// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

//dht11 DHT11;
//MiniLCD lcd(12, 11, 5, 4, 3, 2);
PortDLCD lcd;

uint8_t buffer[5][8]; // graph screen buffer 25x8 px

///////////// DHT
#define PB_DHT_PIN 2

//int humidity;
//int temperature;
#define DHTLIB_OK                0
#define DHTLIB_ERROR_CHECKSUM   -1
#define DHTLIB_ERROR_TIMEOUT    -2



// this routine depends on freq
int ackOrTimeout(uint8_t level) {//PD_DHT_PIN
    unsigned long loopCnt = 500000L;
    //while(digitalRead(DHT_PIN) == level)
    while(((PINB & (1<< PB_DHT_PIN)))  == (level << PB_DHT_PIN))
        if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;
    
    return DHTLIB_OK;
}


int readDHT11() {
    // BUFFER TO RECEIVE
    uint8_t bits[5];
    uint8_t cnt = 7;
    uint8_t idx = 0;
    
    // EMPTY BUFFER
    for (int i=0; i< 5; i++) bits[i] = 0;
    
    // REQUEST SAMPLE
    DDRB |= 1<<PB_DHT_PIN;//pinMode(DHT_PIN, OUTPUT);
    PORTB &= ~(1 << PB_DHT_PIN); //digitalWrite(DHT_PIN, LOW);
    delay(18);
    PORTB |= 1<<PB_DHT_PIN;//digitalWrite(DHT_PIN, HIGH);
    delayMicroseconds(40);
    
    // Prepare to listen
    DDRB &= ~(1 << PB_DHT_PIN);//pinMode(DHT_PIN, INPUT);
    
    // ACKNOWLEDGE or TIMEOUT
    if (ackOrTimeout(LOW)) return DHTLIB_ERROR_TIMEOUT;
    if (ackOrTimeout(HIGH)) return DHTLIB_ERROR_TIMEOUT;
    
    // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
    for (int i=0; i<40; i++)  {
        // measure pulse
        if (ackOrTimeout(LOW)) return DHTLIB_ERROR_TIMEOUT;
        unsigned long t = micros();
        if (ackOrTimeout(HIGH)) return DHTLIB_ERROR_TIMEOUT;
        
        if ((micros() - t) > 40) bits[idx] |= (1 << cnt);
        if (cnt == 0)   // next byte?
        {
            cnt = 7;    // restart at MSB
            idx++;      // next byte!
        }
        else cnt--;
    }
    
    // WRITE TO RIGHT VARS
    // as bits[1] and bits[3] are allways zero they are omitted in formulas.
    CUR_HUM = bits[0];
    CUR_TEMP = bits[2];
    
    uint8_t sum = bits[0] + bits[2];
    
    if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;
    return DHTLIB_OK;
}


inline uint8_t drawPixel(uint8_t x, uint8_t y) {
  uint8_t row = y / 8, subrow = y % 8;
  uint8_t col = x / 5, cbit = x % 5;
  
  uint8_t charIdx = row*4+col;
  // find character index
  buffer[charIdx][subrow] |= (1 << (4-cbit));

  return charIdx;
}

inline void clearBuff() {
    for (uint8_t i=0; i<5; i++) 
      for (uint8_t j=0; j<8; j++)  buffer[i][j]=0; 
}

void flushBuffer() {
   for (uint8_t i=0; i<5; i++)
    lcd.createChar(i, buffer[i]);
}


// Sleep with watchdog
ISR(WDT_vect) {
}

long intLog(long x) { //x = 0...1000
    // table step is 0.1
    long logTable[10] = {-29999, -2302, -1609, -1204, -916, -693, -511, -357, -223, 0};
    
    long x1 = x / 100; x1 *= 100;
    long x2 = x1 + 100;
    long y1 = logTable[x1 / 100];
    long y2 = logTable[x2 / 100];
    
    long w1 = ((x2 - x) << 8) / (x2 - x1);
    long w2 = ((x - x1) << 8) / (x2 - x1);
    
    return (y1 * w1 + y2 * w2) >> 8;
}

long dewPointInt(int celsius, int humidity) {
    long a = 17271;
    long b = 237700;
    
    long temp = (a * celsius * 1000 ) / (b + celsius * 1000) + intLog(humidity * 10) ;
    long Td = (b * temp ) / (a - temp) / 1000;
    return Td;
}


void setup() {
  avgTempFilled = false;

  lcd.begin(16, 2);   
  // service chars
  lcd.createChar(CELSIUS_CHAR, (uint8_t*)celsiusChar);
  lcd.createChar(DEW_CHAR, (uint8_t*)dewChar);
  lcd.createChar(RH_CHAR, (uint8_t*)RH);
  
  /*Turn off peripherals*/
  ADCSRA &= ~(1<<ADEN); //Disable ADC
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F;  //Disable digital input buffers on all ADC0-ADC5 pins.
  
  // joint from power.h
  (PRR |= (uint8_t)((1<<PRADC)|(1<<PRSPI)|(1<<PRUSART0)|(1<<PRTIM2)|(1<<PRTWI)));
  
  /*** Setup the WDT ***/
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE); // this line is necessary! even if next overrides!
  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */ 
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
}

const char symbols[2] = {'t','\7'};
const char units[2] = {CELSIUS_CHAR, '%'};


void draw2ndLine(uint8_t optionIndex) {
    // statistics for last 24h
      short sum = 0;
      short vmax = -100;
      short vmin = 100;  
      uint8_t i=0;
      
      for (; i<STORE_POINTS; i++) {
          sum+=avgBuf[optionIndex][i];
          vmax = max(vmax, avgBuf[optionIndex][i]);
          vmin = min(vmin, avgBuf[optionIndex][i]);
      }
      // build graphic
      clearBuff();
      int mamp = vmax - vmin; //max amplitude
      if (!mamp) mamp = 1; //protect from div by 0
      
      for (i=0; i<STORE_POINTS; i++) {  // last 25th bar - current value
          short y = (avgBuf[optionIndex][i] - vmin) * 7 / mamp;
          for (int h = 0; h<y+1; h++)
            drawPixel(i, 7-h);
      }
      // if trend increasing, draw upper dot
      if (avgBuf[optionIndex][STORE_POINTS]>avgBuf[optionIndex][STORE_POINTS-1])
          drawPixel(STORE_POINTS, 0);
      else if (avgBuf[optionIndex][STORE_POINTS]<avgBuf[optionIndex][STORE_POINTS-1])
          drawPixel(STORE_POINTS, 6);
      else
          drawPixel(STORE_POINTS, 3); //equal
      
      flushBuffer();
      
      lcd.setCursor(0,1);
      lcd.write('A'); lcd.write(symbols[optionIndex]);  //lcd.write("a:");
      lcd.write(avgEnough ? itoa(sum/STORE_POINTS, strBuff, 10) : "--");
      lcd.write(units[optionIndex]); lcd.write(' ');
      lcd.write(itoa(vmin, strBuff, 10));
      lcd.write((uint8_t)0); lcd.write("\1\2\3\4"); // graph
      lcd.write(itoa(vmax, strBuff, 10));
      
      
}

void loop() { //performed once a 16 second than sleeps
    
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  int chk = readDHT11();
  
  // Draw 1st line with current values
  //
  lcd.setCursor(0,0);
  if (chk == DHTLIB_OK) {  
      //int calibratedHumidity = humidity+2;//seems 2 is required
      CUR_HUM+=4; //changed 2->4 (compared with dht-22. Not uploaded yet
      if (!avgTempFilled) { //initial fill with current temperature
          for (uint8_t i=0; i<STORE_POINTS; i++) {
              avgBuf[0][i] = CUR_TEMP;  
              avgBuf[1][i] = CUR_HUM;
          }
          avgTempFilled = true;
      } else {
          if (man_ms - hour_ts > 24*3600000 / STORE_POINTS) { // save temp half-hourly
              // shift values
              for (uint8_t i=0; i<STORE_POINTS; i++) {
                 avgBuf[0][i] = avgBuf[0][i+1];
                 avgBuf[1][i] = avgBuf[1][i+1];  
              }
              
              //avgBuf[0][STORE_POINTS-1] =  temperature;
              //avgBuf[1][STORE_POINTS-1] = calibratedHumidity;
              
              hour_ts = man_ms;
              if (point == STORE_POINTS-1) { 
                  point=0;
                  avgEnough = true;
              } else
                  point++;
          }
      }
 //  Screen Info Layout:
 //  1234567890ABCDEF
 //  t 22C Rh22% d10C
 //  At 22 20_____25C
 //  Ah 40 30_____50% 
      
      lcd.write("t ");
      lcd.write(itoa(CUR_TEMP, strBuff, 10));
      lcd.write(CELSIUS_CHAR); lcd.write(" R");lcd.write(RH_CHAR);
      lcd.write(itoa(CUR_HUM, strBuff, 10));
      lcd.write("% "); lcd.write(DEW_CHAR);
      lcd.write(itoa(dewPointInt(CUR_TEMP, CUR_HUM), strBuff, 10));
      lcd.write(CELSIUS_CHAR); lcd.write(' ');
      
 
  } else {
      if (chk == DHTLIB_ERROR_TIMEOUT) lcd.write("Check DHT sensor");

  }
  
       
      // draw second line   
  draw2ndLine(modeTrigger=!modeTrigger);

  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
  sleep_mode();
  sleep_disable();
  man_ms += 16900; // calibrated for 8MHz
  
}
