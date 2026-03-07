/*  
 * Model Train Controller
 * Arduino Nano Project
 * 
 * Components:
 * - Motor PWM (pin 3)
 * - Motor polarity relay (pin 4), not yet here
 * - Speaker (pin 5)
 * - Traffic lights (pins A2, A3)
 * - LCD 1602 (pins 6, 7, 8, 10, 11, 12)
 * - Control buttons (A4, A5)
 * - Speed buttons (A0, A1)
 * - IR Sensor (pin 2)
 * - IR beam LED (pin 9) @36 kHz
 *
 * BUGS: motor interrupted during tone playback. Same timer?
 */

#include <LiquidCrystal.h>

//Geometry
const int TRAIN_LEN=112; //mm
const int TRACK_LEN=155*8+2*240*2; //2200mm outer length
const int WORLD_SCALE=160; // N-scale 1:160

// Pin definitions
const int MOTOR_PWM_PIN = 3;
const int MOTOR_RELAY_PIN = 4;
const int SPEAKER_PIN = 5;
const int LIGHT_1_PIN = A3;
const int LIGHT_2_PIN = A2;

//A6 A7 have no pull-up
const int BUTTON_CONTROL_1 = A4;
const int BUTTON_CONTROL_2 = A5;
const int BUTTON_SPEED_UP = A0;
const int BUTTON_SPEED_DOWN = A1;

//IR beam 36KHz on Timer1
const int IR_BEAM = 9;
const int IR_SENSOR = 2;

// LCD pins (4-bit mode)
// We try to use right-side pins to drive LCD
const int LCD_RS = 6;
const int LCD_EN = 7;
const int LCD_D4 = 8;
const int LCD_D5 = 10;
const int LCD_D6 = 11;
const int LCD_D7 = 12;

const int LCD_REFRESH = 1000 / 5; // 5Hz
// Motor control
const int MIN_PWM = 0;
const int MAX_PWM = 255;
int currentSpeed = 0;
long detectedSpeed = -1;
bool motorForward = true;
bool demoMode = false;
int demoStage = 0;

// Autonomous DEMO mode, TODO
enum demoStages {
	  DemoInit=0, //stop everything
	  DemoIntro1,
	  DemoIntro2,
	  DemoDetect,
	  DemoDetectSpeed,
	  DemoIntro3,
	  DemoWhistle,
	  DemoSlowdown,
	  DemoEnd
};

const char* stageStrs[] = {
   //0123456789ABCDEF
	"",
	"DEMO MODE\nWillkommen!",
	"1982 Zug can\n fahren auch",
	""
};
const int stageLengs[] = {0, 2000, 2000, 4000};

// LCD setup
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void setup() {
	pinMode(MOTOR_RELAY_PIN, OUTPUT);
	pinMode(MOTOR_PWM_PIN, OUTPUT);
	digitalWrite(MOTOR_RELAY_PIN, LOW); // Default forward
	
	pinMode(SPEAKER_PIN, OUTPUT);
	
	pinMode(BUTTON_CONTROL_1, INPUT_PULLUP);
	pinMode(BUTTON_CONTROL_2, INPUT_PULLUP);
	pinMode(BUTTON_SPEED_UP, INPUT_PULLUP);
	pinMode(BUTTON_SPEED_DOWN, INPUT_PULLUP);
	
	//IR sensing
	pinMode(IR_SENSOR, INPUT);
	pinMode(IR_BEAM, OUTPUT);
	
	// Initialize traffic lights
	pinMode(LIGHT_1_PIN, OUTPUT);
	pinMode(LIGHT_2_PIN, OUTPUT);
	digitalWrite(LIGHT_1_PIN, LOW);
	digitalWrite(LIGHT_2_PIN, LOW);
	
	lcd.begin(16, 2);
	lcd.print("N Eisenbahn v0.1");
	lcd.setCursor(0, 1);
	lcd.print("von Roman & Vad");
	delay(1000);
    lcd.clear();
	
	Serial.begin(9600);
	
	// Configure Timer1 for 36 kHz
	// I don't give a fuck about arduino registers
	TCCR1A = 0;
	TCCR1B = 0;
	
	TCCR1A = _BV(COM1A0);               // Toggle OC1A on compare
	TCCR1B = _BV(WGM12) | _BV(CS10);    // CTC mode, no prescaler
	OCR1A = 221;                        // 36 kHz
	//OCR1A = 210; //38 kHz

	//Motor PWM (Timer2) default is 490
    //3TCCR2B = TCCR2B & B11111000 | B00000011; // 980.39 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000110; //  122.55 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000010; //  3921.16 Hz

}

void loop() {
	if (demoMode) {
		updateDemoState();
			return;
		}
		pollButtons();
		if (pollSensors(detectedSpeed == -1)) {
			//playTone(400, 10);
		}
		updateMotor();
		updateDisplay();
		updateTrafficLights();
		delay(50);
}

bool _expectLevel(int lvl) {
	// Lightup the BEAM!
	TCCR1A |= _BV(COM1A0);
	delayMicroseconds(500);
	// Disable carrier
	TCCR1A &= ~_BV(COM1A0);
	digitalWrite(IR_BEAM, LOW);

	return  digitalRead(IR_SENSOR) == lvl;
} 

//can block for 10s if called with detect speed 
long pollSensors(bool detect_speed) {
  
	if (_expectLevel(HIGH)) { // Beam interrupted

		long start_ts = millis();
		//detectedSpeed = -1;
		
		if (!detect_speed) return start_ts;
        delay(100); //no train pass through in 0.1s
		while (millis() < start_ts + 10000) {//10s naive constant
			delay(10); //poll at 100hz
			if (_expectLevel(LOW)) { //BEAM restored
				detectedSpeed = constrain((long)TRAIN_LEN * 1000 / (millis() - start_ts), 1, 999);
				return start_ts;
			}
				
		}
		
	}
    return 0;
}

void pollButtons() {
	static long lastControl1Press = 0;
	static long lastControl2Press = 0;
	static bool speedUpLocked = true;
	static bool speedDownLocked = true;
	
	// Control button 1 with debouncing
	if (digitalRead(BUTTON_CONTROL_1) == LOW && (millis() - lastControl1Press) > 200) {
		// Toggle direction
		//Serial.print("BTN 1\n");
		motorForward = !motorForward;
		setMotorDirection(motorForward);
		lastControl1Press = millis();
		detectedSpeed = -1;
		//kickstart engine
		setMotorSpeed(255);
		delay(10);
		setMotorSpeed(currentSpeed);
		
		//playTone(440, 50);
	}
	
	// Control button 2 with debouncing
	if (digitalRead(BUTTON_CONTROL_2) == LOW && (millis() - lastControl2Press) > 200) {
		lastControl2Press = millis();
        setMotorSpeed(0); //tone() interferes w/ PWM
		whistleBlast(1300);
	}
	
	// Speed buttons with debouncing
	if (digitalRead(BUTTON_SPEED_UP) == LOW/* && speedUpLocked*/) {
		currentSpeed = min(currentSpeed + 1, MAX_PWM);
		speedUpLocked = false;
	} else if (digitalRead(BUTTON_SPEED_UP) == HIGH) {
		speedUpLocked = true;
	}
	
	if (digitalRead(BUTTON_SPEED_DOWN) == LOW/* && speedDownLocked*/) {
		currentSpeed = max(currentSpeed - 1, MIN_PWM);
		speedDownLocked = false;
	} else if (digitalRead(BUTTON_SPEED_DOWN) == HIGH) {
		speedDownLocked = true;
	}
	
}

void updateMotor() {
	// Frequent calls drop PWM duty cycles
	static long lastRefresh = 0;
	long ts = millis();
	if (ts - lastRefresh > 200) {
		setMotorSpeed(currentSpeed);
		lastRefresh = ts;
	}
}

void setMotorSpeed(int speed) {
	analogWrite(MOTOR_PWM_PIN, speed);
}

void setMotorDirection(bool forward) {
	digitalWrite(MOTOR_RELAY_PIN, forward ? LOW : HIGH);
}

void updateDisplay() {
	static long lastRefresh = 0;
	long ts = millis();
	if (ts - lastRefresh > LCD_REFRESH) {
		lcd.setCursor(0, 0);
        //lcd.clear(); //clear is slow
		lcd.print("Lst:");
		lcd.print(currentSpeed * 100 / 255);
		lcd.print("%  ");
	  
		lcd.setCursor(0, 1);
		lcd.print("Gsw:");
		if (detectedSpeed >= 0) {
    lcd.print(detectedSpeed);
				lcd.print("mm/s");
                lcd.print(" ");
                lcd.print(detectedSpeed * 36 * 16 / 1000); // kmh: 3.6 * 160 / 1000 
		} else {
				lcd.print("messen...    ");
		}
		lastRefresh = ts;
	}
}

void playTone(int freq, int duration) {
	tone(SPEAKER_PIN, freq, duration);
	delay(duration);
	noTone(SPEAKER_PIN);
}

void whistleBlast(int totalDuration) {
	unsigned long start = millis();
	unsigned long now;
	
	//TODO get rid of float
	while ((now = millis()) - start < totalDuration) {
	
		float progress = float(now - start) / totalDuration;
		
		// ---- Attack Phase (first 30%) ----
		if (progress < 0.1) {
		  int freq = 2500 + progress * 3000;   // rise from 2.5kHz to ~3.4kHz
		  tone(SPEAKER_PIN, freq);
		  delay(15 + (20 * (0.3 - progress))); // more interruptions early
		  noTone(SPEAKER_PIN);
		  delay(10);
		}
		
		// ---- Full Chaos Phase ----
		else {
		  int freq = random(2600, 3000);
		  tone(SPEAKER_PIN, freq);
		  delay(random(12, 12));
		  noTone(SPEAKER_PIN);
		  delay(random(3, 6));
		}
	}
}


void updateTrafficLights() {
	// Turn on light 1 when motor is running
	digitalWrite(LIGHT_1_PIN, detectedSpeed >= 0 ? LOW : HIGH);
  
	// Turn on light 2 when motor is reversing
  	digitalWrite(LIGHT_2_PIN, currentSpeed > 0 ? HIGH : LOW);
}

void updateDemoState() {
	static long detectionTS = 0;
	static long stageTS = 0;
	const char *str = NULL;
	bool timeout = (millis() - stageTS) > stageLengs[demoStage];

	switch (demoStage) {
		case DemoInit:
			//reset staic vars, stop motors, etc.
		
		break;
		case DemoIntro1:
			str = "DEMO MODE\nWillkommen!\n";
		break;
		case DemoIntro2:
			str = "1982 Zug\nkann noch fahren\n";
		break;
		case DemoEnd:
			demoMode = false;
	}

	if (str) {
		lcd.clear();
		lcd.write(str);
	}

	if (timeout) {
		demoStage++;
		stageTS = millis();
	}
  
}
