/*  
 * Model Train Controller
 * Arduino Nano Project
 * 
 * Made for Eisenbahnfreunde (efbbev.de)
 *
 */

#include <LiquidCrystal.h>

//Geometry
const int TRAIN_LEN=112; //locomotive, mm
const int TRACK_LEN=160*8+2*240; //1760mm outer loop length
const int STA_DIST=160*3+240; //From sensor to station, mm
const int WORLD_SCALE=160; // N-scale 1:160

//Fine-tunes
const int MOTOR_CUTOFF=28 * 100 / 255; //%, doesn't move at lower PWM duty

// Pin definitions
// Note: A6 A7 have no pull-up
const int MOTOR_PWM_PIN = 5; //D3 and D11 are affected by tone()
const int MOTOR_RELAY_PIN = 4; //Not installed yet
const int SPEAKER_PIN = 3;
const int RED_LED = A3; 
const int GREEN_LED = A2;

const int BUTTON_CONTROL_1 = A4;//toggle direction, trigger speed measure
const int BUTTON_CONTROL_2 = A5;//play whistle
const int BUTTON_SPEED_UP = A0;
const int BUTTON_SPEED_DOWN = A1;

//IR beam 36KHz on Timer1, works only for D9 pin
const int IR_BEAM = 9;
const int IR_SENSOR = 2;

// LCD pins (4-bit mode)
// We try to use right-side pins to drive LCD, avoiding wire intersections
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

// Global vars
int pwmDuty = 0;
long detectedSpeed = -1;
unsigned long lastDetectionTS = 0;
unsigned loops = 0; 
bool motorForward = true;

bool demoMode = false;
int demoStage = 0;

// Autonomous DEMO mode
enum DemoStages {
	DemoInit = 0,
	DemoIntro1,
	DemoIntro2,
	DemoIntro3,
	DemoDetectSpeed,
	DemoSlowdown,
	DemoStationStop,
	DemoStationWait,
	DemoEnd
};
const int stageLengs[] = {0, 2000, 2000, 2000, 0, 0, 10000, 5000, 0};


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
	pinMode(RED_LED, OUTPUT);
	pinMode(GREEN_LED, OUTPUT);
	digitalWrite(RED_LED, LOW);
	digitalWrite(GREEN_LED, LOW);
	
	lcd.begin(16, 2);
	lcd.print("N Eisenbahn v0.1");
	lcd.setCursor(0, 1);
	lcd.print("von Roman & Vad");
	delay(2000);
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

	//Motor PWM (Timer2) default is 490 (up to 1kHz is ok)

}

void loop() {
	if (demoMode) {
		updateDemoState();
		return;
	}
	pollButtons();
	if (pollTrainSensor(detectedSpeed == -1)) {
		playTone(400, 5);
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
//can be called subsequentally with detect_speed=true
long pollTrainSensor(bool detect_speed) {
  
	if (_expectLevel(HIGH)) { //BEAM interrupted

		long start_ts = millis();
		loops++;
		
		if (!detect_speed) return start_ts;

		//Filter out unstable beam on train edges
		delay(100); //no train pass through in 0.1s
		while (millis() < start_ts + 10000) {//locomotive can't pass longer than 10s?
			delay(10); //poll at ~100hz, sensor isn't too fast
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
	
	// Control button 1 with debouncing
	if (digitalRead(BUTTON_CONTROL_1) == LOW && (millis() - lastControl1Press) > 200) {
		// Toggle direction
		motorForward = !motorForward;
		setMotorDirection(motorForward);
		lastControl1Press = millis();
		detectedSpeed = -1;
		//kickstart engine
		setMotorSpeed(255);
		delay(10);
		setMotorSpeed(pwmDuty);
	}
	
	// Control button 2 with debouncing
	if (digitalRead(BUTTON_CONTROL_2) == LOW && (millis() - lastControl2Press) > 200) {
		//two buttons simultaneously
		if (lastControl1Press - lastControl2Press < 200) {
			demoMode = true;
			return;
		}
		lastControl2Press = millis();
		whistleBlast(1500); //During the whistle sensor is inactive
	}
	
	// Speed buttons with autorepeat
	if (digitalRead(BUTTON_SPEED_UP) == LOW) {
		pwmDuty = min(pwmDuty + 1, MAX_PWM);
	}
	
	if (digitalRead(BUTTON_SPEED_DOWN) == LOW) {
		pwmDuty = max(pwmDuty - 1, MIN_PWM);
	}
	
}

void updateMotor() {
	// Frequent calls can drop PWM duty cycles
	static long lastRefresh = 0;
	long ts = millis();
	if (ts - lastRefresh > 100) {
		setMotorSpeed(pwmDuty);
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
		lcd.setCursor(0, 0); //clear is slow
		lcd.print("Lst:");
		lcd.print(pwmDuty * 100 / 255);
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
	delay(duration); //TODO: check if blocking necessary
	noTone(SPEAKER_PIN);
}

void whistleBlast(int totalDuration) {
	unsigned long start = millis();
	unsigned long now;
	
	while ((now = millis()) - start < totalDuration) {

		unsigned long progr = (now - start) * 100 / totalDuration;
		
		// Attack Phase (first 10%)
		if (progr < 10) {
		  int freq = 2500 + progr * 30;   // rise from 2.5kHz to ~3.4kHz
		  tone(SPEAKER_PIN, freq);
		  delay(15 + (20 * (10 - progr))); // more interruptions early
		  noTone(SPEAKER_PIN);
		  delay(10);
		}
		
		// Chaos Phase
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
	digitalWrite(RED_LED, detectedSpeed >= 0 ? LOW : HIGH);
  
	digitalWrite(GREEN_LED, pwmDuty > 0 ? HIGH : LOW);
}

void updateDemoState() {
	//Common logic for diving throug stages:
	//switch-case executed only once per stage and can block
	
	static long detectionTS = 0;
	static long stageTS = 0;
	static bool stageProcessed = false;
	
	const char *str1 = nullptr;
	const char *str2 = nullptr;

////DemoInit = 0,
	//DemoIntro1,	DemoIntro2,	DemoIntro3, DemoDetectSpeed,DemoSlowdown,
	//DemoSlowdown,DemoStationStop,DemoStationWait
	// This should be executed only once per stage
	if (!stageProcessed) {
		switch (demoStage) {
			case DemoInit:
				//reset staic vars, stop motors, etc.
				detectedSpeed=-1;
				digitalWrite(GREEN_LED, LOW);
				digitalWrite(RED_LED, LOW);
			break;
			case DemoIntro1:
				str1 = "DEMO MODE";
				str2 = "Willkommen!";
			break;
			case DemoIntro2:
					//  0123456789ABCDEF
				str1 = "Zug aus1982 kann";
				str2 = "noch gut fahren";
			break;
			case DemoIntro3:
				digitalWrite(GREEN_LED, HIGH);
				whistleBlast(1000);
				str1 = "Los!";
				str2 = " ";
				pwmDuty = random(50, 80) * 100 / 255;
				setMotorSpeed(pwmDuty);
			break;
			case DemoDetectSpeed:
				//can take a while, maybe introduce timeout error
				while ((detectionTS = pollTrainSensor(true)) == 0) {};
				lcd.clear();
				lcd.write("x160:"); lcd.print(detectedSpeed*36*16/1000); lcd.print("km/h");
				lcd.setCursor(0, 1);
				lcd.write("Gsw:"); lcd.print(detectedSpeed); lcd.print("mm/s");
			break;
			case DemoSlowdown:
			{
				//At this point we know the speed and remaining distance to the station
				//STA_DIST - TRAIN_LEN;
				//But we want to start slowing down 240mm before destination
				int dist = TRACK_LEN + STA_DIST - TRAIN_LEN - 240;
				delay(dist * 1000 / detectedSpeed);
				lcd.clear();
				lcd.write("Ankunft...");
				//Here tune-up is required because speed varies
				//pwmDuty -> MOTOR_CUTOFF, assume linear speed dependency
				// 240mm till stop point
				int decelerate = (long)detectedSpeed * detectedSpeed / 2 / 240; //mm/s^2
				int timeToStop = detectedSpeed / decelerate;
				int speedToPWM = 100*detectedSpeed / (pwmDuty - MOTOR_CUTOFF);
				int spd = detectedSpeed;
				for (int i=0; i<timeToStop; i++) {
					spd-= decelerate;
					setMotorSpeed(spd * speedToPWM / 100);
					lcd.setCursor(0, 1);
					lcd.write("Gsw:"); lcd.print(spd); lcd.print("mm/s");
					delay(1000);
				}
				setMotorSpeed(0);
			}
			break;
			case DemoStationStop:
					//  0123456789ABCDEF
				str1 = "Zug endet hier";
				str2 = "bitte aussteigen";
			break;
			case DemoStationWait:
					//  0123456789ABCDEF
				str1 = "Abfart jetzt";
				str2 = "bitte einteigen";
			break;
			case DemoEnd:
				demoMode = false;
			break;	
		}

		if (str1) {
			lcd.clear();
			lcd.write(str1);
			lcd.setCursor(0, 1);
			lcd.write(str2);
		}
		stageProcessed = true;
	}

	//Timeout?
	if (stageLengs[demoStage] == 0 || ((millis() - stageTS) > stageLengs[demoStage])) {
		demoStage++; //TODO handle overflow
		if (demoStage > DemoStationWait) demoStage = DemoIntro3;
		stageTS = millis();
		stageProcessed = false;
	}
  
}
