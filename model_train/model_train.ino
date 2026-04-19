/*  
 * Model Train Controller
 * Arduino Nano Project
 * 
 * Made for Eisenbahnfreunde (efbbev.de)
 * DR Baureihe 118
 * Reference train https://de.wikipedia.org/wiki/DR-Baureihe_V_180
 * Speed: up to 120 km/h
 * Length: 19.5m				:160 12.2cm
 * Power: 1380 kW
 * Service weight: 95t 			:160 23g
 * Traction: 270 kN
 */

#include <LiquidCrystal.h>

// Use a potentiometer as input instead of step +/- buttons
#define ANALOG_THROTTLE 0
// There's Arduino HAT format PCB with different PIN connections
#define ARDUINO_HAT 0

//Track Geometry
//Curved track: outer length 155, inner length 150
constexpr int TRAIN_LEN=112; //locomotive, mm
constexpr int TRAIN_CLEN=TRAIN_LEN * 152 / 145; //Observable length in the curve, where sensor located
constexpr int TRACK_LEN=152*8+2*240+2*110+2*50; //1980mm outer loop length
constexpr int STA_DIST=1450; //From sensor to Station platform 1, mm
constexpr int STA2_DIST = 1330;//From sensor to Station platform 2, mm
constexpr int WORLD_SCALE=160; // N-scale 1:160

//Fine-tunes
constexpr int MOTOR_CUTOFF=23 * 255 / 100; //%, train doesn't move at lower PWM duty
constexpr int MOTOR_MAX_REAL=55 * 255 / 100; //55%, at this power train moves 140km/h, which is MAX for 1970s DR Class V119 diesel loc

// //////////////////////////////
// Pin definitions
// Note: A6 A7 have no pull-up
/////////////////////////////////
const int MOTOR_PWM_PIN = 5; //D3 and D11 are affected by tone()
const int MOTOR_RELAY_PIN = 4;
const int SWITCH_LEFT_PIN = A0;
const int SWITCH_RIGHT_PIN = A1;
const int SPEAKER_PIN = 3;
const int RED_LED = A2;
const int GREEN_LED = A3;

const int BUTTON_1 = A4;//trigger speed measure
const int BUTTON_2 = A5;//play whistle, switch rails
const int SPEED_HANDLE = A6; //Voltage dividor, optionally linear potentiometer as option with direct speed mapping
const int RAIL_CURRENT = A7; //TODO short circuit

//IR beam 36KHz on Timer1, works only for D9 pin
const int IR_BEAM = 9;
const int IR_SENSOR = 2;

// LCD pins (4-bit mode)
// We try to use right-side pins to drive LCD, avoiding PCB wire intersections
#if ARDUINO_HAT
const int LCD_RS = 12;
const int LCD_EN = 11;
const int LCD_D4 = 10;
const int LCD_D5 = 8;
const int LCD_D6 = 7;
const int LCD_D7 = 6;
#else
const int LCD_RS = 6;
const int LCD_EN = 7;
const int LCD_D4 = 8;
const int LCD_D5 = 10;
const int LCD_D6 = 11;
const int LCD_D7 = 12;
#endif

//
// Run params
//
const int LCD_REFRESH = 1000 / 5; // 5Hz
// Motor operation range
const int MIN_PWM = -255; //Set to 0 if no relay installed
const int MAX_PWM = 255;
const int MAX_RAIL_CURRENT = 200; //mA

// Global vars
bool sensorInstalled = false;
long detectedSpeed = 0;
unsigned long lastDetectionTS = 0;
unsigned railCurrent = 0; //Actually a mapped voltage  on a 0.5 Ohm resistor

#if ANALOG_THROTTLE
unsigned long lastThrottleUpdate = 0;
#endif

int pwmDuty = 0; //(MIN_PWM..MAX_PWM) negative value is for motor reverse

bool demoMode = false;
bool setupMode = false; //TODO: let user customize EEPROM	 constants on 1st start
int demoStage = 0;
bool switchIsLeft = false; //actual initial position is unknown

// Autonomous DEMO mode
enum DemoStages {
	DemoInit = 0,
	DemoIntro1,
	DemoIntro2,
	DemoIntro3,
	DemoDetectSpeed,
	DemoSlowdown,
	DemoStationStop,
	DemoReverse,
	DemoStationWait,
	DemoEnd
};
const int stageLengs[] = {0, 2000, 2000, 2000, 0, 0, 10000, 0, 5000, 0};


// LCD setup
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void setup() {
	
	//Motor
	pinMode(MOTOR_PWM_PIN, OUTPUT);
	pinMode(MOTOR_RELAY_PIN, OUTPUT);
	//Motor and relay OFF
	analogWrite(MOTOR_PWM_PIN, 0);
	delay(10);
	digitalWrite(MOTOR_RELAY_PIN, LOW);

	//Ensure Rail switch to Left
	pinMode(SWITCH_LEFT_PIN, OUTPUT);
	pinMode(SWITCH_RIGHT_PIN, OUTPUT);
	switchIsLeft = false;
	toggleSwitch();//set it to LEFT!

	pinMode(SPEAKER_PIN, OUTPUT);
	
	pinMode(BUTTON_1, INPUT_PULLUP);
	pinMode(BUTTON_2, INPUT_PULLUP);
	pinMode(SPEED_HANDLE, INPUT);
	pinMode(RAIL_CURRENT, INPUT);

	if (digitalRead(BUTTON_1) == LOW && digitalRead(BUTTON_2) == LOW) {
		setupMode = true;
	}
	
	// Initialize traffic lights
	pinMode(RED_LED, OUTPUT);
	pinMode(GREEN_LED, OUTPUT);
	digitalWrite(RED_LED, LOW);
	digitalWrite(GREEN_LED, LOW);
	
	lcd.begin(16, 2);
	lcd.print("N Eisenbahn v0.2");
	lcd.setCursor(0, 1);
	lcd.print("von Roman & Vad");
	delay(2000);
	
	//Serial.begin(9600);
	
	// Configure Timer1 for 36 kHz
	// I don't give a fuck about arduino registers
	TCCR1A = 0;
	TCCR1B = 0;
	
	TCCR1A = _BV(COM1A0);               // Toggle OC1A on compare
	TCCR1B = _BV(WGM12) | _BV(CS10);    // CTC mode, no prescaler
	OCR1A = 221;                        // 36 kHz
	//OCR1A = 210; //38 kHz

	//Motor PWM (Timer2) default is 490 (up to 1kHz is ok)
	//Maybe change
	
	//Check if sensor installed
	pinMode(IR_SENSOR, INPUT_PULLUP);
	pinMode(IR_BEAM, OUTPUT);
	delay(1);
	sensorInstalled = _expectLevel(LOW);
	pinMode(IR_SENSOR, INPUT);
	
	if (!sensorInstalled) {
		lcd.clear();
		lcd.print("Ohne Sensor");
		delay(1000);
	}

	lcd.clear();
}


void demoLoop() {
	updateDemoState();
}

void setupLoop() {
	//TODO let user enter params and save to EEPROM
}

void operationLoop() {
	pollButtons();
	
	if (sensorInstalled && pwmDuty && pollTrainSensor(detectedSpeed == -1)) {
		playTone(400, 5);
	}
	updateMotor();
	updateDisplay();
	updateTrafficLights();
	//TODO: Kurzschluss check
	monitorCurrent(20); //update env at 50Hz
}

void loop() {
	if (demoMode) {
		demoLoop();
	}
	else if (setupMode) {
		setupLoop();
	}
	else {
		operationLoop();
	}
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
	if (!sensorInstalled) return 0;

	if (_expectLevel(HIGH)) { //BEAM interrupted

		long start_ts = millis();
		
		if (!detect_speed) return start_ts;

		//Filter out unstable beam on train edges
		delay(100); //no train can sneak through in 0.1s
		while (millis() < start_ts + 12000) {//locomotive can't pass longer than 12s?
			delay(10); //poll at ~100hz, sensor isn't too fast
			if (_expectLevel(LOW)) { //BEAM restored
				detectedSpeed = constrain((long)TRAIN_CLEN * 1000 / (millis() - start_ts), 1, 999);
				return start_ts;
			}
		}
	}
	return 0;
}

void monitorCurrent(long delayMS) {
	long start = millis();
	unsigned maxLocal = 0;
	while (millis()-start < delayMS) {
		unsigned maxVal = 0;
		for (int i=0; i<10; i++) {
			maxVal = max(maxVal, analogRead(RAIL_CURRENT));
			delayMicroseconds(99);
		}
		maxLocal = max(maxVal, maxLocal);
		//Emergency stop if exceeds limit
		//TODO
	}

	railCurrent = maxLocal;
}

void pollButtons() {
	static long lastControl1Press = 0;
	static long lastControl2Press = 0;
	
	// BTN 1
	if (digitalRead(BUTTON_1) == LOW && (millis() - lastControl1Press) > 200) {
		lastControl1Press = millis();
		if (sensorInstalled) {
			detectedSpeed = -1;
		}
		//kickstart engine
		setMotorPower(255);
		delay(5);
		setMotorPower(pwmDuty);
	}
	
	// BTN 2
	if (digitalRead(BUTTON_2) == LOW && (millis() - lastControl2Press) > 200) {
		//two buttons simultaneously
		lastControl2Press = millis();
		if (sensorInstalled && (lastControl2Press - lastControl1Press < 200)) {
			demoMode = true;
			return;
		}
		toggleSwitch();
		whistleBlast(1500); //During the whistle sensor is inactive
	}
	
	int handle = analogRead(SPEED_HANDLE); //0..1023
	const int deadZone = 50;
	const int center = 512;
	
#if	ANALOG_THROTTLE
	int targetDuty;
	//
	if (handle > center - deadZone && handle < center + deadZone) {
    	targetDuty = 0;
  	}
  	else  if (handle <= center - deadZone) {
    	targetDuty = max(map(handle, 0, center - deadZone, -MOTOR_MAX_REAL, 0), MIN_PWM);
	}
	else {
		targetDuty = min(map(handle, center + deadZone, 1023, 0, MOTOR_MAX_REAL), MAX_PWM);
	}
  	// Soft follow the targetDuty to avoid rapid speed jumps and direction change
  	const int stepInterval = 50;   // ms between updates
	const int maxStep = 5;
	unsigned long now = millis();

  // Update in small steps every stepInterval
	if (now - lastThrottleUpdate >= stepInterval) {
		lastThrottleUpdate = now;
	
    	int diff = targetDuty - pwmDuty;

    	// Limit how fast we move toward target
    	if (diff > maxStep) diff = maxStep;
    	else if (diff < -maxStep) diff = -maxStep;

    	pwmDuty += diff;
	}
  
  
#else
	// Speed buttons with autorepeat, since we use voltage divider some tolerance required
	int step = abs(pwmDuty) < MOTOR_CUTOFF ? 2 : 1; //faster in motor dead zone
	
	if (handle > center + deadZone) {
		pwmDuty = min(pwmDuty + step, MAX_PWM);
	}
	
	if (handle < center - deadZone) {
		pwmDuty = max(pwmDuty - step, MIN_PWM);
	}
#endif	
}

void updateMotor() {
	// Too frequent calls might drop effective PWM duty cycles
	static unsigned long lastRefresh = 0;
	long ts = millis();
	if (ts - lastRefresh > 50) {
		setMotorPower(pwmDuty);
		setMotorReverse(pwmDuty < 0);
		lastRefresh = ts;
	}
}

void setMotorPower(int pwr) {
	//Enforce 0 around motor dead zone, so relay switches at 0 motor current
	int val = abs(pwr) < MOTOR_CUTOFF - 5 ? 0 : abs(pwr);
	analogWrite(MOTOR_PWM_PIN, abs(pwr));
}

void setMotorReverse(bool rev) {
	//Rule here is to avoid touching relay when train is moving
	digitalWrite(MOTOR_RELAY_PIN, rev ? HIGH : LOW);
}

void toggleSwitch() {
	switchIsLeft = !switchIsLeft;
	int pin = switchIsLeft ? SWITCH_LEFT_PIN : SWITCH_RIGHT_PIN;
	digitalWrite(pin, HIGH);
	delay(50);
	digitalWrite(pin, LOW);
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
		lcd.print("I= ");
		int mV = map(railCurrent, 0, 1023, 0, 5000); //not precise
		lcd.print(mv * 2);
		lcd.print(" mA");
		// lcd.print("Gsw:");
		// if (detectedSpeed >= 0) {
		// 	lcd.print(detectedSpeed);
		// 	lcd.print("mm/s");
		// 	lcd.print(" ");
		// 	lcd.print(detectedSpeed * 36 * 16 / 1000); // kmh: 3.6 * 160 / 1000 
		// } else {
		// 	lcd.print("messen...    ");
		// }
		lastRefresh = ts;
	}
}

void playTone(int freq, int duration) {
	tone(SPEAKER_PIN, freq, duration);
	delay(duration); //TODO: check if blocking necessary
	noTone(SPEAKER_PIN);
}

//uint8_t fastSine(uint8_t phase) {
	//static const uint8_t quarterSine[64] = {
		//0, 0, 1, 1, 2, 3, 4,
		//5, 6, 8, 9, 11, 13, 15, 17,
	//20, 22, 25, 27, 30, 33, 36, 40,
	//43, 47, 50, 54, 58, 62, 66, 71,
	//75, 80, 84, 89, 94, 99, 104, 109,
	//114, 119, 125, 130, 136, 141, 147, 153,
	//158, 164, 170, 176, 182, 188, 194, 200,
	//206, 213, 219, 225, 231, 238, 244, 250, 255
	//};
	
  //uint8_t quadrant = phase >> 6;   // 0–3
  //uint8_t index = phase & 0x3F;    // 0–63

	//if (quadrant == 0) {
		//return quarterSine[index];
	
	//else if (quadrant == 1) {
		//return quarterSine[63 - index];
	
	//else if (quadrant == 2) {
		//return 255 - quarterSine[index];
	//} 
	//else {
		//return 255 - quarterSine[63 - index];
	//}
//}

// Proto, if possible to have it w/o DAC
//void horn(int duration) {
	//tone(SPEAKER_PIN, 255);
	//delay(1000);
	
	//uint16_t phase1 = 0;
	//uint16_t phase2 = 0;
	//uint16_t phase3 = 0;

	//int divisor = pwmDuty * 100 / 255;
	//if (divisor == 0) divisor = 1;

//// Frequency steps (tune these!)
	//uint16_t step1 = 255*15/divisor;
	//uint16_t step2 = 311*15/divisor;
	//uint16_t step3 = 440*15/divisor;
	//// Fast PWM on Timer2 (pin 3 = OC2B)
	//TCCR2A = 0;
	//TCCR2B = 0;
	//TCCR2A = _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // Fast PWM
	//TCCR2B = _BV(CS20); // no prescaler (~31 kHz PWM)
	//duration = duration * 10;
	//while(duration--) {
		//// advance phases
		//phase1 += step1;
		//phase2 += step2;
		//phase3 += step3;

		//// lookup sine values
		//uint8_t s1 = fastSine(phase1 >> 8);
		//uint8_t s2 = fastSine(phase2 >> 8);
		//uint8_t s3 = fastSine(phase3 >> 8);
	
		//// mix signals
		//uint16_t mix = s1;//s1 + s2 + s3;
		////mix /= 4;
	
		//// output PWM duty cycle
		//OCR2B = mix;
	
		//// crude sample rate control
		//delayMicroseconds(10);  // adjust for pitch/smoothness
	//}
	//OCR2B = 0;
	//TCCR2A = 0;
	//TCCR2B = 0;
//}

void whistleBlast(int totalDuration) {
	//Traffic whistle with an interrupt ball
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
	//switch-case executed only once per stage and can block, since we don't need to poll user input
	
	static long detectionTS = 0;
	static long stageTS = 0;
	static bool stageProcessed = false;
	static int loops = 0;
	static bool platform2 = false;
	
	const char *str1 = nullptr;
	const char *str2 = nullptr;

	if (!stageProcessed) {
		switch (demoStage) {
			case DemoInit:
				//reset staic vars, stop motors, etc.
				detectedSpeed=-1;
				pwmDuty = 0;
				setMotorPower(0);
				delay(200);
				setMotorReverse(false);
				switchIsLeft = false;
				toggleSwitch();
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
				digitalWrite(RED_LED, LOW);
				whistleBlast(1000);
				loops++;
				str1 = "Los!";
				str2 = " ";
				pwmDuty = random(MOTOR_CUTOFF + 8, MOTOR_MAX_REAL - 20);
				// Simplified few-step acceleration
				for (int duty = MOTOR_CUTOFF; duty <= pwmDuty; duty+=4) {
					setMotorPower(duty);
					delay(100);
				}
				setMotorPower(pwmDuty);
			break;
			case DemoDetectSpeed:
				//can take a while, maybe introduce timeout error
				//detectionTS is a moment when train nose hits the sensor
				while ((detectionTS = pollTrainSensor(true)) == 0) {};
				// program continues when whole train passes the sensor
				lcd.clear();
				lcd.write("x160:"); lcd.print(detectedSpeed*36*16/1000); lcd.print("km/h");
				lcd.setCursor(0, 1);
				lcd.write("Gsw:"); lcd.print(detectedSpeed); lcd.print("mm/s");
				platform2 = (loops % 3) == 0;
				
				if (platform2) toggleSwitch();
				
			break;
			case DemoSlowdown:
			{
				//At this point we know the speed and remaining distance to the station
				//STA_DIST - TRAIN_LEN;
				//But we want to start slowing down 320mm before destination
				
				int brakingDist = 280;
				long dist = platform2 ? STA2_DIST - brakingDist : STA_DIST - brakingDist;

				if (detectedSpeed*36*16/1000 > 90 && !platform2) { //extra loop when 90+ km/h
					dist+=TRACK_LEN; //add full circle if express(fast) train 
				}

				//Let it drive w/ initial speed
				delay(dist * 1000 / detectedSpeed - (millis()-detectionTS));
				//Time to slow down, 'brakingDist' till stop
				digitalWrite(GREEN_LED, LOW);
				digitalWrite(RED_LED, HIGH);
				lcd.clear();
				lcd.write("Ankunft..");
				lcd.setCursor(0, 1);
				lcd.write("Gleis ");
				lcd.write(platform2 ? "2" : "1");
				
				//Linear deceleration from arbitrary train speed to full stop, reality is more complicated 
				//pwmDuty -> MOTOR_CUTOFF, assume linear speed dependency
				// 240mm till stop point
				long decelerate = (long)detectedSpeed * detectedSpeed / brakingDist / 2; //a=-v^2 / 2d, mm/s^2
                int timeToStop = 10*detectedSpeed / decelerate; //deci-seconds till speed reaches zero
				
				//Linear Speed to PWM interpolation (shiity. need to calculate instead)
				int cuttOff = MOTOR_CUTOFF - 7;
				long speedToPWM = 100*detectedSpeed / (pwmDuty - cuttOff);
				//Serial.print("PWM CONST: "); Serial.println(speedToPWM);

				//Ensure values at limits, aka unit test
				// Serial.print("CURR SPEED PWM:"); Serial.println(cuttOff + (long)detectedSpeed * 100 / speedToPWM );
				// Serial.print("HALF SPEED PWM:"); Serial.println(cuttOff + (long)detectedSpeed / 2 * 100 / speedToPWM );
				// Serial.print("ZERO SPEED PWM:"); Serial.println(cuttOff + 0 * speedToPWM / 100);

				for (int i=0; i<timeToStop; i++) {
					int spd = (detectedSpeed * 10 - i * decelerate) / 10;

					int pwm = cuttOff + (long)spd * 100 / speedToPWM;  // inside 0-255 range!
					setMotorPower(pwm);
					delay(100);
				}
				setMotorPower(0);
			}
			break;
			case DemoStationStop:
					//  0123456789ABCDEF
				str1 = "Zug endet hier";
				str2 = "bitte aussteigen";
			break;
			case DemoReverse:
				// We return it to platform 1
				if (platform2)  {
					setMotorReverse(true);
					setMotorPower(35*255/100);
					while (pollTrainSensor(true) == 0) {
						delay(50);
					}
					//At this moment last cart has passed the sensor
					//TRACK_LEN-STA_DIST to go
					//Cart is 1.5 shorter, so
					int speed = detectedSpeed * 3 / 2;
					delay((TRACK_LEN-STA_DIST+TRAIN_LEN) * 1000 / speed);
					
					setMotorPower(MOTOR_CUTOFF);
					delay(500);
					setMotorPower(0);
					toggleSwitch();
					delay(200);
					setMotorReverse(false);
				}
			break;
			case DemoStationWait:
					//  0123456789ABCDEF
				str1 = "Abfahrt jetzt";
				str2 = "bitte einsteigen";
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
