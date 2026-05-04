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

//#include <LiquidCrystal.h>
#include "MiniLCD.h" //(saves 500 bytes of EEPROM!)
#include "AudioFile.h"

// Play horn audio sample, requires 8 Ohm speaker instead of buzzer
#define USE_HORN 1
// Use a potentiometer as input instead of step +/- buttons
#define ANALOG_THROTTLE 1
// There's Arduino HAT format PCB with different PIN connections
#define ARDUINO_HAT 0

//Track Geometry
//PIKO curved track piece: outer length 155, inner length 150
constexpr int TRAIN_LEN=112; //locomotive, mm
constexpr int TRAIN_CLEN=TRAIN_LEN * 152 / 145; //Observable length in the curve, where sensor located
constexpr int TRACK_LEN=152*8+2*240+2*110+2*55; //1980mm outer loop length
constexpr int STA_DIST=152+240+110+4*152+240+55; //1450; //From sensor to Station platform 1, mm
constexpr int STA2_DIST = 1320 + 50;//From sensor to Station platform 2, mm, +compensate wire resistance
constexpr int WORLD_SCALE=160; // N-scale 1:160

//////////////////////////
// Params and fine-tunes
//////////////////////////
constexpr int MOTOR_CUTOFF=22 * 255 / 100; //%, train doesn't move at lower PWM duty
constexpr int MOTOR_MAX_REAL=55 * 255 / 100; //55%, at this power train moves 140km/h, which is MAX for 1970s DR Class V119 diesel loc
const int LCD_REFRESH = 1000 / 2; // 2Hz
// Motor operation range
const int MIN_PWM = -255; //Set to 0 if no relay installed
const int MAX_PWM = 255;
const int MAX_RAIL_CURRENT = 500; //mA
const int SENTINEL_RESISTOR = 487; //mOhm, R9 on the schematic

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
const int RAIL_CURRENT = A7; // short circuit detector

//IR beam 36KHz on Timer1, works only for TIMER1 (D9 pin)
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

// Global vars
bool sensorInstalled = false;
long detectedSpeed = 0;
unsigned long lastDetectionTS = 0;
unsigned railCurrent = 0; //Actually a mapped voltage  on a 0.5 Ohm resistor

#if ANALOG_THROTTLE
unsigned long lastThrottleUpdate = 0;
#endif

#if USE_HORN
volatile uint8_t volume = 0;
const uint16_t wavSize = sizeof(horn_short_11k_wav);
volatile uint16_t sampleIndex = 44; //raw data offset in wav
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
//LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
MiniLCD lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void setup() {
	lcd.begin(16, 2);
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

	analogReference(INTERNAL);
	pinMode(SPEED_HANDLE, INPUT); //this is connected to 0..5V divider
	pinMode(RAIL_CURRENT, INPUT); //this uses internal 1.1V reference

	if (digitalRead(BUTTON_1) == LOW && digitalRead(BUTTON_2) == LOW) {
		setupMode = true;
	}
	
	// Initialize traffic lights
	pinMode(RED_LED, OUTPUT);
	pinMode(GREEN_LED, OUTPUT);
	digitalWrite(RED_LED, LOW);
	digitalWrite(GREEN_LED, LOW);
	
	lcd.begin(16, 2);
	lcd.print(F("N Eisenbahn v0.2"));
	lcd.setCursor(0, 1);
	lcd.print(F("von Roman & Vad"));
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
		lcd.print(F("Ohne Sensor"));
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
		// playTone(400, 5);
		// tone(SPEAKER_PIN, 400, 5);
		// delay(5);
		// noTone(SPEAKER_PIN);
	}
	updateMotor();
	updateDisplay();
	updateTrafficLights();
	ensureSwitchPos();
	//Kurzschluss check
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
		monitorCurrent(100); //no train can sneak through in 0.1s
		while (millis() < start_ts + 12000) {//locomotive can't pass longer than 12s?
			monitorCurrent(10); //poll at ~100hz, sensor isn't too fast
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
		
		maxLocal = max(maxLocal, analogRead(RAIL_CURRENT));
		railCurrent = maxLocal;
		delayMicroseconds(10);
		
		//Emergency stop if exceeds limit
		long mV = map(maxLocal, 0, 1023, 0, 1100); //1.1v ref voltage
		if (mV * 1000 / SENTINEL_RESISTOR > MAX_RAIL_CURRENT) {
 			emergencyStop();
		}
	}

	
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
		monitorCurrent(5);
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

	analogReference(DEFAULT);
	int handle = analogRead(SPEED_HANDLE); //0..1023
	analogReference(INTERNAL);

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
	if (ts - lastRefresh > 20) {
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

// Since we don't have a feedback signal from rail switch
// we trigger same electro-magent to ensure good contact
void ensureSwitchPos() {
	static unsigned long lastRefresh = 0;
	long ts = millis();
	if (ts - lastRefresh > 2000 && pwmDuty != 0) {
		int pin = switchIsLeft ? SWITCH_LEFT_PIN : SWITCH_RIGHT_PIN;
		digitalWrite(pin, HIGH);
		monitorCurrent(5);
		digitalWrite(pin, LOW);
		lastRefresh = ts;
	}
}

void toggleSwitch() {
	switchIsLeft = !switchIsLeft;
	int pin = switchIsLeft ? SWITCH_LEFT_PIN : SWITCH_RIGHT_PIN;
	digitalWrite(pin, HIGH);
	monitorCurrent(40);
	digitalWrite(pin, LOW);
}

void updateDisplay() {
	static long lastRefresh = 0;
	long ts = millis();
	if (ts - lastRefresh > LCD_REFRESH) {
		lcd.setCursor(0, 0); //clear is slow
		lcd.print("Lst=");
		lcd.print(pwmDuty * 100 / 255);
		lcd.print("% ");
		lcd.print("I=");
		long mV = map(railCurrent, 0, 1023, 0, 1100);//0..1.1V
		lcd.print(mV * 1000 / SENTINEL_RESISTOR); // U/R
		lcd.print(F("mA  "));
		lcd.setCursor(0, 1);
		lcd.print(F("Gsw:"));
		if (detectedSpeed >= 0) {
			lcd.print(detectedSpeed);
			lcd.print(F("mm/s"));
			lcd.print(" ");
			lcd.print(detectedSpeed * 36 * 16 / 1000); // kmh: 3.6 * 160 / 1000 
		} else {
			lcd.print("messen...    ");
		}
		lastRefresh = ts;
	}
}

void whistleBlast(int totalDuration) {
	//Traffic whistle with an interrupt ball
	//Non-block impl is possible with TIMER2 interrupt (TODO)
	unsigned long start = millis();
	unsigned long now;
	
	while ((now = millis()) - start < totalDuration) {

		unsigned long progr = (now - start) * 100 / totalDuration;
		
		// Attack Phase (first 10%)
		if (progr < 10) {
			int freq = 2500 + progr * 30;   // rise from 2.5kHz to ~3.4kHz 
			tone(SPEAKER_PIN, freq);
			monitorCurrent(15 + (20 * (10 - progr))); // more interruptions early
			noTone(SPEAKER_PIN);
			monitorCurrent(10);
		}
		
		// Chaos Phase
		else {
			int freq = random(2600, 3000);
			tone(SPEAKER_PIN, freq);
			monitorCurrent(random(12, 12));
			noTone(SPEAKER_PIN);
			monitorCurrent(random(3, 6));
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
				monitorCurrent(200);
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
				pwmDuty = random(MOTOR_CUTOFF + 20, MOTOR_MAX_REAL - 20);
				// Simplified few-step acceleration
				for (int duty = MOTOR_CUTOFF; duty <= pwmDuty; duty+=4) {
					setMotorPower(duty);
					monitorCurrent(200);
				}
				setMotorPower(pwmDuty);
				checkTrainContact();
			break;
			case DemoDetectSpeed:
				//can take a while, maybe introduce timeout error
				//detectionTS is a moment when train nose hits the sensor
				while ((detectionTS = pollTrainSensor(true)) == 0) {
					monitorCurrent(1);
				};
				// program continues when whole train passes the sensor
				lcd.clear();
				lcd.print(F("x160:")); lcd.print(detectedSpeed*36*16/1000); lcd.print(F("km/h"));
				lcd.setCursor(0, 1);
				lcd.print(F("Gsw:")); lcd.print(detectedSpeed); lcd.print(F("mm/s"));
				platform2 = (loops % 4) == 0;
				
				if (platform2) toggleSwitch();
				else ensureSwitchPos();
				
			break;
			case DemoSlowdown:
			{
				// 1. Tuning Parameters
				long brakingDist = 500;      
				long brakeSoftness = 1;      
				
				// NEW: Tuning the "depth" of the curve. 
				// 0 = Linear, 1000 = Quadratic. Try 400-500 for a balanced feel.
				long quadraticStrength = 220; 

				// 2. Determine Target Platform
				int targetPlatform = platform2 ? STA2_DIST : STA_DIST;
				long distToStation = targetPlatform - brakingDist - TRAIN_LEN;

				if (detectedSpeed * 36 * 16 / 1000 > 99 && !platform2) { 
					distToStation += TRACK_LEN; 
				}

				// Cruise
				long cruiseTimeMs = (distToStation * 1000L) / detectedSpeed;
				monitorCurrent(cruiseTimeMs);

				// 3. Physics Setup
				long initialSpeed = (long)detectedSpeed;
				long decel_milli = (initialSpeed * initialSpeed * 1000L) / (2L * brakingDist * brakeSoftness);
				if (decel_milli == 0) decel_milli = 1; 

				long totalBrakeTimeMs = (initialSpeed * 1000000L) / decel_milli;

				// 4. Setup Display/LEDs
				digitalWrite(GREEN_LED, LOW);
				digitalWrite(RED_LED, HIGH);
				lcd.clear();
				lcd.print(F("Ankunft.."));
				lcd.setCursor(0, 1);
				lcd.print(F("Gleis "));
				lcd.print(platform2 ? "2" : "1");

				int startPWM = pwmDuty;
				int minPWM = MOTOR_CUTOFF - 5;
				unsigned long brakeStartTime = millis();

				// 5. braking Loop
				while (millis() - brakeStartTime < totalBrakeTimeMs) {
					unsigned long elapsedMs = millis() - brakeStartTime;

					// Calculate Current Speed
					long speedReduction = (decel_milli * elapsedMs) / 1000000L;
					long currentSpeed = initialSpeed - speedReduction;
					if (currentSpeed < 0) currentSpeed = 0;

					// Speed to PWM mapping (Linear + Quadratic)
					int currentPWM = minPWM;
					if (initialSpeed > 0) {
						long pwmRange = (long)startPWM - minPWM;
						
						// Calculate Linear Ratio (0 to 1000)
						long ratioLin = (currentSpeed * 1000L) / initialSpeed;
						
						// Calculate Quadratic Ratio (0 to 1000)
						// (ratio^2 / 1000) keeps it in the 0-1000 range
						long ratioQuad = (ratioLin * ratioLin) / 1000L;

						// Blend them: Result = (Linear * (1-strength)) + (Quadratic * strength)
						long ratioBlended = (ratioLin * (1000 - quadraticStrength) + ratioQuad * quadraticStrength) / 1000;
						
						currentPWM = minPWM + (int)((pwmRange * ratioBlended) / 1000);
					}

					// Ensure PWM is in motor operational range
					if (currentPWM > startPWM) currentPWM = startPWM;
					if (currentPWM < minPWM) currentPWM = minPWM;

					setMotorPower(currentPWM);
					monitorCurrent(50); 
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
					ensureSwitchPos();
					setMotorReverse(true);
					setMotorPower(35*255/100);
					checkTrainContact();
					while (pollTrainSensor(true) == 0) {
						monitorCurrent(50);
					}
					//At this moment last cart has passed the sensor
					//TRACK_LEN-STA_DIST to go
					//Cart is 1.5 shorter, so
					int speed = detectedSpeed * 2 / 3;
					long wait = ((long)TRACK_LEN-STA_DIST+TRAIN_LEN) * 1000 / speed;
					
					monitorCurrent(wait);
					
					setMotorPower(MOTOR_CUTOFF+10);
					monitorCurrent(500);
					setMotorPower(0);
					toggleSwitch();
					monitorCurrent(200);
					setMotorReverse(false);
				}
			break;
			case DemoStationWait:
					//  0123456789ABCDEF
				str1 = "Abfahrt jetzt";
				str2 = "bitte einsteigen";
				ensureSwitchPos();
			break;

			case DemoEnd:
				demoMode = false;
			break;	
		}

		if (str1) {
			lcd.clear();
			lcd.print(str1); //Check if println or \r\n works
			lcd.setCursor(0, 1);
			lcd.print(str2);
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

void checkTrainContact() {
	monitorCurrent(5);
	if (railCurrent == 0) {
		lcd.clear();
		lcd.print(F("Lokkontakt?"));
		while (railCurrent == 0) {
			monitorCurrent(500);
		}
		lcd.clear();
		lcd.print("klar");
	}
}

void emergencyStop() {
	digitalWrite(IR_BEAM, LOW);
	digitalWrite(MOTOR_PWM_PIN, LOW);
	delay(1);
	digitalWrite(MOTOR_RELAY_PIN, LOW);
	lcd.clear();
	lcd.print(F("Ausfall:"));
	long mV = map(railCurrent, 0, 1023, 0, 1100);//0..1.1V
	lcd.print(mV * 1000 / SENTINEL_RESISTOR); // U/R
	lcd.print(F("mA "));
	lcd.setCursor(0, 1);
	lcd.print(F("Kurzschluss"));
	exit(0);
}

#if USE_HORN
// Hardcode for 11KHz sample rate, 8-bit uint_8 samples
void playHorn() {
	sampleIndex = 44;
	cli();
	//Squeeze max out of TIMER2, fast PWM
	TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2B1); //62.5kHz
  	TCCR2B = (1 << CS20); // prescaler = 1

  	// overflow interrupt
  	TIMSK2 = (1 << TOIE2);
	sei();
}

ISR(TIMER2_OVF_vect) {
	static uint8_t divider = 0;
	// 62.5kHz / 6 = 10.4kHz (almost 11kHz)
  	divider++;
  	if (divider < 6) return; // div by 8 will work for 8kHz wavs
  	divider = 0;

	uint8_t sample = pgm_read_byte(&horn_short_11k_wav[sampleIndex++]);
	OCR2B = sample; //Simplest. TODO: try volume ramp-up and linear interpolation at 22kHz, dither

	if (sampleIndex > wavSize) {
		//unset overflow interrupt
		TIMSK2 &= ~(1 << TOIE2);
	}
}
#endif
