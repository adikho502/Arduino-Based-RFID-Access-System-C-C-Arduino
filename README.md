# Arduino-Based-RFID-Access-System-C-C-Arduino
Performed DFMEA analysis on eight subsystems using AIAG methodology to identify and mitigate high-risk failure modes. Developed a state machine–driven microcontroller security system integrating RFID authentication, ultrasonic sensing, dual-servo control, and LCD/joystick interface for real-time monitoring and alerts.
 
 
 
 //Project: Safe With A Built-In Security System
  //Author: Andrew Dikho, Aidan Konja, Luke Zora, Lauren Petterle
  //Date: 04/21/2025

  //RFID logic adapted from MFRC522 library example "DumpInfo" by GithubCommunity
  //Modifications: Added access control with 2 keycards, servo activation, sound playback, and state machine management



#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <LiquidCrystal.h>

#define SS_PIN 53
#define RST_PIN 11
#define TRIG_PIN 25
#define ECHO_PIN 23
#define pitches.h

const int G1 = 2, G2 = 3, G3 = 4, Y1 = 5, Y2 = 6, Y3 = 7, R1 = 8, R2 = 9, R3 = 10;
const int buzz = A2;
const int rs = 33, en = 32, d4 = 31, d5 = 43, d6 = 41, d7 = 39;
const int joystickbutton = 49;

int distance = 36; // Assumes Max distance
int state = 1; // Assumes initial state = 1
int lastState = -1; // Tracks previous state to avoid flickering on LCD
bool servoposition = false; // Inside latchopen() and joystick() to avoid code running more than it should
bool lastPressed = false; // Tracks joysticks state, Low or High and debounces button
unsigned long lastBlinkTime = 0; // Stores last time the LED's changed state
bool ledsOn = false; //Stores LED state on or off for FlashingLED()
bool initialized = false; //Stores a variable in state 3 to keep from flickering on LCD and unneccesary movements on Servo

MFRC522 mfrc522(SS_PIN, RST_PIN);
Servo myServo;
Servo myServo2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(buzz, OUTPUT);
  pinMode(joystickbutton, INPUT_PULLUP);

  for (int i = 2; i <= 10; i++) pinMode(i, OUTPUT); // Sets all LED's to OUTPUTS

  SPI.begin();               // Initializes RFID
  mfrc522.PCD_Init();        // RFID
  myServo.attach(12);        // Distance Srvo
  myServo.write(0);          // Set To Default Distance
  myServo2.attach(13);       // Latch Servo
  myServo2.write(0);         // Sets Default

  Serial.println("System Initialized"); //Stores that initial conditions are set on Serial Monitor
}

void loop() {
  distance = sensprox();

  switch (state) {                 //Enables State Machine
    case 1: state1(); break;       // Runs state1() function once and break; prevents from running another case
    case 2: state2(); break;       // Runs state2() function once and break; prevents from running another case
    case 3: state3(); break;       // Runs state3() function once and break; prevents from running another case
  }

  joystick();

  if (state == 1) { //Runs along with state1() function
    lcd.clear();
    lcd.setCursor(0, 0);
    if (distance == 36) {          // If distance is equal or more than 36 inches, no distnace is shown
      lcd.print("NO MOTION");
      lcd.setCursor(0, 1);
      lcd.print("Distance: -- in ");
    } else {
      lcd.print("MOTION DETECTED"); // Once distance is less than 36 inches lcd updates to show accurate distance in inches
      lcd.setCursor(0, 1);
      lcd.print("Distance: ");
      lcd.print(distance);
      lcd.print(" in   ");
    }
  }

  if (state == 2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WARNING");
    lcd.setCursor(0, 1);
    lcd.print("SCAN TO DISARM");
    lastState = state;         // holds most accurate state reading
  }
}

void state1() {
  initialized = false;
  ledprox();
  buzzprox();
  gaugeprox();
  scan();

  if (state != lastState) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("NO MOTION");
    lastState = state;
  }
}

void state2() {
  flashingLED();
  buzzmaxfreq();
  gaugeproxmax();
  sensproxoff();
  latchopen();
}

void state3() {
  if (!initialized) {
    LEDoff();
    buzzeroff();
    gaugeproxdefault();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ACCESS GRANTED");
    lcd.setCursor(0, 1);
    lcd.print("Welcome!");
    delay(2000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press Joystick");
    lcd.setCursor(0, 1);
    lcd.print("To Reset Alarm");

    initialized = true;
  }
}

void scan() {
  static int closeCount = 0; //static holds number across loop iterations/ counts how many times the object is close <= 11
  static unsigned long lastScanTime = 0; // Holds last time scan checked the distance
  const unsigned long scanInterval = 150; // Minimum time between scans 150ms

  if (millis() - lastScanTime < scanInterval) return; // Scans every 150ms whether distance is <= 11 or not
  lastScanTime = millis();                            // Holds last scan time

  if (distance <= 11) {
    closeCount++;
    if (closeCount >= 3) {
      Serial.println("✅ State changed to 2");
      state = 2;
      closeCount = 0;
    }
  } else {
    closeCount = 0;
  }
}

int sensprox() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int newDist = (int)(duration * 0.0135 / 2);                  // Converts distance to inches
  if (duration == 0 || newDist < 1 || newDist > 36) return 36; // Duration - 0 (Nothing Detected) newDist < 0, too close, greater than 36, out of range
  return newDist;              // Will Return 36 if any of those statements are true, if not returns correct distance (newDist)
}
// Function adapted from MFRC522 "DumpInfo" example
// Modified to implement custom control and integrate with state machine 
void latchopen() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;

  String readUID = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) readUID += "0";
    readUID += String(mfrc522.uid.uidByte[i], HEX);
  }
  readUID.toUpperCase();

  Serial.print("Scanned UID: "); //Displays the correct keykard that is scanned
  Serial.println(readUID);

  if (readUID == "D7E42403" || readUID == "C37A762D") { // The 2 Keycards included in our Kit
    Serial.println("✅ Access Granted");
    myServo2.write(180);                                //Opens Latch
    delay(750);                                        
    play1upsound();                                     //Plays sound to signal access granted
    servoposition = true;                               // Holds state that the latch was open
    state = 3;                                          // State 3 is activated
  } else {
    Serial.println("❌ Access Denied");
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void joystick() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  bool currentlyPressed = digitalRead(joystickbutton) == LOW; //Comparing the button reading to LOW, Low == Button Press, High == Rest

  if (currentlyPressed && !lastPressed && millis() - lastDebounceTime > debounceDelay) { // ONLY runs when currentlyPressed is true 
    lastDebounceTime = millis(); //Records last time button was pressed                                                        //!last Pressed will always return true
                                 // lastdebounceTime will always be bigger than debouncedelay(50)
    if (state == 3) {
      Serial.println("🕹 Joystick Pressed → Resetting system");
      myServo2.write(0);
      servoposition = false;
      sensproxon();
      playMarioMusic();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("   Resetting");
      lcd.setCursor(0,1);
      lcd.print("   System...");
      delay(5000);
      state = 1;
      lastState = -1;
      initialized = false;

      digitalWrite(buzz, HIGH);
      delay(100);
      digitalWrite(buzz, LOW);
    }
  }

  lastPressed = currentlyPressed; //Resets back to current button state
}

void sensproxon() {
  pinMode(TRIG_PIN, OUTPUT); //Turns Sensor On
  pinMode(ECHO_PIN, INPUT);
}

void sensproxoff() {
  pinMode(TRIG_PIN, INPUT); //Disables this pin , stops from sending signals to read a distance from
  pinMode(ECHO_PIN, INPUT);
  distance = 36;
}

void ledprox() {      // Maps All LED's to a threshold to give a smooth flow with proximity
  analogWrite(G2, map(constrain(distance, 32, 36), 36, 32, 0, 255)); 
  analogWrite(G1, map(constrain(distance, 28, 32), 32, 28, 0, 255));
  analogWrite(G3, map(constrain(distance, 24, 28), 28, 24, 0, 255));
  analogWrite(Y1, map(constrain(distance, 20, 24), 24, 20, 0, 255));
  analogWrite(Y2, map(constrain(distance, 16, 20), 20, 16, 0, 255));
  analogWrite(Y3, map(constrain(distance, 12, 16), 16, 12, 0, 255));
  analogWrite(R1, map(constrain(distance, 8, 12), 12, 8, 0, 255));
  analogWrite(R2, map(constrain(distance, 4, 8), 8, 4, 0, 255));
  analogWrite(R3, map(constrain(distance, 1, 4), 4, 1, 0, 255));
}

void buzzprox() { //Maps Buzzer Based on Prox.
  if (distance >= 36) noTone(buzz);
  else tone(buzz, map(distance, 36, 1, 100, 2000));
}

void gaugeprox() { // Maps Gauge Servo based on Prox.
  int angle = map(distance, 36, 1, 180, 0);
  angle = constrain(angle, 0, 180);
  myServo.write(angle);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" : Servo angle: ");
  Serial.println(angle);
}

void flashingLED() { //Blinks the LED's every 500ms
  if (millis() - lastBlinkTime >= 500) {
    ledsOn = !ledsOn;
    for (int i = 2; i <= 10; i++) digitalWrite(i, ledsOn ? HIGH : LOW); // If ledson == false turn on LED's, else turn them off
    lastBlinkTime = millis();
  }
}

void buzzmaxfreq() { //Alarm noise in State 2
  tone(buzz, 2000);
}

void gaugeproxmax() { // Signaling "Danger" on the Gauge on Front of Box
  myServo.write(0);
}

void LEDoff() {
  for (int i = 2; i <= 10; i++) digitalWrite(i, LOW); //Turns LED's OFF
}

void buzzeroff() { // Turns Buzzer OFF
  noTone(buzz);
  digitalWrite(buzz, LOW);
}

void gaugeproxdefault() {
  myServo.write(180);
}

void playMarioMusic() {
  // Mario Theme Melody Code by TECHARENA, modified for Arduino Mega project
  // https://www.hackster.io/techarea98/super-mario-theme-song-with-piezo-buzzer-and-arduino-2cc461

  #define NOTE_E6 1319
  #define NOTE_G6 1568
  #define NOTE_A6 1760
  #define NOTE_AS6 1865
  #define NOTE_B6 1976
  #define NOTE_C7 2093
  #define NOTE_D7 2349
  #define NOTE_E7 2637
  #define NOTE_F7 2794
  #define NOTE_G7 3136
  #define NOTE_A7 3520

  int melodyPin = buzz;
  pinMode(melodyPin, OUTPUT);
  

  int melody[] = {
    NOTE_E7, NOTE_E7, 0, NOTE_E7, 0, NOTE_C7, NOTE_E7, 0,
    NOTE_G7, 0, 0, 0, NOTE_G6, 0, 0, 0,
    NOTE_C7, 0, 0, NOTE_G6, 0, 0, NOTE_E6, 0,
    0, NOTE_A6, 0, NOTE_B6, 0, NOTE_AS6, NOTE_A6, 0,
    NOTE_G6, NOTE_E7, NOTE_G7, NOTE_A7, 0, NOTE_F7, NOTE_G7, 0,
    NOTE_E7, 0, NOTE_C7, NOTE_D7, NOTE_B6, 0, 0,
    NOTE_C7, 0, 0, NOTE_G6, 0, 0, NOTE_E6, 0,
    0, NOTE_A6, 0, NOTE_B6, 0, NOTE_AS6, NOTE_A6, 0,
    NOTE_G6, NOTE_E7, NOTE_G7, NOTE_A7, 0, NOTE_F7, NOTE_G7, 0,
    NOTE_E7, 0, NOTE_C7, NOTE_D7, NOTE_B6, 0, 0
  };

  int tempo[] = {
    12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12,
    9, 9, 9, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12,
    9, 9, 9, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12
  };

  int size = sizeof(melody) / sizeof(melody[0]);

  for (int thisNote = 0; thisNote < size; thisNote++) {
    int noteDuration = 1000 / tempo[thisNote];
    int freq = melody[thisNote];

    if (freq == 0) {
      delay(noteDuration); // Rest
    } else {
      long delayValue = 1000000 / freq / 2;
      long numCycles = (long)freq * noteDuration / 1000;

      for (long i = 0; i < numCycles; i++) {
        digitalWrite(melodyPin, HIGH);
        delayMicroseconds(delayValue);
        digitalWrite(melodyPin, LOW);
        delayMicroseconds(delayValue);
      }
    }

    
    delay(noteDuration * 1.30);
    
  }
}



void play1upsound() {
 // Mario 1-Up Sound by Benjamin T. Perry, modified for Arduino Mega Project
 // https://bikeshedeffect.weebly.com/arduino-piezo-sounds.html

  // Play 1-up sound
  tone(buzz,NOTE_E6,125);
  delay(130);
  tone(buzz,NOTE_G6,125);
  delay(130);
  tone(buzz,NOTE_E7,125);
  delay(130);
  tone(buzz,NOTE_C7,125);
  delay(130);
  tone(buzz,NOTE_D7,125);
  delay(130);
  tone(buzz,NOTE_G7,125);
  delay(125);
  noTone(buzz);
}

