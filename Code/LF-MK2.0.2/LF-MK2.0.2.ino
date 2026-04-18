#include <IRremote.hpp>
#define DEFAULT_SPEED 255  //rychlost robota
// Motor A - Levý
const int PWM1 = 11;
const int DIR1 = 10;

// Motor B - Pravý
const int PWM2 = 6;
const int DIR2 = 7;

// IR Sensory
#define RECV_PIN A0
#define U1 A1
#define U2 A2
#define U3 A3
#define U4 A4
#define U5 A5
#define U6 A6
#define U7 A7

int IR1, IR2, IR3, IR4, IR5, IR6, IR7;

const int buttonPin = 3;
int buttonState;
int lastButtonState = LOW;
bool isOn = false;

const int buttonPin2 = 2;
int buttonState2;
int lastButtonState2 = LOW;
bool isOn2 = false;

bool irState = HIGH;
bool lastIrState = HIGH;
unsigned long lastIrTrigger = 0;

int LFspeed, Lspeed, Rspeed;
int lastError = 0;
float Kp = 3;   // Proporcionální konstanta (ladit podle potřeby)
float Kd = 14;  // Derivační konstanta (tlumí cukání)
int Max_speed = 100;

int sensorPins[] = { U1, U2, U3, U4, U5, U6, U7 };

int minValues[7], maxValues[7];
int threshold[7] = { 700, 700, 700, 700, 700, 700, 700 };  //předurčená hodnota 700

void setup() {
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  pinMode(U7, INPUT);
  pinMode(U6, INPUT);
  pinMode(U5, INPUT);
  pinMode(U4, INPUT);
  pinMode(U3, INPUT);
  pinMode(U2, INPUT);
  pinMode(U1, INPUT);

  pinMode(buttonPin, INPUT);

  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);

  Serial.begin(24000000);
}

void loop() {
  checkPowerButton();
  checkCalibrationButton();
  checkIRRemote();

  if (isOn2) {
    calibrate();
  } else if (isOn) {
    readAllSensors();
    if (Max_speed < DEFAULT_SPEED) {
      Max_speed += 1;
      delay(1);
    } else if (Max_speed >= DEFAULT_SPEED) {
      Max_speed = DEFAULT_SPEED;
    }
    linefollow();
  } else {
    Max_speed = 100;
    stop();
  }
}

void readAllSensors() {
  IR1 = analogRead(U1) > threshold[0] ? 1 : 0;
  IR2 = analogRead(U2) > threshold[1] ? 1 : 0;
  IR3 = analogRead(U3) > threshold[2] ? 1 : 0;
  IR4 = analogRead(U4) > threshold[3] ? 1 : 0;
  IR5 = analogRead(U5) > threshold[4] ? 1 : 0;
  IR6 = analogRead(U6) > threshold[5] ? 1 : 0;
  IR7 = analogRead(U7) > threshold[6] ? 1 : 0;
}

void linefollow() {
  isOn2 = false;
  int error = (65 * IR1 + 30 * IR2 + 15 * IR3 - 15 * IR5 - 30 * IR6 - 65 * IR7);
  int derivative = error - lastError;  // Výpočet změny chyby
  int adjustment = (Kp * error) + (Kd * derivative);

  if (IR1 == 0 && IR2 == 0 && IR3 == 0 && IR4 == 0 && IR5 == 0 && IR6 == 0 && IR7 == 0) {
    if (lastError < 0) sharpLeft();        // Čára zmizela vlevo
    else if (lastError > 0) sharpRight();  // Čára zmizela vpravo
    return;
  }

  moveCustom(Max_speed + adjustment, Max_speed - adjustment);
  lastError = error;  // Uložení chyby pro příští smyčku
}

void sharpLeft() {
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, DEFAULT_SPEED);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, DEFAULT_SPEED);
}

void sharpRight() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, DEFAULT_SPEED);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM2, DEFAULT_SPEED);
}

void stop() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
}

void moveCustom(int L, int R) {
  digitalWrite(DIR1, L >= 0 ? HIGH : LOW);
  analogWrite(PWM1, constrain(abs(L), 0, 255));
  digitalWrite(DIR2, R >= 0 ? HIGH : LOW);
  analogWrite(PWM2, constrain(abs(R), 0, 255));
}

void calibrate() {
  isOn = false;
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(sensorPins[i]);
    maxValues[i] = analogRead(sensorPins[i]);
  }
  for (int i = 0; i < 3000; i++) {
    moveCustom(150, -150);
    for (int j = 0; j < 7; j++) {
      int value = analogRead(sensorPins[j]);
      if (value < minValues[j]) minValues[j] = value;
      if (value > maxValues[j]) maxValues[j] = value;
    }
    delay(1);
  }
  for (int i = 0; i < 7; i++) {
    threshold[i] = 7 * (minValues[i] + maxValues[i]) / 10;
  }
  stop();
  delay(100);
  isOn2 = false;
}

void checkPowerButton() {
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      isOn = !isOn;
    }
    delay(50);
  }
  lastButtonState = buttonState;
}

void checkCalibrationButton() {
  buttonState2 = digitalRead(buttonPin2);
  if (buttonState2 != lastButtonState2) {
    if (buttonState2 == LOW) {
      isOn2 = !isOn2;
    }
    delay(50);
  }
  lastButtonState2 = buttonState2;
}

void checkIRRemote() {
  if (IrReceiver.decode()) {
    uint32_t receivedCode = IrReceiver.decodedIRData.decodedRawData;
    if (receivedCode == 0xBA44FF00 || receivedCode == 0xE817BF40) {
      isOn = !isOn;
    }
    IrReceiver.resume();
  }
}
