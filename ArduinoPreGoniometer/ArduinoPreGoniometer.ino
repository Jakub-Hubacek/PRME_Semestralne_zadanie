/***************************************************************************
 *  3-osý polohovací systém – aktualizovaný kód                           *
 *  – Softvérové nulovanie podľa aktuálnej pozície enkodérov              *
 *  – Ochrana proti preskakovaniu krokov                                  *
 *  – Formát príkazov: X45;Y-30.5;Z10;                                     *
 ***************************************************************************/

#include <AccelStepper.h>

// ─────────────────────────── PINY A KONŠTANTY ───────────────────────────
#define motorInterfaceType 1

// motorové piny
const int dirPinY  = 2, stepPinY  = 3;
const int dirPinX  = 4, stepPinX  = 5;
const int dirPinZ  = 6, stepPinZ  = 7;

// analógové enkodéry
const int encoderPinZ = A0;
const int encoderPinX = A1;
const int encoderPinY = A2;

// parametre motorov
const float stepsPerRevolution = 200.0;
const float degreesPerStep     = 360.0 / stepsPerRevolution;

// smer motorov (1 = pôvodný, -1 = opačný)
const int8_t dirSignX = -1;
const int8_t dirSignY = -1;
const int8_t dirSignZ = -1;

// softvérové offsety z enkodérov
float homeOffsetX = 0;
float homeOffsetY = 0;
float homeOffsetZ = 0;

// ───────────────────────────── INŠTANCIE ────────────────────────────────
AccelStepper stepperX(motorInterfaceType, stepPinX, dirPinX);
AccelStepper stepperY(motorInterfaceType, stepPinY, dirPinY);
AccelStepper stepperZ(motorInterfaceType, stepPinZ, dirPinZ);

String inputString;
bool state = false; // blikacia LED

// ─────────────────────────────  SETUP  ──────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // motorové parametre
  stepperX.setMaxSpeed(50);  stepperX.setAcceleration(40);
  stepperY.setMaxSpeed(50);  stepperY.setAcceleration(40);
  stepperZ.setMaxSpeed(50);  stepperZ.setAcceleration(40);

  // načítaj aktuálnu (domácu) pozíciu z enkodérov
  float rawX = readRawEncoder(encoderPinX);
  float rawY = readRawEncoder(encoderPinY);
  float rawZ = readRawEncoder(encoderPinZ);

  // nastav ako softvérové offsety
  homeOffsetX = rawX;
  homeOffsetY = rawY;
  homeOffsetZ = rawZ;

  // nastav softvérové nuly pre motory
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  Serial.println(F("Inicializácia dokončená:"));
  //Serial.print (F("  X home: ")); Serial.println(homeOffsetX,1);
  //Serial.print (F("  Y home: ")); Serial.println(homeOffsetY,1);
 // Serial.print (F("  Z home: ")); Serial.println(homeOffsetZ,1);
}

// ─────────────────────────────  LOOP  ───────────────────────────────────
void loop() {
  stepperX.run();
  stepperY.run();
  stepperZ.run();

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length()) {
        parseFullCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }

  // spätná väzba + kontrola preskoku krokov
  static uint32_t t = millis();
  if (millis() - t >= 100) {
    t = millis();
    float encX = readEncoder(encoderPinX, homeOffsetX);
    float encY = readEncoder(encoderPinY, homeOffsetY);
    float encZ = readEncoder(encoderPinZ, homeOffsetZ);

    Serial.print(F("ENC_X: ")); Serial.print(encX,1);
    Serial.print(F(" | ENC_Y: ")); Serial.print(encY,1);
    Serial.print(F(" | ENC_Z: ")); Serial.println(encZ,1);

    checkSlip(stepperX, encX, 'X');
    checkSlip(stepperY, encY, 'Y');
    checkSlip(stepperZ, encZ, 'Z');
  }
}

// ────────────────────────── POMOCNÉ FUNKCIE ─────────────────────────────

// načítanie surového uhla z enkodéra
float readRawEncoder(int pin) {
  int value = analogRead(pin);
  float angle = map(value, 0, 1023, 0, 4095) * 360.0 / 4095.0;
  return wrapAngle(angle);
}

// načítanie uhla s home offsetom
float readEncoder(int pin, float offset) {
  float angle = readRawEncoder(pin) - offset;
  return wrapAngle(angle);
}

// zabalenie uhla do rozsahu -180° až +180°
float wrapAngle(float a) {
  while (a > 180.0)  a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// konverzia uhla na počet krokov
inline long angleToSteps(float angleDeg) {
  return (long)(angleDeg / degreesPerStep);
}

// pohnutie motorom na daný absolútny uhol
void moveToAngle(char axis, float targetAngle, AccelStepper &stepper) {
  int8_t sign = 1;
  switch (axis) {
    case 'X': sign = dirSignX; break;
    case 'Y': sign = dirSignY; break;
    case 'Z': sign = dirSignZ; break;
  }
  long steps = (long)(sign * targetAngle / degreesPerStep);
  steps = constrain(steps, -(long)(stepsPerRevolution/2), (long)(stepsPerRevolution/2)); // ±180°
  stepper.moveTo(steps);

  //Serial.print(F("Motor ")); Serial.print(axis);
  //Serial.print(F(" → "));    Serial.print(targetAngle); Serial.println(F("°"));
}

// kontrola rozdielu medzi encoderom a motorom (ochrana proti preskočeniu krokov)
void checkSlip(AccelStepper &stepper, float encoderAngle, char axis) {
  int8_t sign = (axis == 'X') ? dirSignX : (axis == 'Y') ? dirSignY : dirSignZ;
  float motorAngle = stepper.currentPosition() * degreesPerStep * sign;
  float diff = abs(encoderAngle - motorAngle);
  if (diff > 5.0) { // nastaviteľná citlivosť
    //Serial.print(F("!!! UPOZORNENIE: Preskočenie osi "));
    //Serial.print(axis);
    //Serial.print(F(" o ")); Serial.print(diff,1); Serial.println(F("°"));
  }
}

// rozparsovanie príkazu typu "X45.3;Y-10;Z90;"
void parseFullCommand(String cmd) {
  cmd.trim();
  cmd.replace(',', '.');

  float newAngle[3] = {NAN, NAN, NAN};

  while (cmd.length()) {
    int sep  = cmd.indexOf(';');
    String part = (sep == -1) ? cmd : cmd.substring(0, sep);
    part.trim();

    if (part.length() > 1) {
      char  axis = toupper(part[0]);
      float ang  = part.substring(1).toFloat();
      ang = wrapAngle(ang);

      switch (axis) {
        case 'X': newAngle[0] = ang; break;
        case 'Y': newAngle[1] = ang; break;
        case 'Z': newAngle[2] = ang; break;
      }
    }

    if (sep == -1) break;
    cmd = cmd.substring(sep + 1);
  }

  if (!isnan(newAngle[0])) moveToAngle('X', newAngle[0], stepperX);
  if (!isnan(newAngle[1])) moveToAngle('Y', newAngle[1], stepperY);
  if (!isnan(newAngle[2])) moveToAngle('Z', newAngle[2], stepperZ);

  //Serial.println(F("Príkaz spracovaný."));
  digitalWrite(LED_BUILTIN, state);
  state = !state;
}
