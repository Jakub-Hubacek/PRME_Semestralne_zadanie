/***************************************************************************
 *  3-osý polohovací systém – úplný, zjednotený kód                       *
 *  – analógové „enkodéry“ 0-360° → -180°…+180°                           *
 *  – príkazy typu  X45;Y-30;Z10; (bodka alebo čiarka v desatinnom čísle) *
 *  – možnosť invertovať smer každej osi                                  *
 *  – neovplyvnené osi sa pri príkaze nehýbu                              *
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

// ofsety enkodérov (nastav podľa domácej polohy)
const float encoderOffsetX = 151;
const float encoderOffsetY = 294;
const float encoderOffsetZ = 75;

// parametre krokových motorov
const float stepsPerRevolution = 200.0;             // prispôsob podľa motora
const float degreesPerStep     = 360.0 / stepsPerRevolution;

// invertovanie smeru osi (1 = pôvodný, -1 = opačný)
const int8_t dirSignX = -1;
const int8_t dirSignY =  1;
const int8_t dirSignZ = -1;

// ───────────────────────────── INŠTANCIE ────────────────────────────────
AccelStepper stepperX(motorInterfaceType, stepPinX, dirPinX);
AccelStepper stepperY(motorInterfaceType, stepPinY, dirPinY);
AccelStepper stepperZ(motorInterfaceType, stepPinZ, dirPinZ);

String inputString;                       // buffer sériového príkazu

// ─────────────────────────────  SETUP  ──────────────────────────────────
void setup() {
  Serial.begin(115200);

  // rýchlosť a akcelerácia – uprav, ak motory nestíhajú / sú pomalé
  stepperX.setMaxSpeed(100);  stepperX.setAcceleration(50);
  stepperY.setMaxSpeed(400);  stepperY.setAcceleration(200);
  stepperZ.setMaxSpeed(100);  stepperZ.setAcceleration(50);

  // vyčítaj reálne uhly z enkodérov a nastav softvérové nuly
  float angleX = readEncoder(encoderPinX, encoderOffsetX);
  float angleY = readEncoder(encoderPinY, encoderOffsetY);
  float angleZ = readEncoder(encoderPinZ, encoderOffsetZ);

  stepperX.setCurrentPosition(angleToSteps(angleX));
  stepperY.setCurrentPosition(angleToSteps(angleY));
  stepperZ.setCurrentPosition(angleToSteps(angleZ));

  Serial.println(F("Inicializácia dokončená:"));
  Serial.print (F("  X = ")); Serial.print(angleX,1); Serial.print(F("°  steps: ")); Serial.println(stepperX.currentPosition());
  Serial.print (F("  Y = ")); Serial.print(angleY,1); Serial.print(F("°  steps: ")); Serial.println(stepperY.currentPosition());
  Serial.print (F("  Z = ")); Serial.print(angleZ,1); Serial.print(F("°  steps: ")); Serial.println(stepperZ.currentPosition());
}

// ─────────────────────────────  LOOP  ───────────────────────────────────
void loop() {
  // nezávislé obsluhovanie motorov
  stepperX.run();
  stepperY.run();
  stepperZ.run();

  // čítanie sériového príkazu
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {      // koniec riadku
      if (inputString.length()) {
        parseFullCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }

  // spätná väzba (10 Hz) – voliteľné
  static uint32_t t = millis();
  if (millis() - t >= 100) {
    t = millis();
    Serial.print(F("ENC_X: ")); Serial.print(readEncoder(encoderPinX, encoderOffsetX),1);
    Serial.print(F(" | ENC_Y: ")); Serial.print(readEncoder(encoderPinY, encoderOffsetY),1);
    Serial.print(F(" | ENC_Z: ")); Serial.println(readEncoder(encoderPinZ, encoderOffsetZ),1);
  }
}

// ────────────────────── POMOCNÉ FUNKCIE ────────────────────────

// načítaj enkodér a vráť uhol -180 … +180 °
float readEncoder(int pin, float offset) {
  int   value    = analogRead(pin);
  float angleDeg = map(value, 0, 1023, 0, 4095) * 360.0 / 4095.0; // 0-360°
  angleDeg -= offset;                                             // odrátaj offset
  if (angleDeg >  180.0) angleDeg -= 360.0;                       // prebaliť
  if (angleDeg <=-180.0) angleDeg += 360.0;
  return angleDeg;
}

// prevod uhlov na kroky
inline long angleToSteps(float angleDeg) {
  return (long)(angleDeg / degreesPerStep);
}

// pohnúť konkrétnou osou na absolútny uhol
void moveToAngle(char axis, float targetAngle, AccelStepper &stepper) {
  int8_t sign = 1;
  switch (axis) {                         // aplikuj prípadné invertovanie
    case 'X': sign = dirSignX; break;
    case 'Y': sign = dirSignY; break;
    case 'Z': sign = dirSignZ; break;
  }
  long steps = (long)(sign * targetAngle / degreesPerStep);
  steps = constrain(steps, -(long)(stepsPerRevolution/2), (long)(stepsPerRevolution/2)); // ±180°
  stepper.moveTo(steps);

  Serial.print(F("Motor ")); Serial.print(axis);
  Serial.print(F(" → "));    Serial.print(targetAngle); Serial.println(F("°"));
}

// rozparsuj príkaz štýlu  „X-30.5;Y45;Z0;“
void parseFullCommand(String cmd) {
  cmd.trim();
  cmd.replace(',', '.');                    // ak LabVIEW posiela des. čiarku

  float newAngle[3] = {NAN, NAN, NAN};      // X,Y,Z (NAN = os sa nehýbe)

  while (cmd.length()) {
    int sep  = cmd.indexOf(';');
    String part = (sep == -1) ? cmd : cmd.substring(0, sep);
    part.trim();

    if (part.length() > 1) {
      char  axis = toupper(part[0]);
      float ang  = part.substring(1).toFloat();
      if (ang >  180.0) ang -= 360.0;       // prebaliť
      if (ang <=-180.0) ang += 360.0;

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

  Serial.println(F("Príkaz spracovaný."));
}