#include <AccelStepper.h>

// ----- Nastavenie motorov -----
#define motorInterfaceType 1
const int dirPinY = 2, stepPinY = 3;
const int dirPinX = 4, stepPinX = 5;
const int dirPinZ = 6, stepPinZ = 7;

AccelStepper stepperX(motorInterfaceType, stepPinX, dirPinX);
AccelStepper stepperY(motorInterfaceType, stepPinY, dirPinY);
AccelStepper stepperZ(motorInterfaceType, stepPinZ, dirPinZ);

// ----- Enkodéry (simulované analógové) -----
const int encoderPinZ = A0;
const int encoderPinX = A1;
const int encoderPinY = A2;

// ----- Kalibračné offsety enkodérov -----
const float encoderOffsetX = 151 ;   // <- nastav podľa tvojej domácej polohy
const float encoderOffsetY = 294;
//const float encoderOffsetY = 0;
const float encoderOffsetZ = 75;

// ----- Konštanty -----
const float stepsPerRevolution = 200.0; // prispôsobiť tvojmu motoru / prevodovke
const float degreesPerStep = 360.0 / stepsPerRevolution;

// ----- Pozície -----
float targetAngleX = 0;
float targetAngleY = 0;
float targetAngleZ = 0;

// ----- Pomocné premenné -----
String inputString = "";

void setup() {
  Serial.begin(115200);

  stepperX.setMaxSpeed(100); stepperX.setAcceleration(50);
  stepperY.setMaxSpeed(100); stepperY.setAcceleration(50);
  stepperZ.setMaxSpeed(100); stepperZ.setAcceleration(50);

  // --- Inicializácia podľa enkodérov ---
  float angleX = readEncoder(encoderPinX, encoderOffsetX);
  float angleY = readEncoder(encoderPinY, encoderOffsetY);
  float angleZ = readEncoder(encoderPinZ, encoderOffsetZ);

  long stepsX = angleToSteps(angleX);
  long stepsY = angleToSteps(angleY);
  long stepsZ = angleToSteps(angleZ);

  stepperX.setCurrentPosition(stepsX);
  stepperY.setCurrentPosition(stepsY);
  stepperZ.setCurrentPosition(stepsZ);

  Serial.println("Inicializácia pozície podľa enkodérov:");
  Serial.print("X = "); Serial.print(angleX, 1); Serial.print("° / steps: "); Serial.println(stepsX);
  Serial.print("Y = "); Serial.print(angleY, 1); Serial.print("° / steps: "); Serial.println(stepsY);
  Serial.print("Z = "); Serial.print(angleZ, 1); Serial.print("° / steps: "); Serial.println(stepsZ);
}

void loop() {
  // Nezávislé ovládanie motorov
  stepperX.run();
  stepperY.run();
  stepperZ.run();

  // Spracovanie sériového vstupu
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        parseFullCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }

  // Na ukážku: čítanie enkodérov
  float angleX = readEncoder(encoderPinX, encoderOffsetX);
  float angleY = readEncoder(encoderPinY, encoderOffsetY);
  float angleZ = readEncoder(encoderPinZ, encoderOffsetZ);

  // Odoslanie spätných hodnôt – voliteľné
  Serial.print("ENC_X: "); Serial.print(angleX, 1);
  Serial.print(" | ENC_Y: "); Serial.print(angleY, 1);
  Serial.print(" | ENC_Z: "); Serial.println(angleZ, 1);
  delay(100); // aktualizačná perióda
}

// ----- Funkcia na čítanie analógového enkodéra s offsetom -----
//float readEncoder(int pin, float offset) {
//  int value = analogRead(pin);
//  int angle12bit = map(value, 0, 1023, 0, 4095);
//  float angleDeg = (angle12bit / 4095.0) * 360.0;

  // Aplikuj offset
//  angleDeg -= offset;

  // Oprav rozsah na 0–360
//  if (angleDeg < 0) angleDeg += 360.0;
//  if (angleDeg >= 360.0) angleDeg -= 360.0;

//  return angleDeg;
//}


// ----- Funkcia na čítanie analógového enkodéra s offsetom -----
// výsledok: -180° … +180°
float readEncoder(int pin, float offset) {
  int value      = analogRead(pin);
  float angleDeg = map(value, 0, 1023, 0, 4095) * 360.0 / 4095.0; // 0–360°
  angleDeg      -= offset;                                         // zohľadni offset

  // PREPIS: zabaľ do intervalu -180..180
  if (angleDeg > 180.0)  angleDeg -= 360.0;   // 270° -> -90°
  if (angleDeg <= -180.0) angleDeg += 360.0;  // -190° -> 170°

  return angleDeg;                            // vracia priamo -180..180
}


// ----- Funkcia na spracovanie príkazu (napr. X45;Y-30;Z10;) -----
void parseFullCommand(String cmd) {
  cmd.trim();
  float angles[3] = {0, 0, 0}; // X, Y, Z
  char axes[3] = {'X', 'Y', 'Z'};

  int idx = 0;
  while (cmd.length() > 0 && idx < 3) {
    int sep = cmd.indexOf(';');
    String part = (sep == -1) ? cmd : cmd.substring(0, sep);

    char axis = toupper(part.charAt(0));
    float angle = part.substring(1).toFloat();
    angle = constrain(angle, -180, 180);

    switch (axis) {
      case 'X': angles[0] = angle; break;
      case 'Y': angles[1] = angle; break;
      case 'Z': angles[2] = angle; break;
      default: break;
    }

    if (sep == -1) break;
    cmd = cmd.substring(sep + 1);
    idx++;
  }

  moveToAngle('X', angles[0], stepperX);
  moveToAngle('Y', angles[1], stepperY);
  moveToAngle('Z', angles[2], stepperZ);

  Serial.println("Command received and moving...");
}

// ----- Pohyb na absolútny uhol -----
void moveToAngle(char axis, float targetAngle, AccelStepper &stepper) {
  float steps = targetAngle / degreesPerStep;
  steps = constrain(steps, -stepsPerRevolution / 2, stepsPerRevolution / 2); // -180° až 180°
  stepper.moveTo(steps);
  Serial.print("Motor "); Serial.print(axis); Serial.print(" -> "); Serial.print(targetAngle); Serial.println("°");
}

long angleToSteps(float angleDeg) {
  // Pre rozsah -180 až +180:
  if (angleDeg > 180.0) angleDeg -= 360.0;
  return (long)(angleDeg / degreesPerStep);
}