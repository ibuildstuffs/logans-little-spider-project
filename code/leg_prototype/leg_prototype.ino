#include <Servo.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Pin assignment.
const byte COXA_PIN = 8;
const byte FEMUR_PIN = 9;
const byte TIBIA_PIN = 10;

// Leg dimensions in millimeters. Measure pivot-to-pivot for each segment.
const float COXA_LENGTH = 74.97f;
const float FEMUR_LENGTH = 150.0f;
const float TIBIA_LENGTH = 284.4615f;

// Servo command mapping. These are raw Servo.write() degree offsets.
// Your calibrated servos use 0 = left, 90 = center, 180 = right.
const float COXA_ZERO_DEG = 90.0f;
const float COXA_DIRECTION = -1.0f;
const float FEMUR_ZERO_DEG = 90.0f;
const float FEMUR_DIRECTION = -1.0f;
const float TIBIA_ZERO_DEG = 90.0f;
const float TIBIA_DIRECTION = -1.0f;

const unsigned long DEFAULT_MOVE_MS = 400;
const unsigned long STANCE_MS = 650;
const unsigned long SWING_MS = 580;
const unsigned long APPROACH_MS = 450;
const float SWING_HEIGHT = 100.0f;

struct Position {
  float x;
  float y;
  float z;
};

// x is outward from the body, y is forward/backward, and positive z is up.
const Position HOME = {450.0f, 0.0f, -30.0f};
const Position GAIT_FRONT = {450.0f, 60.0f, -30.0f};
const Position GAIT_REAR = {450.0f, -60.0f, -30.0f};

enum MotionMode {
  IDLE,
  MANUAL_MOVE,
  WALK_CYCLE
};

Servo coxa;
Servo femur;
Servo tibia;

Position currentPosition = HOME;
Position segmentStart = HOME;
Position segmentTarget = HOME;
unsigned long segmentStartTime = 0;
unsigned long segmentDuration = 0;
bool segmentActive = false;
bool segmentIsParabolicSwing = false;
MotionMode motionMode = IDLE;
byte walkPhase = 0;
int walkCyclesLeft = 0;
bool walkContinuously = false;
bool currentPositionKnown = true;

char inputBuffer[80];
byte inputLength = 0;

#ifdef USBCON
#define LOG_PORT SerialUSB
#else
#define LOG_PORT Serial
#endif

float radiansToDegrees(float radians) {
  return radians * 180.0f / PI;
}

bool calculateServoAngles(const Position &position, float &coxaAngle, float &femurAngle, float &tibiaAngle) {
  float horizontal = sqrt(position.x * position.x + position.y * position.y);
  float planar = horizontal - COXA_LENGTH;
  float distance = sqrt(planar * planar + position.z * position.z);
  float minReach = fabs(TIBIA_LENGTH - FEMUR_LENGTH);
  float maxReach = TIBIA_LENGTH + FEMUR_LENGTH;

  if (horizontal <= COXA_LENGTH || distance < minReach || distance > maxReach) {
    return false;
  }

  float femurCos = (FEMUR_LENGTH * FEMUR_LENGTH + distance * distance -
                    TIBIA_LENGTH * TIBIA_LENGTH) /
                   (2.0f * FEMUR_LENGTH * distance);
  float kneeCos = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH -
                   distance * distance) /
                  (2.0f * FEMUR_LENGTH * TIBIA_LENGTH);
  femurCos = constrain(femurCos, -1.0f, 1.0f);
  kneeCos = constrain(kneeCos, -1.0f, 1.0f);

  float yawDegrees = radiansToDegrees(atan2(position.y, position.x));
  float femurDegrees = radiansToDegrees(atan2(position.z, planar) + acos(femurCos));
  float kneeBendDegrees = 180.0f - radiansToDegrees(acos(kneeCos));

  coxaAngle = COXA_ZERO_DEG + COXA_DIRECTION * yawDegrees;
  femurAngle = FEMUR_ZERO_DEG + FEMUR_DIRECTION * femurDegrees;
  tibiaAngle = TIBIA_ZERO_DEG + TIBIA_DIRECTION * kneeBendDegrees;

  return coxaAngle >= 0.0f && coxaAngle <= 180.0f &&
         femurAngle >= 0.0f && femurAngle <= 180.0f &&
         tibiaAngle >= 0.0f && tibiaAngle <= 180.0f;
}

bool writePosition(const Position &position) {
  float coxaAngle;
  float femurAngle;
  float tibiaAngle;
  if (!calculateServoAngles(position, coxaAngle, femurAngle, tibiaAngle)) {
    return false;
  }

  coxa.write((int)(coxaAngle + 0.5f));
  femur.write((int)(femurAngle + 0.5f));
  tibia.write((int)(tibiaAngle + 0.5f));
  return true;
}

void stopMotion() {
  segmentActive = false;
  motionMode = IDLE;
  walkContinuously = false;
}

void writeRawAngles(int coxaAngle, int femurAngle, int tibiaAngle) {
  coxa.write(constrain(coxaAngle, 0, 180));
  femur.write(constrain(femurAngle, 0, 180));
  tibia.write(constrain(tibiaAngle, 0, 180));
}

void setCurrentPosition(const Position &position) {
  currentPosition = position;
  segmentStart = position;
  segmentTarget = position;
  currentPositionKnown = true;
}

void centerServos() {
  stopMotion();
  writeRawAngles(90, 90, 90);
  Position straight = {COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH, 0.0f, 0.0f};
  setCurrentPosition(straight);
  LOG_PORT.println(F("OK CENTER RAW 90 90 90"));
}

void forceHome() {
  stopMotion();
  if (!writePosition(HOME)) {
    currentPositionKnown = false;
    LOG_PORT.println(F("ERR HOME position/calibration invalid"));
    return;
  }
  setCurrentPosition(HOME);
  LOG_PORT.println(F("OK HOME"));
}

bool beginSegment(const Position &target, unsigned long duration, bool parabolicSwing) {
  float ignoredCoxa;
  float ignoredFemur;
  float ignoredTibia;
  if (!currentPositionKnown) {
    LOG_PORT.println(F("ERR current position unknown; send HOME or CENTER first"));
    return false;
  }
  if (!calculateServoAngles(target, ignoredCoxa, ignoredFemur, ignoredTibia)) {
    LOG_PORT.println(F("ERR target unreachable or outside servo limits"));
    return false;
  }

  segmentStart = currentPosition;
  segmentTarget = target;
  segmentStartTime = millis();
  segmentDuration = duration;
  segmentActive = true;
  segmentIsParabolicSwing = parabolicSwing;
  return true;
}

void updateSegment() {
  if (!segmentActive) {
    return;
  }

  unsigned long elapsed = millis() - segmentStartTime;
  float progress = segmentDuration == 0 ? 1.0f : (float)elapsed / (float)segmentDuration;
  progress = constrain(progress, 0.0f, 1.0f);

  if (segmentIsParabolicSwing) {
    currentPosition.x = segmentStart.x + (segmentTarget.x - segmentStart.x) * progress;
    currentPosition.y = segmentStart.y + (segmentTarget.y - segmentStart.y) * progress;
    currentPosition.z = segmentStart.z + (segmentTarget.z - segmentStart.z) * progress +
                        SWING_HEIGHT * 4.0f * progress * (1.0f - progress);
  } else {
    // Smoothstep reduces abrupt acceleration for straight placement movements.
    float blend = progress * progress * (3.0f - 2.0f * progress);
    currentPosition.x = segmentStart.x + (segmentTarget.x - segmentStart.x) * blend;
    currentPosition.y = segmentStart.y + (segmentTarget.y - segmentStart.y) * blend;
    currentPosition.z = segmentStart.z + (segmentTarget.z - segmentStart.z) * blend;
  }

  if (!writePosition(currentPosition)) {
    segmentActive = false;
    motionMode = IDLE;
    currentPositionKnown = false;
    LOG_PORT.println(F("ERR motion crossed an invalid position"));
    return;
  }

  if (progress >= 1.0f) {
    setCurrentPosition(segmentTarget);
    segmentActive = false;
  }
}

void startManualMove(const Position &target, unsigned long duration) {
  if (beginSegment(target, duration, false)) {
    motionMode = MANUAL_MOVE;
    LOG_PORT.println(F("OK MOVING"));
  }
}

void startWalkCycle(int cycles, bool continuous) {
  walkContinuously = continuous;
  walkCyclesLeft = continuous ? 0 : constrain(cycles, 1, 100);
  walkPhase = 0;
  motionMode = WALK_CYCLE;
  if (!beginSegment(GAIT_FRONT, APPROACH_MS, false)) {
    motionMode = IDLE;
    return;
  }
  if (walkContinuously) {
    LOG_PORT.println(F("OK WALK CONTINUOUS"));
  } else {
    LOG_PORT.print(F("OK WALK "));
    LOG_PORT.println(walkCyclesLeft);
  }
}

void updateWalkCycle() {
  if (motionMode != WALK_CYCLE || segmentActive) {
    return;
  }

  if (walkPhase == 0) {
    walkPhase = 1;
    beginSegment(GAIT_REAR, STANCE_MS, false);
  } else if (walkPhase == 1) {
    walkPhase = 2;
    beginSegment(GAIT_FRONT, SWING_MS, true);
  } else {
    if (!walkContinuously) {
      walkCyclesLeft--;
    }
    if (!walkContinuously && walkCyclesLeft <= 0) {
      motionMode = IDLE;
      LOG_PORT.println(F("DONE WALK"));
    } else {
      walkPhase = 1;
      beginSegment(GAIT_REAR, STANCE_MS, false);
    }
  }
}

void printStatus() {
  float coxaAngle;
  float femurAngle;
  float tibiaAngle;
  bool positionValid = currentPositionKnown &&
                       calculateServoAngles(currentPosition, coxaAngle, femurAngle, tibiaAngle);

  LOG_PORT.print(F("POS "));
  LOG_PORT.print(currentPosition.x, 1);
  LOG_PORT.print(' ');
  LOG_PORT.print(currentPosition.y, 1);
  LOG_PORT.print(' ');
  LOG_PORT.print(currentPosition.z, 1);
  if (positionValid) {
    LOG_PORT.print(F(" ANGLES "));
    LOG_PORT.print(coxaAngle, 1);
    LOG_PORT.print(' ');
    LOG_PORT.print(femurAngle, 1);
    LOG_PORT.print(' ');
    LOG_PORT.println(tibiaAngle, 1);
  } else {
    LOG_PORT.println(F(" ANGLES UNKNOWN"));
  }
}

void printHelp() {
  LOG_PORT.println(F("Commands: MOVE x y z [milliseconds] | HOME | CENTER | RAW c f t | WALK [cycles] | STOP | STATUS | HELP"));
  LOG_PORT.println(F("HOME directly writes the home IK pose. CENTER directly writes raw 90 90 90."));
  LOG_PORT.println(F("WALK repeats until STOP; WALK cycles runs a finite number of cycles."));
  LOG_PORT.println(F("Coordinates in mm: x outward, y forward/backward, +z upward."));
}

void handleCommand(char *line) {
  char *command = strtok(line, " \t");
  if (command == NULL) {
    return;
  }
  for (char *character = command; *character; character++) {
    *character = (char)toupper(*character);
  }

  if (strcmp(command, "MOVE") == 0) {
    char *xText = strtok(NULL, " \t");
    char *yText = strtok(NULL, " \t");
    char *zText = strtok(NULL, " \t");
    char *durationText = strtok(NULL, " \t");
    if (xText == NULL || yText == NULL || zText == NULL) {
      LOG_PORT.println(F("ERR usage: MOVE x y z [milliseconds]"));
      return;
    }
    Position target = {atof(xText), atof(yText), atof(zText)};
    unsigned long duration = durationText == NULL ? DEFAULT_MOVE_MS : strtoul(durationText, NULL, 10);
    startManualMove(target, duration);
  } else if (strcmp(command, "HOME") == 0) {
    forceHome();
  } else if (strcmp(command, "CENTER") == 0) {
    centerServos();
  } else if (strcmp(command, "RAW") == 0) {
    char *coxaText = strtok(NULL, " \t");
    char *femurText = strtok(NULL, " \t");
    char *tibiaText = strtok(NULL, " \t");
    if (coxaText == NULL || femurText == NULL || tibiaText == NULL) {
      LOG_PORT.println(F("ERR usage: RAW coxa femur tibia"));
      return;
    }
    stopMotion();
    writeRawAngles(atoi(coxaText), atoi(femurText), atoi(tibiaText));
    currentPositionKnown = false;
    LOG_PORT.println(F("OK RAW"));
  } else if (strcmp(command, "WALK") == 0) {
    char *cyclesText = strtok(NULL, " \t");
    if (cyclesText == NULL) {
      startWalkCycle(0, true);
    } else {
      startWalkCycle(atoi(cyclesText), false);
    }
  } else if (strcmp(command, "STOP") == 0) {
    stopMotion();
    LOG_PORT.println(F("OK STOPPED"));
  } else if (strcmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else {
    LOG_PORT.println(F("ERR unknown command; send HELP"));
  }
}

void readCommandByte(char character) {
  if (character == '\n' || character == '\r') {
    if (inputLength > 0) {
      inputBuffer[inputLength] = '\0';
      handleCommand(inputBuffer);
      inputLength = 0;
    }
  } else if (inputLength < sizeof(inputBuffer) - 1) {
    inputBuffer[inputLength++] = character;
  } else {
    inputLength = 0;
    LOG_PORT.println(F("ERR command too long"));
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    readCommandByte((char)Serial.read());
  }
#ifdef USBCON
  while (SerialUSB.available() > 0) {
    readCommandByte((char)SerialUSB.read());
  }
#endif
}

void setup() {
  Serial.begin(115200);
#ifdef USBCON
  SerialUSB.begin(115200);
#endif
  coxa.attach(COXA_PIN);
  femur.attach(FEMUR_PIN);
  tibia.attach(TIBIA_PIN);

  forceHome();
  delay(500);
  LOG_PORT.println(F("READY LEG CONTROLLER"));
  printHelp();
}

void loop() {
  readSerialCommands();
  updateSegment();

  if (motionMode == MANUAL_MOVE && !segmentActive) {
    motionMode = IDLE;
    LOG_PORT.println(F("DONE MOVE"));
  } else {
    updateWalkCycle();
  }
}
