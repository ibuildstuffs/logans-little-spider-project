#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

const byte COXA_PIN = 8;
const byte FEMUR_PIN = 9;
const byte TIBIA_PIN = 10;

// Calibration mapping:
// physical_angle 0 should mean the joint's mechanical zero position.
// servo_angle is the raw value sent to Servo.write().
//
// Find these zero values with RAW commands first, then update them here.
const int COXA_PHYSICAL_ZERO_SERVO_ANGLE = 90;
const int FEMUR_PHYSICAL_ZERO_SERVO_ANGLE = 90;
const int TIBIA_PHYSICAL_ZERO_SERVO_ANGLE = 90;

// Change a direction to -1 if positive physical angles move the wrong way.
const int COXA_DIRECTION = 1;
const int FEMUR_DIRECTION = 1;
const int TIBIA_DIRECTION = 1;

const int SAFE_PHYSICAL_ANGLE = 0;
const int MIN_PHYSICAL_TEST_ANGLE = -45;
const int MAX_PHYSICAL_TEST_ANGLE = 45;
const int STEP_DEGREES = 2;
const unsigned long STEP_DELAY_MS = 25;

Servo coxa;
Servo femur;
Servo tibia;

char inputBuffer[80];
byte inputLength = 0;

#ifdef USBCON
#define LOG_PORT SerialUSB
#else
#define LOG_PORT Serial
#endif

int physicalToServoAngle(const char *jointName, int physicalAngle) {
  if (strcmp(jointName, "COXA") == 0 || strcmp(jointName, "0") == 0) {
    return constrain(COXA_PHYSICAL_ZERO_SERVO_ANGLE + COXA_DIRECTION * physicalAngle, 0, 180);
  }
  if (strcmp(jointName, "FEMUR") == 0 || strcmp(jointName, "1") == 0) {
    return constrain(FEMUR_PHYSICAL_ZERO_SERVO_ANGLE + FEMUR_DIRECTION * physicalAngle, 0, 180);
  }
  if (strcmp(jointName, "TIBIA") == 0 || strcmp(jointName, "2") == 0) {
    return constrain(TIBIA_PHYSICAL_ZERO_SERVO_ANGLE + TIBIA_DIRECTION * physicalAngle, 0, 180);
  }
  return 90;
}

Servo *servoForJoint(const char *jointName) {
  if (strcmp(jointName, "COXA") == 0 || strcmp(jointName, "0") == 0) {
    return &coxa;
  }
  if (strcmp(jointName, "FEMUR") == 0 || strcmp(jointName, "1") == 0) {
    return &femur;
  }
  if (strcmp(jointName, "TIBIA") == 0 || strcmp(jointName, "2") == 0) {
    return &tibia;
  }
  return NULL;
}

const __FlashStringHelper *displayNameForJoint(const char *jointName) {
  if (strcmp(jointName, "COXA") == 0 || strcmp(jointName, "0") == 0) {
    return F("COXA");
  }
  if (strcmp(jointName, "FEMUR") == 0 || strcmp(jointName, "1") == 0) {
    return F("FEMUR");
  }
  return F("TIBIA");
}

void writePhysicalAngle(const char *jointName, int physicalAngle) {
  Servo *servo = servoForJoint(jointName);
  if (servo == NULL) {
    LOG_PORT.println(F("ERR unknown joint"));
    return;
  }

  int servoAngle = physicalToServoAngle(jointName, physicalAngle);
  servo->write(servoAngle);
  LOG_PORT.print(F("OK SET "));
  LOG_PORT.print(displayNameForJoint(jointName));
  LOG_PORT.print(F(" physical="));
  LOG_PORT.print(physicalAngle);
  LOG_PORT.print(F(" servo="));
  LOG_PORT.println(servoAngle);
}

void writeRawServoAngle(const char *jointName, int servoAngle) {
  Servo *servo = servoForJoint(jointName);
  if (servo == NULL) {
    LOG_PORT.println(F("ERR unknown joint"));
    return;
  }

  servoAngle = constrain(servoAngle, 0, 180);
  servo->write(servoAngle);
  LOG_PORT.print(F("OK RAW "));
  LOG_PORT.print(displayNameForJoint(jointName));
  LOG_PORT.print(F(" servo="));
  LOG_PORT.println(servoAngle);
}

void centerAll() {
  coxa.write(physicalToServoAngle("COXA", SAFE_PHYSICAL_ANGLE));
  femur.write(physicalToServoAngle("FEMUR", SAFE_PHYSICAL_ANGLE));
  tibia.write(physicalToServoAngle("TIBIA", SAFE_PHYSICAL_ANGLE));
  LOG_PORT.println(F("OK CENTERED ALL JOINTS AT PHYSICAL 0"));
}

void sweepServo(const char *jointName) {
  LOG_PORT.print(F("TESTING "));
  LOG_PORT.println(displayNameForJoint(jointName));

  writePhysicalAngle(jointName, SAFE_PHYSICAL_ANGLE);
  delay(500);

  for (int angle = SAFE_PHYSICAL_ANGLE; angle <= MAX_PHYSICAL_TEST_ANGLE; angle += STEP_DEGREES) {
    servoForJoint(jointName)->write(physicalToServoAngle(jointName, angle));
    delay(STEP_DELAY_MS);
  }
  for (int angle = MAX_PHYSICAL_TEST_ANGLE; angle >= MIN_PHYSICAL_TEST_ANGLE; angle -= STEP_DEGREES) {
    servoForJoint(jointName)->write(physicalToServoAngle(jointName, angle));
    delay(STEP_DELAY_MS);
  }
  for (int angle = MIN_PHYSICAL_TEST_ANGLE; angle <= SAFE_PHYSICAL_ANGLE; angle += STEP_DEGREES) {
    servoForJoint(jointName)->write(physicalToServoAngle(jointName, angle));
    delay(STEP_DELAY_MS);
  }

  LOG_PORT.print(F("DONE "));
  LOG_PORT.println(displayNameForJoint(jointName));
}

void testAll() {
  centerAll();
  delay(800);
  sweepServo("COXA");
  delay(800);
  sweepServo("FEMUR");
  delay(800);
  sweepServo("TIBIA");
  centerAll();
}

void printHelp() {
  LOG_PORT.println(F("Joint servo test commands:"));
  LOG_PORT.println(F("  CENTER"));
  LOG_PORT.println(F("  TEST ALL"));
  LOG_PORT.println(F("  TEST COXA   or TEST 0"));
  LOG_PORT.println(F("  TEST FEMUR  or TEST 1"));
  LOG_PORT.println(F("  TEST TIBIA  or TEST 2"));
  LOG_PORT.println(F("  SET COXA 0      physical angle"));
  LOG_PORT.println(F("  SET FEMUR 30    physical angle"));
  LOG_PORT.println(F("  RAW COXA 90     direct Servo.write angle"));
  LOG_PORT.println(F("  RAW FEMUR 120   direct Servo.write angle"));
  LOG_PORT.println(F("Use RAW to find mechanical zero, then update *_PHYSICAL_ZERO_SERVO_ANGLE."));
}

void uppercase(char *text) {
  for (char *character = text; *character; character++) {
    *character = (char)toupper(*character);
  }
}

void handleCommand(char *line) {
  uppercase(line);
  char *command = strtok(line, " \t");
  if (command == NULL) {
    return;
  }

  if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else if (strcmp(command, "CENTER") == 0) {
    centerAll();
  } else if (strcmp(command, "TEST") == 0) {
    char *jointName = strtok(NULL, " \t");
    if (jointName == NULL) {
      LOG_PORT.println(F("ERR usage: TEST ALL|COXA|FEMUR|TIBIA"));
      return;
    }
    if (strcmp(jointName, "ALL") == 0) {
      testAll();
      return;
    }
    Servo *servo = servoForJoint(jointName);
    if (servo == NULL) {
      LOG_PORT.println(F("ERR unknown joint"));
      return;
    }
    sweepServo(jointName);
  } else if (strcmp(command, "SET") == 0) {
    char *jointName = strtok(NULL, " \t");
    char *angleText = strtok(NULL, " \t");
    if (jointName == NULL || angleText == NULL) {
      LOG_PORT.println(F("ERR usage: SET COXA|FEMUR|TIBIA physical_angle"));
      return;
    }
    writePhysicalAngle(jointName, atoi(angleText));
  } else if (strcmp(command, "RAW") == 0) {
    char *jointName = strtok(NULL, " \t");
    char *angleText = strtok(NULL, " \t");
    if (jointName == NULL || angleText == NULL) {
      LOG_PORT.println(F("ERR usage: RAW COXA|FEMUR|TIBIA servo_angle"));
      return;
    }
    writeRawServoAngle(jointName, atoi(angleText));
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
  centerAll();
  LOG_PORT.println(F("READY JOINT SERVO TEST"));
  printHelp();
}

void loop() {
  readSerialCommands();
}
