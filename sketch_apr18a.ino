#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <KerbalSimpit.h>
#include <Encoder.h>
#include <SevSeg.h>

KerbalSimpit mySimpit(Serial);
LiquidCrystal_I2C lcd(0x27, 16, 2);
SevSeg sevseg;

// Joystick
const byte PIN_VERTICAL = A0;
const byte PIN_HORIZONTAL = A1;
const byte PIN_JOYSTICK_SW = 46;

// LCD page encoder
Encoder pageEnc(2, 3);
const byte PIN_PAGE_ENC_SW = 4;

// One shared toggle switch
const byte PIN_DISPLAY_MODE_SWITCH = 5;

// LEDs
const byte PIN_SAS_LED = 6;
const byte PIN_RCS_LED = 7;
const byte PIN_ORBIT_LED = 15;
const byte PIN_SYSTEM_OFF_LED = 16;

// Main controls
const byte PIN_BRAKES = 8;
const byte PIN_LIGHTS = 9;
const byte PIN_STAGE = 10;
const byte PIN_SAS = 11;
const byte PIN_RCS = 12;
const byte PIN_GEAR = 14;

// Throttle encoder
Encoder throttleEnc(18, 19);
const byte PIN_THROTTLE_SW = 29;

// 1 digit stage display: A, B, C, D, E, F, G
const byte stageSegmentPins[] = {22, 23, 24, 25, 26, 27, 28};

// 4 digit 7 segment altitude display
byte numDigits = 4;

// Reversed from original so USSR displays in the correct order
byte digitPins[] = {30, 31, 32, 33};

byte segmentPins[] = {34, 35, 36, 37, 38, 39, 40, 41};

// Action groups
const byte agPins[] = {42, 43, 44, 45};

const int DEADZONE = 50;
const unsigned long BUTTON_DEBOUNCE_MS = 250;
const unsigned long JOYSTICK_SEND_MS = 20;
const unsigned long LCD_UPDATE_MS = 100;
const unsigned long SEV_SET_MS = 25;

const uint16_t MAX_THROTTLE = 65535;
const uint16_t MIN_THROTTLE = 0;
const uint16_t STEP_SIZE = 2184;
const long MAX_ENCODER_POS = 120;

const int totalPages = 4;

const byte digitPatterns[] = {
  0b00111111,
  0b00000110,
  0b01011011,
  0b01001111,
  0b01100110,
  0b01101101,
  0b01111101,
  0b00000111,
  0b01111111,
  0b01101111
};

// Stage countdown / display
bool isCounting = false;
bool firstStageDone = false;
int stageCount = 0;
unsigned long stageStartTime = 0;

// LCD pages
long oldPagePosition = -999;
int currentPage = 0;

// Throttle
uint16_t currentThrottle = 0;
long oldThrottlePosition = -999;

// Flight data
double vSurf = 0.0;
float altSea = 0.0;
float apoapsis = 0.0;
float periapsis = 0.0;
float burnDV = 0.0;
int32_t burnTime = 0;
float targetDist = 0.0;
float targetV = 0.0;

// 4 digit display data
long currentAltitude = 0;
bool connected = false;

// LEDs
bool reachedSpace = false;

// Timing state for stable SevSeg refresh
unsigned long lastJoystickSend = 0;
unsigned long lastJoystickToggle = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastSevSet = 0;

// Button states
bool lastAgState[4] = {HIGH, HIGH, HIGH, HIGH};
bool lastStageState = HIGH;
bool lastPageEncSwState = HIGH;
bool lastJoystickSwState = HIGH;
bool joystickRollMode = false;
bool lastSasState = HIGH;
bool lastRcsState = HIGH;
bool lastBrakesState = HIGH;
bool lastLightsState = HIGH;
bool lastGearState = HIGH;
bool lastThrottleSwState = HIGH;

bool pressed(byte pin, bool &lastState) {
  bool currentState = digitalRead(pin);
  bool didPress = currentState == LOW && lastState == HIGH;
  lastState = currentState;
  return didPress;
}

bool displayModeActive() {
  return digitalRead(PIN_DISPLAY_MODE_SWITCH) == LOW;
}

void displayStageDigit(int digit) {
  if (digit < 0 || digit > 9) {
    for (int i = 0; i < 7; i++) {
      digitalWrite(stageSegmentPins[i], LOW);
    }
    return;
  }

  byte pattern = digitPatterns[digit];

  for (int i = 0; i < 7; i++) {
    digitalWrite(stageSegmentPins[i], bitRead(pattern, i));
  }
}

void resetStageDisplay() {
  isCounting = false;
  firstStageDone = false;
  stageCount = 0;
  displayStageDigit(-1);
}

void formatDist(float dist) {
  if (dist > 1000000) {
    lcd.print(dist / 1000000, 2);
    lcd.print("Mm");
  } else if (dist > 10000) {
    lcd.print(dist / 1000, 1);
    lcd.print("km");
  } else {
    lcd.print(dist, 0);
    lcd.print("m");
  }
}

void displayAscent() {
  sevseg.refreshDisplay();
  lcd.setCursor(0, 0);
  lcd.print("SPD:");
  lcd.print(vSurf, 1);
  lcd.print("m/s    ");

  sevseg.refreshDisplay();
  lcd.setCursor(0, 1);
  lcd.print("ALT:");
  lcd.print(altSea > 10000 ? altSea / 1000 : altSea, 1);
  lcd.print(altSea > 10000 ? "km     " : "m      ");
  sevseg.refreshDisplay();
}

void displayOrbit() {
  sevseg.refreshDisplay();
  lcd.setCursor(0, 0);
  lcd.print("AP:");
  formatDist(apoapsis);
  lcd.print("        ");

  sevseg.refreshDisplay();
  lcd.setCursor(0, 1);
  lcd.print("PE:");
  formatDist(periapsis);
  lcd.print("        ");
  sevseg.refreshDisplay();
}

void displayManeuver() {
  sevseg.refreshDisplay();
  lcd.setCursor(0, 0);
  lcd.print("NODE:");
  lcd.print(burnDV, 1);
  lcd.print("dV     ");

  sevseg.refreshDisplay();
  lcd.setCursor(0, 1);
  lcd.print("TIME:");
  lcd.print(burnTime);
  lcd.print("s      ");
  sevseg.refreshDisplay();
}

void displayTarget() {
  sevseg.refreshDisplay();
  lcd.setCursor(0, 0);
  lcd.print("TGT:");
  formatDist(targetDist);
  lcd.print("        ");

  sevseg.refreshDisplay();
  lcd.setCursor(0, 1);
  lcd.print("RELV:");
  lcd.print(targetV, 1);
  lcd.print("m/s    ");
  sevseg.refreshDisplay();
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch (messageType) {
    case VELOCITY_MESSAGE:
      if (msgSize == sizeof(velocityMessage)) {
        velocityMessage vData;
        memcpy(&vData, msg, sizeof(velocityMessage));
        vSurf = vData.surface;
      }
      break;

    case ALTITUDE_MESSAGE:
      if (msgSize == sizeof(altitudeMessage)) {
        altitudeMessage aData;
        memcpy(&aData, msg, sizeof(altitudeMessage));

        altSea = aData.sealevel;
        currentAltitude = (long)aData.sealevel;
        connected = true;

        if (altSea >= 70000.0) {
          reachedSpace = true;
        }

        if (altSea < 500.0) {
          reachedSpace = false;
        }

        digitalWrite(PIN_ORBIT_LED, reachedSpace ? HIGH : LOW);
      }
      break;

    case APSIDES_MESSAGE:
      if (msgSize == sizeof(apsidesMessage)) {
        apsidesMessage apData;
        memcpy(&apData, msg, sizeof(apsidesMessage));
        apoapsis = apData.apoapsis;
        periapsis = apData.periapsis;
      }
      break;

    case MANEUVER_MESSAGE:
      if (msgSize == sizeof(maneuverMessage)) {
        maneuverMessage mData;
        memcpy(&mData, msg, sizeof(maneuverMessage));
        burnDV = mData.deltaVTotal;
        burnTime = mData.timeToNextManeuver;
      }
      break;

    case TARGETINFO_MESSAGE:
      if (msgSize == sizeof(targetMessage)) {
        targetMessage tData;
        memcpy(&tData, msg, sizeof(targetMessage));
        targetDist = tData.distance;
        targetV = tData.velocity;
      }
      break;

    case ACTIONSTATUS_MESSAGE:
      {
        bool sasActive = msg[0] & 16;
        bool rcsActive = msg[0] & 8;

        digitalWrite(PIN_SAS_LED, sasActive ? HIGH : LOW);
        digitalWrite(PIN_RCS_LED, rcsActive ? HIGH : LOW);
        digitalWrite(PIN_SYSTEM_OFF_LED, (!sasActive && !rcsActive) ? HIGH : LOW);
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_VERTICAL, INPUT);
  pinMode(PIN_HORIZONTAL, INPUT);
  pinMode(PIN_JOYSTICK_SW, INPUT_PULLUP);

  pinMode(PIN_PAGE_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_DISPLAY_MODE_SWITCH, INPUT_PULLUP);

  pinMode(PIN_STAGE, INPUT_PULLUP);
  pinMode(PIN_SAS, INPUT_PULLUP);
  pinMode(PIN_RCS, INPUT_PULLUP);
  pinMode(PIN_BRAKES, INPUT_PULLUP);
  pinMode(PIN_LIGHTS, INPUT_PULLUP);
  pinMode(PIN_GEAR, INPUT_PULLUP);
  pinMode(PIN_THROTTLE_SW, INPUT_PULLUP);

  for (int i = 0; i < 4; i++) {
    pinMode(agPins[i], INPUT_PULLUP);
  }

  for (int i = 0; i < 7; i++) {
    pinMode(stageSegmentPins[i], OUTPUT);
  }

  pinMode(PIN_SAS_LED, OUTPUT);
  pinMode(PIN_RCS_LED, OUTPUT);
  pinMode(PIN_ORBIT_LED, OUTPUT);
  pinMode(PIN_SYSTEM_OFF_LED, OUTPUT);

  displayStageDigit(-1);

  lcd.init();
  lcd.backlight();

  // COMMON_CATHODE restored; digit order stays reversed
  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins, false);
  sevseg.setBrightness(100);

  while (!mySimpit.init()) {
    sevseg.refreshDisplay();
    delay(10);
  }

  mySimpit.inboundHandler(messageHandler);

  mySimpit.registerChannel(ROTATION_MESSAGE);
  mySimpit.registerChannel(VELOCITY_MESSAGE);
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  mySimpit.registerChannel(APSIDES_MESSAGE);
  mySimpit.registerChannel(MANEUVER_MESSAGE);
  mySimpit.registerChannel(TARGETINFO_MESSAGE);
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);

  lcd.print("PS-2 / SPUTNIK-2");
  delay(1500);
  lcd.clear();
}

void handleJoystickButton() {
  bool currentState = digitalRead(PIN_JOYSTICK_SW);

  if (currentState == LOW &&
      lastJoystickSwState == HIGH &&
      millis() - lastJoystickToggle >= BUTTON_DEBOUNCE_MS) {
    joystickRollMode = !joystickRollMode;
    lastJoystickToggle = millis();
  }

  lastJoystickSwState = currentState;
}

void handleJoystick() {
  if (millis() - lastJoystickSend < JOYSTICK_SEND_MS) return;
  lastJoystickSend = millis();

  int rawVertical = analogRead(PIN_VERTICAL);
  int rawHorizontal = analogRead(PIN_HORIZONTAL);

  int verticalVal = (abs(rawVertical - 512) < DEADZONE) ? 0 : (rawVertical - 512);
  int horizontalVal = (abs(rawHorizontal - 512) < DEADZONE) ? 0 : (rawHorizontal - 512);

  rotationMessage rotation;

  if (joystickRollMode) {
    rotation.pitch = 0;
    rotation.yaw = 0;
    rotation.roll = (int16_t)map(horizontalVal, -512, 512, -32768, 32767);
    rotation.mask = 7;
  } else {
    rotation.pitch = (int16_t)map(verticalVal, -512, 512, 32767, -32768);
    rotation.yaw = (int16_t)map(horizontalVal, -512, 512, -32768, 32767);
    rotation.roll = 0;
    rotation.mask = 7;
  }

  mySimpit.send(ROTATION_MESSAGE, rotation);
}

void handleStage() {
  if (pressed(PIN_STAGE, lastStageState)) {
    if (firstStageDone) {
      mySimpit.activateAction(STAGE_ACTION);
      stageCount++;

      if (stageCount > 9) {
        stageCount = 0;
      }

      displayStageDigit(stageCount);
    } else if (!isCounting) {
      isCounting = true;
      stageStartTime = millis();
    }
  }

  if (isCounting) {
    unsigned long elapsed = millis() - stageStartTime;
    int remaining = 5 - (elapsed / 1000);

    if (remaining >= 0) {
      displayStageDigit(remaining);
    }

    if (elapsed >= 5000) {
      mySimpit.activateAction(STAGE_ACTION);
      isCounting = false;
      firstStageDone = true;
      stageCount = 0;
      displayStageDigit(0);
    }
  }
}

void handlePageEncoderButton() {
  if (pressed(PIN_PAGE_ENC_SW, lastPageEncSwState)) {
    resetStageDisplay();
  }
}

void handleActionButtons() {
  if (pressed(PIN_SAS, lastSasState)) mySimpit.toggleAction(SAS_ACTION);
  if (pressed(PIN_RCS, lastRcsState)) mySimpit.toggleAction(RCS_ACTION);
  if (pressed(PIN_BRAKES, lastBrakesState)) mySimpit.toggleAction(BRAKES_ACTION);
  if (pressed(PIN_LIGHTS, lastLightsState)) mySimpit.toggleAction(LIGHT_ACTION);
  if (pressed(PIN_GEAR, lastGearState)) mySimpit.toggleAction(GEAR_ACTION);

  for (int i = 0; i < 4; i++) {
    bool currentButtonState = digitalRead(agPins[i]);

    if (currentButtonState == LOW && lastAgState[i] == HIGH) {
      mySimpit.toggleCAG(i + 1);
    }

    lastAgState[i] = currentButtonState;
  }
}

void handleThrottle() {
  long newPosition = throttleEnc.read();

  if ((newPosition / 4) != (oldThrottlePosition / 4)) {
    if (currentThrottle >= MAX_THROTTLE && newPosition > oldThrottlePosition) {
      throttleEnc.write(MAX_ENCODER_POS);
      oldThrottlePosition = MAX_ENCODER_POS;
      return;
    }

    if (newPosition > oldThrottlePosition) {
      currentThrottle = currentThrottle > (MAX_THROTTLE - STEP_SIZE)
        ? MAX_THROTTLE
        : currentThrottle + STEP_SIZE;
    } else {
      currentThrottle = currentThrottle < STEP_SIZE
        ? MIN_THROTTLE
        : currentThrottle - STEP_SIZE;
    }

    throttleMessage msg;
    msg.throttle = currentThrottle;
    mySimpit.send(THROTTLE_MESSAGE, msg);

    oldThrottlePosition = newPosition;
  }

  if (pressed(PIN_THROTTLE_SW, lastThrottleSwState)) {
    currentThrottle = MAX_THROTTLE;

    throttleMessage msg;
    msg.throttle = currentThrottle;
    mySimpit.send(THROTTLE_MESSAGE, msg);

    throttleEnc.write(MAX_ENCODER_POS);
    oldThrottlePosition = MAX_ENCODER_POS;
  }
}

void handleLCD() {
  if (millis() - lastLcdUpdate < LCD_UPDATE_MS) return;
  lastLcdUpdate = millis();

  if (!displayModeActive()) {
    sevseg.refreshDisplay();
    lcd.setCursor(0, 0);
    lcd.print("PS-2 / SPUTNIK-2");
    sevseg.refreshDisplay();
    lcd.setCursor(0, 1);
    lcd.print(" TELEMETRY: STBY");
    oldPagePosition = -999;
  } else {
    long newPosition = pageEnc.read() / 4;

    if (newPosition != oldPagePosition) {
      currentPage = abs(newPosition % totalPages);
      sevseg.refreshDisplay();
      lcd.clear();
      sevseg.refreshDisplay();
      oldPagePosition = newPosition;
    }

    switch (currentPage) {
      case 0: displayAscent(); break;
      case 1: displayOrbit(); break;
      case 2: displayManeuver(); break;
      case 3: displayTarget(); break;
    }
  }
}

void handleAltitudeDisplay() {
  if (millis() - lastSevSet >= SEV_SET_MS) {
    lastSevSet = millis();

    bool switchIsOn = digitalRead(PIN_DISPLAY_MODE_SWITCH) == LOW;

    if (!switchIsOn) {
      sevseg.setChars("USSR");
    } else {
      if (!connected) {
        sevseg.setChars("wait");
      } else {
        if (currentAltitude >= 10000) {
          sevseg.setNumber(currentAltitude / 10, 2);
        } else {
          sevseg.setNumber(currentAltitude);
        }
      }
    }
  }

  sevseg.refreshDisplay();
}

void loop() {
  sevseg.refreshDisplay();

  mySimpit.update();
  sevseg.refreshDisplay();

  handleStage();
  sevseg.refreshDisplay();
  handlePageEncoderButton();
  sevseg.refreshDisplay();
  handleActionButtons();
  sevseg.refreshDisplay();
  handleThrottle();
  sevseg.refreshDisplay();
  handleJoystickButton();
  sevseg.refreshDisplay();
  handleJoystick();
  sevseg.refreshDisplay();
  handleLCD();
  sevseg.refreshDisplay();
  handleAltitudeDisplay();
}
