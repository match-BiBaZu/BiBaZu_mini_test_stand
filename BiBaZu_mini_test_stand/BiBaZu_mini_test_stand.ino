const int flowSensor_InPin = A1;
const int pressureBeforeValve_InPin = A0;
const int regulatorFeedback_InPin = A2;  // VEAB black output wire
const int regulatorOutPin = 3;           // PWM-to-0-10 V converter input (Mega 2560 OC3C)
const unsigned int regulatorPwmTop = 999;  // 16 MHz / (8 * (999 + 1)) = 2 kHz.

// Measured converter characteristic after calibrating 100% PWM to 10.00 V.
// The converter jumps from 0 V to about 1.04 V at its smallest PWM input.
const float converterCalibrationDutyPercent[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};
const float converterCalibrationVoltage[] = {
  0.000, 1.040, 1.190, 1.304, 1.407, 1.508, 1.609, 1.710, 1.804, 1.911,
  2.020, 3.030, 4.030, 5.030, 6.030, 7.020, 7.990, 9.000, 9.860, 10.000
};
const int converterCalibrationPointCount =
  sizeof(converterCalibrationVoltage) / sizeof(converterCalibrationVoltage[0]);

const int valvePins[] = {4, 5, 6, 7};
const int valveCount = sizeof(valvePins) / sizeof(valvePins[0]);
const int allValveMask = (1 << valveCount) - 1;
const int valveOpenSignal = HIGH;
const int valveClosedSignal = LOW;
const float sampleIntervalMs = 5.0;
const int defaultValvePulseOpenTicks = 10;  // 10 control ticks at 5 ms = 50 ms.
const int maxValvePulseOpenTicks = 100;    // Limit configurable pulses to 500 ms.
int valvePulseOpenTicks = defaultValvePulseOpenTicks;
const int flowCaptureMinSamples = 8;
const int flowCaptureQuietSamples = 3;
float flowCaptureThresholdNlMin = 2.0;

const int stepperStepPin = 8;
const int stepperDirPin = 9;
const int stepperEnablePin = 10;
const int stepperLimitPin = 11;
const int stepperLimitPressedSignal = LOW;
const int stepperStepActiveSignal = LOW;
const int stepperStepIdleSignal = HIGH;
const int stepperPositiveDirSignal = HIGH;
const int stepperNegativeDirSignal = LOW;
const int stepperEnableSignal = HIGH;
const int stepperDisableSignal = LOW;
const unsigned int stepperPulseWidthMicros = 5;
const float stepperMmPerStep = 0.009985846;


const int pressureFeedforwardPwm[] = {35, 55, 71, 86, 100, 112, 124, 136, 157, 176, 195, 212};
const float regulatorMaxPressure = 6.0;  // Calibrated 10 V converter output corresponds to 6 bar.
// Inverse fit from measured regulator feedback at 0.5, 1.5, 2.5, and 3.0 bar:
// measured pressure = command gain * command pressure + command offset.
const float regulatorCommandGain = 1.125664;
const float regulatorCommandOffsetBar = 0.049987;
const float testPressureStepBar = 0.10;
const int defaultTestPulsesPerPressure = 10;
const int maxTestPulsesPerPressure = 100;
const int maxPressureStep = 60;  // 6.0 bar / 0.10 bar.
const int testRepeatWaitTicks = 999;          // 999 + 1 ticks at 5 ms = 5.0 s.
const int testPressureStepWaitTicks = 1999;   // 1999 + 1 ticks at 5 ms = 10.0 s after a pressure increase.

// Flow sensor analog scaling for the white analog output configured to 1-5 V.
// The installed unidirectional sensor is configured as 0 ... 200 l/min.
const float flowSensorMinVoltage = 1.0;
const float flowSensorMaxVoltage = 5.0;
const float flowSensorMinNlMin = 0.0;
const float flowSensorMaxNlMin = 200.0;

int regulatorSetting = 0;
int pressureIndex = 0;
int testStartPressureStep = 10;
int testEndPressureStep = 16;
int testPulsesPerPressure = defaultTestPulsesPerPressure;
int testValveMask = allValveMask;
int pulseCounter = 0;
int valvePulseCounter = 0;
int sampleCounter = 0;
int streamCounter = 0;
int loopCounter = 0;
int waitTime = testPressureStepWaitTicks;
int activeValveMask = allValveMask;
int stepperSpeed = 400;
int stepperMoveDirection = 1;
unsigned long valveOpenStartMicros = 0;
unsigned long valveOpenDurationMicros = 0;
int flowCaptureSampleCount = 0;
int flowCaptureQuietCount = 0;

int rawPressureBeforeValve = 0;
int rawRegulatorFeedback = 0;
int rawFlow = 0;

float voltagePressureBeforeValve = 0;
float voltageRegulatorFeedback = 0;
float voltageFlow = 0;
float pressureBeforeValve = 0;
float regulatorPressure = 0;
float targetRegulatorPressure = 0.0;
float flow = 0;
float flowCaptureBaseline = 0;
float flowCaptureMax = 0;
float flowCaptureVolumeL = 0;

bool valvesOpen = false;
bool pulseRequested = false;
bool testRunning = false;
bool writeSamples = false;
bool streamContinuously = false;
bool stepperEnabled = false;
bool stepperPulseActive = false;
bool stepperReferenced = false;
bool stepperHoming = false;
bool lastStepperLimitPressed = false;
bool flowCaptureActive = false;
bool flowCaptureDone = false;
bool flowCapturePeakSeen = false;

long stepperRemainingSteps = 0;
long stepperCurrentPosition = 0;
unsigned long stepperStepIntervalMicros = 2500;
unsigned long stepperLastStepMicros = 0;
unsigned long stepperPulseStartMicros = 0;

enum OperatingMode {
  MODE_IDLE,
  MODE_MANUAL,
  MODE_TEST
};

OperatingMode operatingMode = MODE_IDLE;

unsigned long previousMicros = 0;
const long interval = 5000;
const int serialCommandMaxLength = 80;
String serialCommandBuffer = "";

void processSerialCommand(String command);
void setupRegulatorPwm();
void writeRegulatorPwm(int value);
float calibratedDutyForVoltage(float desiredVoltage);
void writeRegulatorDutyPercent(float dutyPercent);

void setup() {
  Serial.begin(230400);

  setupRegulatorPwm();
  for (int i = 0; i < valveCount; i++) {
    pinMode(valvePins[i], OUTPUT);
    digitalWrite(valvePins[i], valveClosedSignal);
  }
  pinMode(stepperStepPin, OUTPUT);
  pinMode(stepperDirPin, OUTPUT);
  pinMode(stepperEnablePin, OUTPUT);
  pinMode(stepperLimitPin, INPUT_PULLUP);

  writeRegulatorPwm(0);
  digitalWrite(stepperStepPin, stepperStepIdleSignal);
  digitalWrite(stepperDirPin, stepperPositiveDirSignal);
  digitalWrite(stepperEnablePin, stepperDisableSignal);
  Serial.println("READY");
  Serial.println("Send START from the GUI to begin the test.");
}

void setupRegulatorPwm() {
#if defined(__AVR_ATmega2560__)
  pinMode(regulatorOutPin, OUTPUT);
  digitalWrite(regulatorOutPin, LOW);

  // D3 is OC3C on the Mega 2560. Fast PWM mode 14 with a /8 prescaler
  // produces the 2 kHz signal required by the PWM-to-0-10 V converter.
  // Timer 3 also serves D2 and D5, but this sketch uses neither as PWM.
  TCCR3A = _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
  TCNT3 = 0;
  ICR3 = regulatorPwmTop;
  OCR3C = 0;
#else
#error "The regulator PWM setup is configured for an Arduino Mega 2560."
#endif
}

void writeRegulatorPwm(int value) {
  value = constrain(value, 0, 255);
  float desiredVoltage = value * 10.0 / 255.0;
  writeRegulatorDutyPercent(calibratedDutyForVoltage(desiredVoltage));
}

float calibratedDutyForVoltage(float desiredVoltage) {
  desiredVoltage = constrain(desiredVoltage, 0.0, 10.0);
  if (desiredVoltage <= 0.0) {
    return 0.0;
  }

  // No stable converter output exists between 0 and 1.04 V. Select the
  // closer of the two reachable endpoints instead of extrapolating.
  if (desiredVoltage < converterCalibrationVoltage[1] / 2.0) {
    return 0.0;
  }
  if (desiredVoltage <= converterCalibrationVoltage[1]) {
    return converterCalibrationDutyPercent[1];
  }

  for (int i = 2; i < converterCalibrationPointCount; i++) {
    if (desiredVoltage <= converterCalibrationVoltage[i]) {
      float voltageFraction =
        (desiredVoltage - converterCalibrationVoltage[i - 1]) /
        (converterCalibrationVoltage[i] - converterCalibrationVoltage[i - 1]);
      return converterCalibrationDutyPercent[i - 1] + voltageFraction *
        (converterCalibrationDutyPercent[i] - converterCalibrationDutyPercent[i - 1]);
    }
  }
  return 100.0;
}

void writeRegulatorDutyPercent(float dutyPercent) {
  dutyPercent = constrain(dutyPercent, 0.0, 100.0);
  if (dutyPercent <= 0.0) {
    // A static LOW guarantees a true 0 V command at the converter output.
    TCCR3A &= ~(_BV(COM3C1) | _BV(COM3C0));
    digitalWrite(regulatorOutPin, LOW);
    return;
  }

  int highCounts = constrain(round(dutyPercent * 10.0), 1, regulatorPwmTop + 1);
  OCR3C = highCounts - 1;
  TCCR3A = (TCCR3A & ~_BV(COM3C0)) | _BV(COM3C1);
}

void loop() {
  handleSerialCommands();

  unsigned long currentMicros = micros();
  updateStepper(currentMicros);

  if (currentMicros - previousMicros >= interval) {
    previousMicros += interval;

    updateTest();
    updateRegulatorControl();
    writeRegulatorPwm(regulatorSetting);
    updateValvePulse();
    readSensors();
    writeSerialData();

    loopCounter++;
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char incoming = Serial.read();
    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      processSerialCommand(serialCommandBuffer);
      serialCommandBuffer = "";
      continue;
    }

    if (serialCommandBuffer.length() < serialCommandMaxLength) {
      serialCommandBuffer += incoming;
    } else {
      serialCommandBuffer = "";
      Serial.println("ERROR;COMMAND_TOO_LONG");
    }
  }
}

void processSerialCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }
  command.toUpperCase();

  if (command.startsWith("START")) {
    startTest(command);
  } else if (command == "STOP") {
    stopTest();
  } else if (command == "STREAM_ON") {
    streamContinuously = true;
  } else if (command == "STREAM_OFF") {
    streamContinuously = false;
  } else if (command.startsWith("SET_PRESSURE")) {
    setTargetPressure(command);
  } else if (command.startsWith("SET_FLOW_THRESHOLD")) {
    setFlowCaptureThreshold(command);
  } else if (command.startsWith("SET_PULSE_DURATION")) {
    setPulseDuration(command);
  } else if (command.startsWith("PULSE")) {
    startManualPulse(command);
  } else if (command.startsWith("MOTOR_ENABLE")) {
    setStepperEnabled(commandValue(command) > 0.0);
  } else if (command.startsWith("MOTOR_SPEED")) {
    setStepperSpeed(command);
  } else if (command.startsWith("MOTOR_MOVE")) {
    startStepperMove(command);
  } else if (command.startsWith("MOTOR_ABS")) {
    startStepperAbsoluteMove(command);
  } else if (command == "MOTOR_HOME") {
    startStepperHome();
  } else if (command == "MOTOR_ZERO") {
    setStepperZero();
  } else if (command == "MOTOR_POS") {
    printStepperPosition("POSITION");
  } else if (command == "MOTOR_STOP") {
    stopStepper();
  }
}

void startTest(String command) {
  float requestedStartPressure = commandValueAt(command, 0, 0.50);
  float requestedEndPressure = commandValueAt(command, 1, 0.80);
  int requestedRepeats = constrain(round(commandValueAt(command, 2, defaultTestPulsesPerPressure)), 1, maxTestPulsesPerPressure);
  int requestedMask = round(commandValueAt(command, 3, allValveMask));
  requestedMask = requestedMask & allValveMask;
  int requestedStartStep = constrain(round(requestedStartPressure / testPressureStepBar), 0, maxPressureStep);
  int requestedEndStep = constrain(round(requestedEndPressure / testPressureStepBar), 0, maxPressureStep);
  if (requestedMask == 0) {
    Serial.println("TEST;ERROR;NO_VALVES");
    return;
  }
  if (requestedEndStep < requestedStartStep) {
    int swapStep = requestedStartStep;
    requestedStartStep = requestedEndStep;
    requestedEndStep = swapStep;
  }

  closeValves();
  testStartPressureStep = requestedStartStep;
  testEndPressureStep = requestedEndStep;
  testPulsesPerPressure = requestedRepeats;
  testValveMask = requestedMask;
  pressureIndex = testStartPressureStep;
  pulseCounter = 0;
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 0;
  loopCounter = 0;
  targetRegulatorPressure = pressureIndex * testPressureStepBar;
  activeValveMask = testValveMask;
  pulseRequested = false;
  writeSamples = false;
  testRunning = true;
  operatingMode = MODE_TEST;

  Serial.println("MODE;TEST");
  Serial.print("TEST;RANGE;START;");
  Serial.print(testStartPressureStep * testPressureStepBar);
  Serial.print(";END;");
  Serial.print(testEndPressureStep * testPressureStepBar);
  Serial.print(";STEP;");
  Serial.print(testPressureStepBar);
  Serial.print(";PULSES;");
  Serial.print(testPulsesPerPressure);
  Serial.print(";MASK;");
  Serial.println(testValveMask);
  Serial.print("time");
  Serial.print(";");
  Serial.print("target regulator pressure");
  Serial.print(";");
  Serial.print("pressure before valve");
  Serial.print(";");
  Serial.print("regulator feedback pressure");
  Serial.print(";");
  Serial.print("regulator pwm");
  Serial.print(";");
  Serial.print("valves open");
  Serial.print(";");
  Serial.println("flow");
}

void stopTest() {
  testRunning = false;
  pulseRequested = false;
  writeSamples = false;
  flowCaptureActive = false;
  flowCaptureDone = false;
  operatingMode = MODE_IDLE;
  closeValves();
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 0;
  targetRegulatorPressure = 0.0;
  regulatorSetting = 0;
  writeRegulatorPwm(regulatorSetting);
  Serial.println("STOPPED");
}

void finishTest() {
  testRunning = false;
  pulseRequested = false;
  writeSamples = false;
  flowCaptureActive = false;
  flowCaptureDone = false;
  operatingMode = MODE_MANUAL;
  closeValves();
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 0;
  targetRegulatorPressure = testEndPressureStep * testPressureStepBar;
  updateRegulatorControl();
  writeRegulatorPwm(regulatorSetting);
  Serial.println("STOPPED");
}

void setTargetPressure(String command) {
  float requestedPressure = commandValue(command);
  testRunning = false;
  pulseRequested = false;
  writeSamples = false;
  flowCaptureActive = false;
  flowCaptureDone = false;
  operatingMode = MODE_MANUAL;
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 19;
  loopCounter = 0;
  closeValves();
  targetRegulatorPressure = constrain(requestedPressure, 0.0, regulatorMaxPressure);
  updateRegulatorControl();
  writeRegulatorPwm(regulatorSetting);

  Serial.print("MODE;MANUAL;SETPOINT;");
  Serial.print(targetRegulatorPressure);
  Serial.print(";PWM;");
  Serial.println(regulatorSetting);
  readSensors();
  printMeasurementLine();
}

void setFlowCaptureThreshold(String command) {
  float requestedThreshold = commandValue(command);
  flowCaptureThresholdNlMin = constrain(requestedThreshold, 0.0, flowSensorMaxNlMin);

  Serial.print("FLOW_THRESHOLD;SET;");
  Serial.println(flowCaptureThresholdNlMin, 3);
}

void setPulseDuration(String command) {
  float requestedDurationMs = commandValue(command);
  int requestedTicks = round(requestedDurationMs / sampleIntervalMs);
  valvePulseOpenTicks = constrain(requestedTicks, 1, maxValvePulseOpenTicks);

  Serial.print("PULSE_DURATION;SET;");
  Serial.println(valvePulseOpenTicks * sampleIntervalMs, 3);
}

void startManualPulse(String command) {
  int requestedMask = round(commandValue(command));
  requestedMask = requestedMask & allValveMask;

  if (requestedMask == 0) {
    Serial.println("PULSE;ERROR;NO_VALVES");
    return;
  }

  testRunning = false;
  operatingMode = MODE_MANUAL;
  closeValves();
  activeValveMask = requestedMask;
  pulseRequested = true;
  writeSamples = false;
  flowCaptureActive = false;
  flowCaptureDone = false;
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 0;
  loopCounter = 0;

  Serial.print("PULSE;START;");
  Serial.println(activeValveMask);
}

void setStepperEnabled(bool enabled) {
  stepperEnabled = enabled;
  if (!stepperEnabled) {
    stepperRemainingSteps = 0;
    stepperPulseActive = false;
    stepperHoming = false;
    digitalWrite(stepperStepPin, stepperStepIdleSignal);
  }
  digitalWrite(stepperEnablePin, stepperEnabled ? stepperEnableSignal : stepperDisableSignal);

  Serial.print("MOTOR;ENABLED;");
  Serial.println(stepperEnabled);
}

void setStepperSpeed(String command) {
  int requestedSpeed = round(commandValue(command));
  stepperSpeed = constrain(requestedSpeed, 1, 5000);
  stepperStepIntervalMicros = 1000000UL / stepperSpeed;

  Serial.print("MOTOR;SPEED;");
  Serial.println(stepperSpeed);
}

void startStepperMove(String command) {
  long requestedSteps = round(commandValue(command));

  if (!stepperEnabled) {
    Serial.println("MOTOR;ERROR;DISABLED");
    return;
  }

  if (requestedSteps == 0) {
    Serial.println("MOTOR;ERROR;NO_STEPS");
    return;
  }

  if (requestedSteps > 0) {
    stepperHoming = false;
    beginStepperMove(requestedSteps);
  } else {
    stepperHoming = false;
    beginStepperMove(requestedSteps);
  }

  Serial.print("MOTOR;MOVE;");
  Serial.print(requestedSteps);
  Serial.print(";SPEED;");
  Serial.println(stepperSpeed);
}

void startStepperAbsoluteMove(String command) {
  long targetSteps = round(commandValue(command));

  if (!stepperReferenced) {
    Serial.println("MOTOR;ERROR;NOT_REFERENCED");
    return;
  }

  long relativeSteps = targetSteps - stepperCurrentPosition;
  if (relativeSteps == 0) {
    printStepperPosition("DONE");
    return;
  }

  stepperHoming = false;
  startStepperMoveSteps(relativeSteps, "ABS", targetSteps);
}

void startStepperHome() {
  if (!stepperEnabled) {
    Serial.println("MOTOR;ERROR;DISABLED");
    return;
  }

  if (isStepperLimitPressed()) {
    setStepperZero();
    printStepperPosition("HOME_DONE");
    return;
  }

  stepperHoming = true;
  beginStepperMove(-2147483647L);
  Serial.print("MOTOR;HOME;SPEED;");
  Serial.println(stepperSpeed);
}

void setStepperZero() {
  stepperCurrentPosition = 0;
  stepperReferenced = true;
  printStepperPosition("ZERO");
}

void stopStepper() {
  stepperRemainingSteps = 0;
  stepperPulseActive = false;
  stepperHoming = false;
  digitalWrite(stepperStepPin, stepperStepIdleSignal);
  Serial.println("MOTOR;STOPPED");
  printStepperPosition("POSITION");
}

void updateStepper(unsigned long currentMicros) {
  bool limitPressed = isStepperLimitPressed();
  if (limitPressed && !lastStepperLimitPressed) {
    stepperCurrentPosition = 0;
    stepperReferenced = true;
    printStepperPosition("LIMIT");
  }
  lastStepperLimitPressed = limitPressed;

  if (limitPressed && stepperRemainingSteps > 0 && stepperMoveDirection < 0) {
    stepperRemainingSteps = 0;
    stepperPulseActive = false;
    digitalWrite(stepperStepPin, stepperStepIdleSignal);
    stepperCurrentPosition = 0;
    stepperReferenced = true;
    if (stepperHoming) {
      stepperHoming = false;
      printStepperPosition("HOME_DONE");
    } else {
      printStepperPosition("LIMIT_STOP");
    }
    return;
  }

  if (stepperPulseActive) {
    if (currentMicros - stepperPulseStartMicros >= stepperPulseWidthMicros) {
      digitalWrite(stepperStepPin, stepperStepIdleSignal);
      stepperPulseActive = false;
    }
    return;
  }

  if (!stepperEnabled || stepperRemainingSteps <= 0) {
    return;
  }

  if (currentMicros - stepperLastStepMicros >= stepperStepIntervalMicros) {
    digitalWrite(stepperStepPin, stepperStepActiveSignal);
    stepperPulseStartMicros = currentMicros;
    stepperLastStepMicros = currentMicros;
    stepperRemainingSteps--;
    stepperCurrentPosition += stepperMoveDirection;

    if (stepperRemainingSteps == 0) {
      stepperHoming = false;
      printStepperPosition("DONE");
    }
    stepperPulseActive = true;
  }
}

void beginStepperMove(long signedSteps) {
  stepperMoveDirection = signedSteps > 0 ? 1 : -1;
  digitalWrite(stepperDirPin, stepperMoveDirection > 0 ? stepperPositiveDirSignal : stepperNegativeDirSignal);
  stepperRemainingSteps = abs(signedSteps);
  stepperPulseActive = false;
  digitalWrite(stepperStepPin, stepperStepIdleSignal);
  stepperLastStepMicros = micros();
}

void startStepperMoveSteps(long relativeSteps, const char *label, long targetSteps) {
  if (!stepperEnabled) {
    Serial.println("MOTOR;ERROR;DISABLED");
    return;
  }

  beginStepperMove(relativeSteps);
  Serial.print("MOTOR;");
  Serial.print(label);
  Serial.print(";");
  Serial.print(targetSteps);
  Serial.print(";MOVE;");
  Serial.print(relativeSteps);
  Serial.print(";SPEED;");
  Serial.println(stepperSpeed);
}

bool isStepperLimitPressed() {
  return digitalRead(stepperLimitPin) == stepperLimitPressedSignal;
}

void printStepperPosition(const char *eventName) {
  Serial.print("MOTOR;");
  Serial.print(eventName);
  Serial.print(";POS;");
  Serial.print(stepperCurrentPosition);
  Serial.print(";MM;");
  Serial.print(stepperCurrentPosition * stepperMmPerStep);
  Serial.print(";REF;");
  Serial.println(stepperReferenced);
}

float commandValue(String command) {
  int separatorIndex = command.indexOf(':');
  if (separatorIndex < 0) {
    separatorIndex = command.indexOf(' ');
  }
  if (separatorIndex < 0) {
    return 0.0;
  }
  return command.substring(separatorIndex + 1).toFloat();
}

float commandValueAt(String command, int valueIndex, float defaultValue) {
  int separatorIndex = command.indexOf(':');
  if (separatorIndex < 0) {
    separatorIndex = command.indexOf(' ');
  }
  if (separatorIndex < 0) {
    return defaultValue;
  }

  int valueStart = separatorIndex + 1;
  for (int i = 0; i < valueIndex; i++) {
    valueStart = command.indexOf(':', valueStart);
    if (valueStart < 0) {
      return defaultValue;
    }
    valueStart++;
  }

  int valueEnd = command.indexOf(':', valueStart);
  if (valueEnd < 0) {
    valueEnd = command.length();
  }
  String valueText = command.substring(valueStart, valueEnd);
  valueText.trim();
  if (valueText.length() == 0) {
    return defaultValue;
  }
  return valueText.toFloat();
}

void readSensors() {
  rawPressureBeforeValve = analogRead(pressureBeforeValve_InPin);
  rawRegulatorFeedback = analogRead(regulatorFeedback_InPin);
  rawFlow = analogRead(flowSensor_InPin);

  voltagePressureBeforeValve = 5.0 * (rawPressureBeforeValve / 1023.0);
  voltageRegulatorFeedback = 5.0 * (rawRegulatorFeedback / 1023.0);
  voltageFlow = 5.0 * (rawFlow / 1023.0);

  pressureBeforeValve = (voltagePressureBeforeValve - 1.0) * 2.5;
  // The VEAB outputs 0-10 V for 0-6 bar. A 1:1 divider reduces this to
  // 0-5 V at A2, so the ADC voltage maps directly to 0-6 bar.
  regulatorPressure = voltageRegulatorFeedback * regulatorMaxPressure / 5.0;
  float flowFraction = (voltageFlow - flowSensorMinVoltage) / (flowSensorMaxVoltage - flowSensorMinVoltage);
  flowFraction = constrain(flowFraction, 0.0, 1.0);
  flow = flowSensorMinNlMin + flowFraction * (flowSensorMaxNlMin - flowSensorMinNlMin);
}

void updateTest() {
  if (operatingMode != MODE_TEST) {
    return;
  }

  targetRegulatorPressure = pressureIndex * testPressureStepBar;
  waitTime = pulseCounter > 0 ? testRepeatWaitTicks : testPressureStepWaitTicks;

  if (loopCounter > waitTime) {
    if (pulseCounter < testPulsesPerPressure) {
      activeValveMask = testValveMask;
      pulseRequested = true;
      pulseCounter++;
    } else {
      pressureIndex++;
      pulseCounter = 0;
      if (pressureIndex > testEndPressureStep) {
        finishTest();
      } else {
        targetRegulatorPressure = pressureIndex * testPressureStepBar;
      }
    }
    loopCounter = 0;
  }
}

void updateRegulatorControl() {
  regulatorSetting = constrain(round(feedforwardForPressure(targetRegulatorPressure)), 0, 255);
}

float feedforwardForPressure(float targetPressure) {
  targetPressure = constrain(targetPressure, 0.0, regulatorMaxPressure);
  if (targetPressure <= 0.0) {
    return 0.0;
  }

  float correctedCommandPressure =
    (targetPressure - regulatorCommandOffsetBar) / regulatorCommandGain;
  correctedCommandPressure = constrain(correctedCommandPressure, 0.0, regulatorMaxPressure);
  return 255.0 * correctedCommandPressure / regulatorMaxPressure;
}

void updateValvePulse() {
  if (!pulseRequested) {
    return;
  }

  valvePulseCounter++;

  if (valvePulseCounter <= valvePulseOpenTicks) {
    if (valvePulseCounter == 1) {
      valveOpenStartMicros = micros();
      startFlowCapture();
      openValves();
      writeSamples = true;
      sampleCounter = 0;
    }
  } else {
    unsigned long valveCloseMicros = micros();
    valveOpenDurationMicros = valveCloseMicros - valveOpenStartMicros;
    closeValves();
    valvePulseCounter = 0;
    pulseRequested = false;
    Serial.print("PULSE;DONE;");
    Serial.print(activeValveMask);
    Serial.print(";DURATION_US;");
    Serial.print(valveOpenDurationMicros);
    Serial.print(";DURATION_MS;");
    Serial.println(valveOpenDurationMicros / 1000.0, 3);
  }
}

void writeSerialData() {
  if (!writeSamples) {
    if (!streamContinuously) {
      return;
    }

    streamCounter++;
    if (streamCounter < 20) {
      return;
    }
    streamCounter = 0;
  } else {
    streamCounter = 0;
  }

  printMeasurementLine();

  if (writeSamples) {
    updateFlowCaptureSummary();
    sampleCounter++;
    if (sampleCounter > 99) {
      finishFlowCapture();
      writeSamples = false;
      sampleCounter = 0;
    }
  }
}

void printMeasurementLine() {
  Serial.print(millis());
  Serial.print(";");
  Serial.print(targetRegulatorPressure);
  Serial.print(";");
  Serial.print(pressureBeforeValve);
  Serial.print(";");
  Serial.print(regulatorPressure);
  Serial.print(";");
  Serial.print(regulatorSetting);
  Serial.print(";");
  Serial.print(valvesOpen);
  Serial.print(";");
  Serial.println(flow);
}

void startFlowCapture() {
  flowCaptureActive = true;
  flowCaptureDone = false;
  flowCapturePeakSeen = false;
  flowCaptureSampleCount = 0;
  flowCaptureQuietCount = 0;
  flowCaptureBaseline = flow;
  flowCaptureMax = 0.0;
  flowCaptureVolumeL = 0.0;
}

void updateFlowCaptureSummary() {
  if (!flowCaptureActive || flowCaptureDone) {
    return;
  }

  float correctedFlow = max(0.0, flow - flowCaptureBaseline);
  if (correctedFlow > flowCaptureMax) {
    flowCaptureMax = correctedFlow;
  }

  flowCaptureSampleCount++;
  flowCaptureVolumeL += correctedFlow * (sampleIntervalMs / 60000.0);

  bool aboveBaseline = correctedFlow > flowCaptureThresholdNlMin;

  if (aboveBaseline) {
    flowCapturePeakSeen = true;
    flowCaptureQuietCount = 0;
  } else if (flowCapturePeakSeen && flowCaptureSampleCount >= flowCaptureMinSamples) {
    flowCaptureQuietCount++;
  }

  if (flowCapturePeakSeen && flowCaptureQuietCount >= flowCaptureQuietSamples) {
    finishFlowCapture();
  }
}

void finishFlowCapture() {
  if (!flowCaptureActive || flowCaptureDone) {
    return;
  }

  flowCaptureDone = true;
  flowCaptureActive = false;

  Serial.print("PULSE;FLOW_DONE;SAMPLES;");
  Serial.print(flowCaptureSampleCount);
  Serial.print(";DURATION_MS;");
  Serial.print(flowCaptureSampleCount * sampleIntervalMs, 3);
  Serial.print(";MAX_FLOW;");
  Serial.print(flowCaptureMax);
  Serial.print(";BASELINE_FLOW;");
  Serial.print(flowCaptureBaseline);
  Serial.print(";VOLUME_L;");
  Serial.println(flowCaptureVolumeL, 6);
}

void openValves() {
  valvesOpen = true;
  for (int i = 0; i < valveCount; i++) {
    if (activeValveMask & (1 << i)) {
      digitalWrite(valvePins[i], valveOpenSignal);
    } else {
      digitalWrite(valvePins[i], valveClosedSignal);
    }
  }
}

void closeValves() {
  valvesOpen = false;
  for (int i = 0; i < valveCount; i++) {
    digitalWrite(valvePins[i], valveClosedSignal);
  }
}
