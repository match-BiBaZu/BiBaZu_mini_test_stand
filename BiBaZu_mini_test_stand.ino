const int flowSensor_InPin = A1;
const int pressureBeforeValve_InPin = A0;
const int regulatorFeedback_InPin = A2;  // VEAB black output wire
const int regulatorOutPin = 2;           // VEAB input wire, PWM output

const int valvePins[] = {4, 5, 6, 7};
const int valveCount = sizeof(valvePins) / sizeof(valvePins[0]);
const int allValveMask = (1 << valveCount) - 1;
const int valveOpenSignal = HIGH;
const int valveClosedSignal = LOW;

const int stepperStepPin = 8;
const int stepperDirPin = 9;
const int stepperEnablePin = 10;
const int stepperStepActiveSignal = LOW;
const int stepperStepIdleSignal = HIGH;
const int stepperPositiveDirSignal = HIGH;
const int stepperNegativeDirSignal = LOW;
const int stepperEnableSignal = HIGH;
const int stepperDisableSignal = LOW;
const unsigned int stepperPulseWidthMicros = 5;


const float pressureTargets[] = {0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80};
const int pressureFeedforwardPwm[] = {35, 55, 71, 86, 100, 112, 124, 136, 157, 176, 195, 212};
const int pressureSettingCount = sizeof(pressureTargets) / sizeof(pressureTargets[0]);
const float regulatorMaxPressure = 5.0;  // 255 PWM corresponds to 5 bar regulator setpoint.

// SFAH-100B analog scaling for the white analog output configured to 1-5 V.
// 100B is treated as the bidirectional -100 ... +100 l/min measuring range.
const float flowSensorMinVoltage = 1.0;
const float flowSensorMaxVoltage = 5.0;
const float flowSensorMinNlMin = -100.0;
const float flowSensorMaxNlMin = 100.0;

int regulatorSetting = 0;
int pressureIndex = 0;
int pulseCounter = 0;
int valvePulseCounter = 0;
int sampleCounter = 0;
int streamCounter = 0;
int loopCounter = 0;
int waitTime = 1999;
int activeValveMask = allValveMask;
int stepperSpeed = 400;

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

bool valvesOpen = false;
bool pulseRequested = false;
bool testRunning = false;
bool writeSamples = false;
bool streamContinuously = false;
bool stepperEnabled = false;
bool stepperPulseActive = false;

long stepperRemainingSteps = 0;
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

void setup() {
  Serial.begin(230400);

  pinMode(regulatorOutPin, OUTPUT);
  for (int i = 0; i < valveCount; i++) {
    pinMode(valvePins[i], OUTPUT);
    digitalWrite(valvePins[i], valveClosedSignal);
  }
  pinMode(stepperStepPin, OUTPUT);
  pinMode(stepperDirPin, OUTPUT);
  pinMode(stepperEnablePin, OUTPUT);

  analogWrite(regulatorOutPin, 0);
  digitalWrite(stepperStepPin, stepperStepIdleSignal);
  digitalWrite(stepperDirPin, stepperPositiveDirSignal);
  digitalWrite(stepperEnablePin, stepperDisableSignal);
  Serial.println("READY");
  Serial.println("Send START from the GUI to begin the test.");
}

void loop() {
  handleSerialCommands();

  unsigned long currentMicros = micros();
  updateStepper(currentMicros);

  if (currentMicros - previousMicros >= interval) {
    previousMicros += interval;

    readSensors();
    updateTest();
    updateRegulatorControl();
    analogWrite(regulatorOutPin, regulatorSetting);
    updateValvePulse();
    writeSerialData();

    loopCounter++;
  }
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "START") {
    startTest();
  } else if (command == "STOP") {
    stopTest();
  } else if (command == "STREAM_ON") {
    streamContinuously = true;
  } else if (command == "STREAM_OFF") {
    streamContinuously = false;
  } else if (command.startsWith("SET_PRESSURE")) {
    setTargetPressure(command);
  } else if (command.startsWith("PULSE")) {
    startManualPulse(command);
  } else if (command.startsWith("MOTOR_ENABLE")) {
    setStepperEnabled(commandValue(command) > 0.0);
  } else if (command.startsWith("MOTOR_SPEED")) {
    setStepperSpeed(command);
  } else if (command.startsWith("MOTOR_MOVE")) {
    startStepperMove(command);
  } else if (command == "MOTOR_STOP") {
    stopStepper();
  }
}

void startTest() {
  closeValves();
  pressureIndex = 0;
  pulseCounter = 0;
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 0;
  loopCounter = 0;
  targetRegulatorPressure = pressureTargets[pressureIndex];
  activeValveMask = allValveMask;
  pulseRequested = false;
  writeSamples = false;
  testRunning = true;
  operatingMode = MODE_TEST;

  Serial.println("MODE;TEST");
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
  operatingMode = MODE_IDLE;
  closeValves();
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 0;
  targetRegulatorPressure = 0.0;
  regulatorSetting = 0;
  analogWrite(regulatorOutPin, regulatorSetting);
  Serial.println("STOPPED");
}

void setTargetPressure(String command) {
  float requestedPressure = commandValue(command);
  testRunning = false;
  pulseRequested = false;
  writeSamples = false;
  operatingMode = MODE_MANUAL;
  valvePulseCounter = 0;
  sampleCounter = 0;
  streamCounter = 19;
  loopCounter = 0;
  closeValves();
  targetRegulatorPressure = constrain(requestedPressure, 0.0, regulatorMaxPressure);
  updateRegulatorControl();
  analogWrite(regulatorOutPin, regulatorSetting);

  Serial.print("MODE;MANUAL;SETPOINT;");
  Serial.print(targetRegulatorPressure);
  Serial.print(";PWM;");
  Serial.println(regulatorSetting);
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
    digitalWrite(stepperDirPin, stepperPositiveDirSignal);
    stepperRemainingSteps = requestedSteps;
  } else {
    digitalWrite(stepperDirPin, stepperNegativeDirSignal);
    stepperRemainingSteps = -requestedSteps;
  }

  stepperPulseActive = false;
  digitalWrite(stepperStepPin, stepperStepIdleSignal);
  stepperLastStepMicros = micros();

  Serial.print("MOTOR;MOVE;");
  Serial.print(requestedSteps);
  Serial.print(";SPEED;");
  Serial.println(stepperSpeed);
}

void stopStepper() {
  stepperRemainingSteps = 0;
  stepperPulseActive = false;
  digitalWrite(stepperStepPin, stepperStepIdleSignal);
  Serial.println("MOTOR;STOPPED");
}

void updateStepper(unsigned long currentMicros) {
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

    if (stepperRemainingSteps == 0) {
      Serial.println("MOTOR;DONE");
    }
    stepperPulseActive = true;
  }
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

void readSensors() {
  rawPressureBeforeValve = analogRead(pressureBeforeValve_InPin);
  rawRegulatorFeedback = analogRead(regulatorFeedback_InPin);
  rawFlow = analogRead(flowSensor_InPin);

  voltagePressureBeforeValve = 5.0 * (rawPressureBeforeValve / 1023.0);
  voltageRegulatorFeedback = 5.0 * (rawRegulatorFeedback / 1023.0);
  voltageFlow = 5.0 * (rawFlow / 1023.0);

  pressureBeforeValve = (voltagePressureBeforeValve - 1.0) * 2.5;
  regulatorPressure = (voltageRegulatorFeedback - 1.0) * 5.0 / 4.0;
  flow = flowSensorMinNlMin + ((voltageFlow - flowSensorMinVoltage) / (flowSensorMaxVoltage - flowSensorMinVoltage)) * (flowSensorMaxNlMin - flowSensorMinNlMin);
}

void updateTest() {
  if (operatingMode != MODE_TEST) {
    return;
  }

  targetRegulatorPressure = pressureTargets[pressureIndex];
  waitTime = pulseCounter > 0 ? 999 : 1999;

  if (loopCounter > waitTime) {
    if (pulseCounter < 10) {
      activeValveMask = allValveMask;
      pulseRequested = true;
      pulseCounter++;
    } else {
      pressureIndex++;
      pulseCounter = 0;
      if (pressureIndex >= pressureSettingCount) {
        stopTest();
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
  return 255.0 * targetPressure / regulatorMaxPressure;
}

void updateValvePulse() {
  if (!pulseRequested) {
    return;
  }

  valvePulseCounter++;

  if (valvePulseCounter < 21) {
    if (valvePulseCounter == 1) {
      openValves();
      writeSamples = true;
      sampleCounter = 0;
    }
  } else {
    closeValves();
    valvePulseCounter = 0;
    pulseRequested = false;
    Serial.print("PULSE;DONE;");
    Serial.println(activeValveMask);
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

  if (writeSamples) {
    sampleCounter++;
    if (sampleCounter > 99) {
      writeSamples = false;
      sampleCounter = 0;
    }
  }
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
