//Conveyor Belt Sorting System with Color Reading Sensor
//This is the code used for the Final Year Project. It serves as the brain of the project and conducts the following tasks:

// 1. Homing all actuators and servo to their home base. 
// 2. Classification: the system asks the user to present each color block underneath the sensor and confirm its presence through the serial interface. These serve as margins for threshold tolerances and fallback feature. Once done, the user can begin the belt.
// 3. Belt Running: the belt will run continously throughout the project, and is only stopped for sensor readings and when sorting with pushers. The belt has an auto-adjust feature, where if the belt does not read the color of the block well, it will slightly shift forward to allow for proper reading.
// 4. TCS3200 Sensor: this sensor is used as both an object detector and color reading sensor. It uses multi-sample evaluation and conducts brightness averaging to classify the block.
// 5. Pusher Mechanisms: these actuators are used to sort colored objects, and are connected to the Motor Driver boards that power them.
// 6. Servo Mechanism: the servo is used as a guiding tool to sort light and dark blocks at the end of the belt.

// The above points are very general descriptions of the system, and the following code has comments scattered throughout to help in navigating through the code.

#include <Arduino.h>
#include <Servo.h>

// Lets define the color sensor pins to the Arduino microcontroller pins.
const int S0_PIN = 53;
const int S1_PIN = 52;
const int S2_PIN = 51;
const int S3_PIN = 50;
const int OUT_PIN = 49;
const int LED_PIN = 13;  // sensor LED board control, active LOW on my module

// Motor driver 1: red and yellow pushers
const int AIN1 = 22;
const int AIN2 = 23;
const int PWMA = 2;

const int BIN1 = 24;
const int BIN2 = 25;
const int PWMB = 3;

const int STBY1 = 26;

// Motor driver 2: blue pusher and the belt
const int CIN1 = 27;
const int CIN2 = 28;
const int PWMC = 4;

const int DIN1 = 29;
const int DIN2 = 30;
const int PWMD = 5;

const int STBY2 = 31;

Servo diverterServo;

const int SERVO_PIN = 32;

// These are physical angles, so they were tuned by trial and error.
const int SERVO_CENTER = 100;
const int SERVO_LEFT = 60;
const int SERVO_RIGHT = 125;

const unsigned long SERVO_HOLD_MS = 1000;
const unsigned long SERVO_TRAVEL_DELAY_MS = 8000;
const unsigned long SERVO_RETURN_SETTLE_MS = 300;

const unsigned long COLOR_READ_TIME_MS = 1000;
const unsigned long COLOR_SAMPLE_GAP_MS = 100;

const int COLOR_LED_ON = LOW;
const int COLOR_LED_OFF = HIGH;

// The first sample reading is ignored as its reading is not usually correct due to motion before settling.
const int WINDOW_IGNORE_FIRST_SAMPLES = 1;

const int ARRIVAL_CONFIRM_COUNT_COLOR = 2;
const int ARRIVAL_CONFIRM_COUNT_DARK = 4;

const unsigned long ARRIVAL_COOLDOWN_MS = 200;
const unsigned long OBJECT_SETTLE_MS = 100;
const unsigned long POST_ACTION_RESTART_DELAY_MS = 30;

unsigned long arrivalCooldownUntil = 0;
int colourArrivalStreak = 0;
int darkArrivalStreak = 0;

const int PUSHER_SPEED = 255;
const unsigned long PUSHER_EXTEND_MS = 1650;
const unsigned long PUSHER_RETRACT_MS = 1650;
const unsigned long PUSHER_PAUSE_MS = 300;
const unsigned long STARTUP_HOME_DELAY_MS = 500;

const int BELT_SPEED = 255;

// Sensor-to-pusher travel times. 
const unsigned long BLUE_TRAVEL_TO_PUSHER_MS = 1850;
const unsigned long YELLOW_TRAVEL_TO_PUSHER_MS = 3800;
const unsigned long RED_TRAVEL_TO_PUSHER_MS = 5500;

const unsigned long PUSH_POSITION_SETTLE_MS = 150;

const int MIN_LIGHT_COUNT = 3;
const int MIN_BLOCK_COUNT = 3;
const int MIN_DARK_COUNT = 4;
const int BELT_CONFIRM_COUNT = 4;

bool waitingForBlockToClear = false;
int beltSeenAgainCount = 0;

const int CALIBRATION_SAMPLES = 10;
const int FIXED_RANGE_MARGIN = 60;
const float RANGE_EXPANSION_FACTOR = 0.25;

const int YELLOW_EXTRA_MARGIN_R = 120;
const int YELLOW_EXTRA_MARGIN_G = 80;
const int YELLOW_EXTRA_MARGIN_B = 80;
const int YELLOW_EXTRA_MARGIN_BR = 100;

const int LIGHT_EXTRA_MARGIN_R = 180;
const int LIGHT_EXTRA_MARGIN_G = 180;
const int LIGHT_EXTRA_MARGIN_B = 180;
const int LIGHT_EXTRA_MARGIN_BR = 220;

// Red was the colour that gave me the most false misses, so this is added as a safety measure.
const int RED_HEURISTIC_MIN_R = 900;
const int RED_HEURISTIC_MIN_BRIGHTNESS = 700;
const int RED_HEURISTIC_DOMINANCE_OVER_G = 220;
const int RED_HEURISTIC_DOMINANCE_OVER_B = 180;

const unsigned long FALLBACK_MAX_RED_SCORE = 700;
const unsigned long FALLBACK_MAX_YELLOW_SCORE = 420;
const unsigned long FALLBACK_MAX_BLUE_SCORE = 280;
const unsigned long FALLBACK_MAX_DARK_SCORE = 170;
const unsigned long FALLBACK_MAX_LIGHT_SCORE = 420;

const int MAX_SORT_JOBS = 12;

enum DetectedColor {
    COLOR_UNKNOWN,
    COLOR_RED,
    COLOR_YELLOW,
    COLOR_BLUE,
    COLOR_BELT,
    COLOR_DARK,
    COLOR_LIGHT
};

enum JobType {
    JOB_NONE,
    JOB_PUSHER,
    JOB_SERVO
};

enum PusherState {
    PUSHER_IDLE,
    PUSHER_SETTLING,
    PUSHER_EXTENDING,
    PUSHER_PAUSING,
    PUSHER_RETRACTING
};

enum ServoState {
    SERVO_IDLE_STATE,
    SERVO_HOLDING,
    SERVO_RETURNING
};

struct RawReading {
    unsigned long r;
    unsigned long g;
    unsigned long b;
    unsigned long brightness;
};

struct ColorProfile {
    const char *name;
    const char *confirmWord;
    DetectedColor color;
    bool calibrated;
    unsigned long rMin, rMax;
    unsigned long gMin, gMax;
    unsigned long bMin, bMax;
    unsigned long brMin, brMax;
    unsigned long rCenter, gCenter, bCenter, brCenter;
};

struct SortJob {
    bool active;
    JobType type;
    DetectedColor color;
    unsigned long triggerTime;
};

ColorProfile profiles[] = {
    {"RED", "red", COLOR_RED, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {"YELLOW", "yellow", COLOR_YELLOW, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {"BLUE", "blue", COLOR_BLUE, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {"BELT", "belt", COLOR_BELT, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {"DARK", "dark", COLOR_DARK, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {"LIGHT", "light", COLOR_LIGHT, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

const int PROFILE_COUNT = sizeof(profiles) / sizeof(profiles[0]);

SortJob sortJobs[MAX_SORT_JOBS];

PusherState pusherState = PUSHER_IDLE;
int activePusherJob = -1;
unsigned long pusherStateStartedAt = 0;
unsigned long beltStoppedAt = 0;

ServoState servoState = SERVO_IDLE_STATE;
int activeServoJob = -1;
unsigned long servoStateStartedAt = 0;

bool sensorStopActive = false;
unsigned long sensorStopStartedAt = 0;

bool beltRunning = false;

const char *colorToString(DetectedColor c);
JobType jobTypeFor(DetectedColor c);
unsigned long travelDelayFor(DetectedColor c);
bool enqueueSortJob(DetectedColor c);
bool enqueueSortJobAt(DetectedColor c, unsigned long baseTime);
void clearJob(int index);
int findDueJob(JobType type, unsigned long nowMs);
void shiftQueuedJobs(unsigned long deltaMs, int ignoreIndex);
void compensateRunningTimers(unsigned long deltaMs);
RawReading takeSingleReading();
DetectedColor classifyCurrentColor(bool printResult = true);
DetectedColor readStationaryColourWindow();
bool detectObjectArrival();

// Reads one selected sensor channel. The result is a frequency value.
unsigned long readColor(byte s2State, byte s3State) {
    digitalWrite(S2_PIN, s2State);
    digitalWrite(S3_PIN, s3State);
    delay(10);

    unsigned long pulseWidth = pulseIn(OUT_PIN, LOW, 100000);

    if (pulseWidth == 0) {
        return 0;
    }

    return 1000000UL / pulseWidth;
}

void stopMotor(int in1, int in2, int pwm) {
    analogWrite(pwm, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void runMotorForward(int in1, int in2, int pwm, int speedVal) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speedVal);
}

void runMotorReverse(int in1, int in2, int pwm, int speedVal) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, speedVal);
}

void runBelt() {
    runMotorForward(DIN1, DIN2, PWMD, BELT_SPEED);
    beltRunning = true;
}

void stopBelt() {
    stopMotor(DIN1, DIN2, PWMD);
    beltRunning = false;
}

void startRedPusherOut() {
    runMotorForward(AIN1, AIN2, PWMA, PUSHER_SPEED);
}

void startYellowPusherOut() {
    runMotorForward(BIN1, BIN2, PWMB, PUSHER_SPEED);
}

void startBluePusherOut() {
    runMotorForward(CIN1, CIN2, PWMC, PUSHER_SPEED);
}

void startRedPusherBack() {
    runMotorReverse(AIN1, AIN2, PWMA, PUSHER_SPEED);
}

void startYellowPusherBack() {
    runMotorReverse(BIN1, BIN2, PWMB, PUSHER_SPEED);
}

void startBluePusherBack() {
    runMotorReverse(CIN1, CIN2, PWMC, PUSHER_SPEED);
}

void stopRedPusher() {
    stopMotor(AIN1, AIN2, PWMA);
}

void stopYellowPusher() {
    stopMotor(BIN1, BIN2, PWMB);
}

void stopBluePusher() {
    stopMotor(CIN1, CIN2, PWMC);
}

void stopPusherForColour(DetectedColor c) {
    if (c == COLOR_RED) stopRedPusher();
    else if (c == COLOR_YELLOW) stopYellowPusher();
    else if (c == COLOR_BLUE) stopBluePusher();
}

void pushOutForColour(DetectedColor c) {
    if (c == COLOR_RED) startRedPusherOut();
    else if (c == COLOR_YELLOW) startYellowPusherOut();
    else if (c == COLOR_BLUE) startBluePusherOut();
}

void pullBackForColour(DetectedColor c) {
    if (c == COLOR_RED) startRedPusherBack();
    else if (c == COLOR_YELLOW) startYellowPusherBack();
    else if (c == COLOR_BLUE) startBluePusherBack();
}

void retractMotorToStandby(int in1, int in2, int pwm, const char *name) {
    Serial.print("Retracting ");
    Serial.print(name);
    Serial.println(" pusher...");

    runMotorReverse(in1, in2, pwm, PUSHER_SPEED);
    delay(PUSHER_RETRACT_MS);
    stopMotor(in1, in2, pwm);
    delay(150);
}

void homeAllActuators() {
    Serial.println("Homing actuators...");

    diverterServo.write(SERVO_CENTER);
    delay(STARTUP_HOME_DELAY_MS);

    retractMotorToStandby(AIN1, AIN2, PWMA, "RED");
    retractMotorToStandby(BIN1, BIN2, PWMB, "YELLOW");
    retractMotorToStandby(CIN1, CIN2, PWMC, "BLUE");

    diverterServo.write(SERVO_CENTER);
    delay(300);

    Serial.println("Homing complete.");
}

void servoToLeftStart() {
    Serial.println("LIGHT detected -> servo left");
    diverterServo.write(SERVO_LEFT);
}

void servoToRightStart() {
    Serial.println("DARK detected -> servo right");
    diverterServo.write(SERVO_RIGHT);
}

void servoReturnCenter() {
    diverterServo.write(SERVO_CENTER);
}

bool beltBusyForServo() {
    return !beltRunning || sensorStopActive || pusherState != PUSHER_IDLE;
}

void beginSensorStop() {
    stopBelt();
    sensorStopStartedAt = millis();
    sensorStopActive = true;
}

void compensateRunningTimers(unsigned long deltaMs) {
    if (deltaMs == 0) return;

    if (pusherState != PUSHER_IDLE) {
        pusherStateStartedAt += deltaMs;
        beltStoppedAt += deltaMs;
    }

    if (servoState != SERVO_IDLE_STATE) {
        servoStateStartedAt += deltaMs;
    }

    Serial.print("Adjusted active timers by ");
    Serial.print(deltaMs);
    Serial.println(" ms.");
}

void finishSensorStopAndShiftJobs() {
    if (!sensorStopActive) return;

    unsigned long stoppedFor = millis() - sensorStopStartedAt;

    shiftQueuedJobs(stoppedFor, -1);
    compensateRunningTimers(stoppedFor);

    Serial.print("Sensor stop lasted ");
    Serial.print(stoppedFor);
    Serial.println(" ms; shifted queued jobs.");

    sensorStopActive = false;
    sensorStopStartedAt = 0;
}

JobType jobTypeFor(DetectedColor c) {
    switch (c) {
        case COLOR_RED:
        case COLOR_YELLOW:
        case COLOR_BLUE:
            return JOB_PUSHER;

        case COLOR_DARK:
        case COLOR_LIGHT:
            return JOB_SERVO;

        default:
            return JOB_NONE;
    }
}

unsigned long travelDelayFor(DetectedColor c) {
    switch (c) {
        case COLOR_RED:
            return RED_TRAVEL_TO_PUSHER_MS;

        case COLOR_YELLOW:
            return YELLOW_TRAVEL_TO_PUSHER_MS;

        case COLOR_BLUE:
            return BLUE_TRAVEL_TO_PUSHER_MS;

        case COLOR_DARK:
        case COLOR_LIGHT:
            return SERVO_TRAVEL_DELAY_MS;

        default:
            return 0;
    }
}

bool enqueueSortJobAt(DetectedColor c, unsigned long baseTime) {
    JobType type = jobTypeFor(c);

    if (type == JOB_NONE) return false;

    int slot = -1;

    for (int i = 0; i < MAX_SORT_JOBS; i++) {
        if (!sortJobs[i].active) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        Serial.println("WARNING: sort queue full, skipping block.");
        return false;
    }

    sortJobs[slot].active = true;
    sortJobs[slot].type = type;
    sortJobs[slot].color = c;
    sortJobs[slot].triggerTime = baseTime + travelDelayFor(c);

    Serial.print("Queued ");
    Serial.print(colorToString(c));
    Serial.print(" job in slot ");
    Serial.print(slot);
    Serial.print(" for ");
    Serial.print(sortJobs[slot].triggerTime);
    Serial.println(" ms.");

    return true;
}

bool enqueueSortJob(DetectedColor c) {
    return enqueueSortJobAt(c, millis());
}

void clearJob(int index) {
    if (index < 0 || index >= MAX_SORT_JOBS) return;

    sortJobs[index].active = false;
    sortJobs[index].type = JOB_NONE;
    sortJobs[index].color = COLOR_UNKNOWN;
    sortJobs[index].triggerTime = 0;
}

int findDueJob(JobType type, unsigned long nowMs) {
    int best = -1;
    unsigned long earliestTime = 0;

    for (int i = 0; i < MAX_SORT_JOBS; i++) {
        if (!sortJobs[i].active) continue;
        if (sortJobs[i].type != type) continue;
        if ((long)(nowMs - sortJobs[i].triggerTime) < 0) continue;

        if (best == -1 || sortJobs[i].triggerTime < earliestTime) {
            best = i;
            earliestTime = sortJobs[i].triggerTime;
        }
    }

    return best;
}

void shiftQueuedJobs(unsigned long deltaMs, int ignoreIndex) {
    if (deltaMs == 0) return;

    for (int i = 0; i < MAX_SORT_JOBS; i++) {
        if (!sortJobs[i].active) continue;
        if (i == ignoreIndex) continue;

        sortJobs[i].triggerTime += deltaMs;
    }

    Serial.print("Shifted pending jobs by ");
    Serial.print(deltaMs);
    Serial.println(" ms.");
}

void startPusherJob(int jobIndex) {
    if (jobIndex < 0 || jobIndex >= MAX_SORT_JOBS) return;
    if (!sortJobs[jobIndex].active) return;

    activePusherJob = jobIndex;
    pusherState = PUSHER_SETTLING;
    pusherStateStartedAt = millis();
    beltStoppedAt = millis();

    stopBelt();

    Serial.print("Starting pusher job: ");
    Serial.println(colorToString(sortJobs[jobIndex].color));
}

void processPusherJobs() {
    unsigned long nowMs = millis();

    if (sensorStopActive) {
        return;
    }

    if (pusherState == PUSHER_IDLE) {
        int job = findDueJob(JOB_PUSHER, nowMs);

        if (job != -1) startPusherJob(job);

        return;
    }

    if (activePusherJob < 0 || activePusherJob >= MAX_SORT_JOBS) {
        activePusherJob = -1;
        pusherState = PUSHER_IDLE;
        return;
    }

    DetectedColor c = sortJobs[activePusherJob].color;

    switch (pusherState) {
        case PUSHER_SETTLING:
            if (nowMs - pusherStateStartedAt >= PUSH_POSITION_SETTLE_MS) {
                pushOutForColour(c);
                pusherState = PUSHER_EXTENDING;
                pusherStateStartedAt = nowMs;
            }
            break;

        case PUSHER_EXTENDING:
            if (nowMs - pusherStateStartedAt >= PUSHER_EXTEND_MS) {
                stopPusherForColour(c);
                pusherState = PUSHER_PAUSING;
                pusherStateStartedAt = nowMs;
            }
            break;

        case PUSHER_PAUSING:
            if (nowMs - pusherStateStartedAt >= PUSHER_PAUSE_MS) {
                pullBackForColour(c);
                pusherState = PUSHER_RETRACTING;
                pusherStateStartedAt = nowMs;
            }
            break;

        case PUSHER_RETRACTING:
            if (nowMs - pusherStateStartedAt >= PUSHER_RETRACT_MS) {
                stopPusherForColour(c);

                unsigned long beltWasStoppedFor = nowMs - beltStoppedAt;

                Serial.print("Completed pusher job for ");
                Serial.println(colorToString(c));

                clearJob(activePusherJob);
                shiftQueuedJobs(beltWasStoppedFor, activePusherJob);

                activePusherJob = -1;
                pusherState = PUSHER_IDLE;

                runBelt();
            }
            break;

        default:
            break;
    }
}

void startServoJob(int jobIndex) {
    if (jobIndex < 0 || jobIndex >= MAX_SORT_JOBS) return;
    if (!sortJobs[jobIndex].active) return;

    activeServoJob = jobIndex;
    servoStateStartedAt = millis();

    if (sortJobs[jobIndex].color == COLOR_LIGHT) {
        servoToLeftStart();
    } else if (sortJobs[jobIndex].color == COLOR_DARK) {
        servoToRightStart();
    }

    servoState = SERVO_HOLDING;

    Serial.print("Starting servo job: ");
    Serial.println(colorToString(sortJobs[jobIndex].color));
}

void processServoJobs() {
    unsigned long nowMs = millis();

    // Freeze servo timing while the belt is stopped, otherwise the block won't be where we expect.
    if (!beltRunning) {
        return;
    }

    if (servoState == SERVO_IDLE_STATE) {
        if (beltBusyForServo()) return;

        int job = findDueJob(JOB_SERVO, nowMs);

        if (job != -1) startServoJob(job);

        return;
    }

    if (activeServoJob < 0 || activeServoJob >= MAX_SORT_JOBS) {
        activeServoJob = -1;
        servoState = SERVO_IDLE_STATE;
        return;
    }

    switch (servoState) {
        case SERVO_HOLDING:
            if (nowMs - servoStateStartedAt >= SERVO_HOLD_MS) {
                servoReturnCenter();
                servoState = SERVO_RETURNING;
                servoStateStartedAt = nowMs;
            }
            break;

        case SERVO_RETURNING:
            if (nowMs - servoStateStartedAt >= SERVO_RETURN_SETTLE_MS) {
                Serial.print("Completed servo job for ");
                Serial.println(colorToString(sortJobs[activeServoJob].color));

                clearJob(activeServoJob);
                activeServoJob = -1;
                servoState = SERVO_IDLE_STATE;
            }
            break;

        default:
            break;
    }
}

const char *colorToString(DetectedColor c) {
    switch (c) {
        case COLOR_RED:
            return "RED";

        case COLOR_YELLOW:
            return "YELLOW";

        case COLOR_BLUE:
            return "BLUE";

        case COLOR_BELT:
            return "BELT";

        case COLOR_DARK:
            return "DARK";

        case COLOR_LIGHT:
            return "LIGHT";

        default:
            return "UNKNOWN";
    }
}

RawReading takeSingleReading() {
    RawReading x;

    x.r = readColor(LOW, LOW);
    x.b = readColor(LOW, HIGH);
    x.g = readColor(HIGH, HIGH);
    x.brightness = (x.r + x.g + x.b) / 3;

    return x;
}

String readSerialLineBlocking() {
    while (true) {
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            line.toLowerCase();
            return line;
        }

        delay(10);
    }
}

void clearSerialBuffer() {
    while (Serial.available()) {
        Serial.read();
    }
}

void waitForExactCommand(const char *expectedWord) {
    clearSerialBuffer();

    while (true) {
        Serial.print("Type \"");
        Serial.print(expectedWord);
        Serial.println("\" and press Enter to continue.");

        String input = readSerialLineBlocking();

        if (input.equalsIgnoreCase(expectedWord)) {
            Serial.println("Confirmed.");
            return;
        }

        Serial.print("Received \"");
        Serial.print(input);
        Serial.println("\". Try again.");
    }
}

unsigned long safeSubtract(unsigned long value, unsigned long amount) {
    if (amount > value) return 0;

    return value - amount;
}

unsigned long computeMargin(unsigned long minVal, unsigned long maxVal) {
    unsigned long observedRange = maxVal - minVal;
    unsigned long margin = (unsigned long)(observedRange * RANGE_EXPANSION_FACTOR);

    if (margin < FIXED_RANGE_MARGIN) {
        margin = FIXED_RANGE_MARGIN;
    }

    return margin;
}

void expandProfileRanges(ColorProfile &p) {
    unsigned long rMargin = computeMargin(p.rMin, p.rMax);
    unsigned long gMargin = computeMargin(p.gMin, p.gMax);
    unsigned long bMargin = computeMargin(p.bMin, p.bMax);
    unsigned long brMargin = computeMargin(p.brMin, p.brMax);

    p.rMin = safeSubtract(p.rMin, rMargin);
    p.rMax += rMargin;

    p.gMin = safeSubtract(p.gMin, gMargin);
    p.gMax += gMargin;

    p.bMin = safeSubtract(p.bMin, bMargin);
    p.bMax += bMargin;

    p.brMin = safeSubtract(p.brMin, brMargin);
    p.brMax += brMargin;

    // Yellow and light needed a little more room during testing.
    if (p.color == COLOR_YELLOW) {
        p.rMin = safeSubtract(p.rMin, YELLOW_EXTRA_MARGIN_R);
        p.rMax += YELLOW_EXTRA_MARGIN_R;

        p.gMin = safeSubtract(p.gMin, YELLOW_EXTRA_MARGIN_G);
        p.gMax += YELLOW_EXTRA_MARGIN_G;

        p.bMin = safeSubtract(p.bMin, YELLOW_EXTRA_MARGIN_B);
        p.bMax += YELLOW_EXTRA_MARGIN_B;

        p.brMin = safeSubtract(p.brMin, YELLOW_EXTRA_MARGIN_BR);
        p.brMax += YELLOW_EXTRA_MARGIN_BR;
    }

    if (p.color == COLOR_LIGHT) {
        p.rMin = safeSubtract(p.rMin, LIGHT_EXTRA_MARGIN_R);
        p.rMax += LIGHT_EXTRA_MARGIN_R;

        p.gMin = safeSubtract(p.gMin, LIGHT_EXTRA_MARGIN_G);
        p.gMax += LIGHT_EXTRA_MARGIN_G;

        p.bMin = safeSubtract(p.bMin, LIGHT_EXTRA_MARGIN_B);
        p.bMax += LIGHT_EXTRA_MARGIN_B;

        p.brMin = safeSubtract(p.brMin, LIGHT_EXTRA_MARGIN_BR);
        p.brMax += LIGHT_EXTRA_MARGIN_BR;
    }
}

void calibrateProfile(ColorProfile &p) {
    Serial.println();
    Serial.println("==================================================");
    Serial.print("Place the ");
    Serial.print(p.name);
    Serial.println(" reference under the sensor.");
    Serial.println("Keep the belt stopped and keep the object still.");

    waitForExactCommand(p.confirmWord);

    digitalWrite(LED_PIN, COLOR_LED_ON);
    delay(300);

    bool firstSample = true;

    unsigned long rSum = 0;
    unsigned long gSum = 0;
    unsigned long bSum = 0;
    unsigned long brSum = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        RawReading x = takeSingleReading();

        if (firstSample) {
            p.rMin = p.rMax = x.r;
            p.gMin = p.gMax = x.g;
            p.bMin = p.bMax = x.b;
            p.brMin = p.brMax = x.brightness;
            firstSample = false;
        } else {
            if (x.r < p.rMin) p.rMin = x.r;
            if (x.r > p.rMax) p.rMax = x.r;

            if (x.g < p.gMin) p.gMin = x.g;
            if (x.g > p.gMax) p.gMax = x.g;

            if (x.b < p.bMin) p.bMin = x.b;
            if (x.b > p.bMax) p.bMax = x.b;

            if (x.brightness < p.brMin) p.brMin = x.brightness;
            if (x.brightness > p.brMax) p.brMax = x.brightness;
        }

        rSum += x.r;
        gSum += x.g;
        bSum += x.b;
        brSum += x.brightness;

        delay(COLOR_SAMPLE_GAP_MS);
    }

    digitalWrite(LED_PIN, COLOR_LED_OFF);

    p.rCenter = rSum / CALIBRATION_SAMPLES;
    p.gCenter = gSum / CALIBRATION_SAMPLES;
    p.bCenter = bSum / CALIBRATION_SAMPLES;
    p.brCenter = brSum / CALIBRATION_SAMPLES;

    expandProfileRanges(p);

    p.calibrated = true;

    Serial.println("Calibration complete.");
}

void calibrateAllProfiles() {
    Serial.println();
    Serial.println("===== STARTUP CALIBRATION =====");
    Serial.println("Calibrate RED, YELLOW, BLUE, BELT, DARK, then LIGHT.");
    Serial.println("Use Serial Monitor at 9600 baud with Newline selected.");
    Serial.println();

    stopBelt();
    delay(300);

    for (int i = 0; i < PROFILE_COUNT; i++) {
        calibrateProfile(profiles[i]);
    }

    Serial.println();
    Serial.println("All calibration steps complete.");
    Serial.println("Remove all blocks from the track.");

    waitForExactCommand("begin");

    Serial.println("Starting sorter...");
    Serial.println();
}

bool readingInsideProfile(const RawReading &x, const ColorProfile &p) {
    if (!p.calibrated) return false;

    return x.r >= p.rMin && x.r <= p.rMax &&
           x.g >= p.gMin && x.g <= p.gMax &&
           x.b >= p.bMin && x.b <= p.bMax &&
           x.brightness >= p.brMin && x.brightness <= p.brMax;
}

unsigned long absDiffUL(unsigned long a, unsigned long b) {
    return (a > b) ? (a - b) : (b - a);
}

unsigned long profileDistanceScore(const RawReading &x, const ColorProfile &p) {
    return absDiffUL(x.r, p.rCenter) +
           absDiffUL(x.g, p.gCenter) +
           absDiffUL(x.b, p.bCenter) +
           absDiffUL(x.brightness, p.brCenter);
}

unsigned long fallbackThresholdFor(DetectedColor c) {
    switch (c) {
        case COLOR_RED:
            return FALLBACK_MAX_RED_SCORE;

        case COLOR_YELLOW:
            return FALLBACK_MAX_YELLOW_SCORE;

        case COLOR_BLUE:
            return FALLBACK_MAX_BLUE_SCORE;

        case COLOR_DARK:
            return FALLBACK_MAX_DARK_SCORE;

        case COLOR_LIGHT:
            return FALLBACK_MAX_LIGHT_SCORE;

        default:
            return 0;
    }
}

bool canUseFallbackFor(DetectedColor c) {
    return c == COLOR_RED ||
           c == COLOR_YELLOW ||
           c == COLOR_BLUE ||
           c == COLOR_DARK ||
           c == COLOR_LIGHT;
}

bool looksLikeRedAnyway(const RawReading &x) {
    return x.r >= RED_HEURISTIC_MIN_R &&
           x.brightness >= RED_HEURISTIC_MIN_BRIGHTNESS &&
           x.r > x.g + RED_HEURISTIC_DOMINANCE_OVER_G &&
           x.r > x.b + RED_HEURISTIC_DOMINANCE_OVER_B;
}

DetectedColor classifyCurrentColor(bool printResult) {
    RawReading x = takeSingleReading();

    if (printResult) {
        Serial.print("R: ");
        Serial.print(x.r);
        Serial.print(" G: ");
        Serial.print(x.g);
        Serial.print(" B: ");
        Serial.print(x.b);
        Serial.print(" | Brightness: ");
        Serial.print(x.brightness);
        Serial.print(" -> ");
    }

    int exactIndex = -1;
    unsigned long exactScore = 0;

    for (int i = 0; i < PROFILE_COUNT; i++) {
        if (readingInsideProfile(x, profiles[i])) {
            unsigned long score = profileDistanceScore(x, profiles[i]);

            if (exactIndex == -1 || score < exactScore) {
                exactIndex = i;
                exactScore = score;
            }
        }
    }

    if (exactIndex != -1) {
        if (printResult) Serial.println(profiles[exactIndex].name);

        return profiles[exactIndex].color;
    }

    int fallbackIndex = -1;
    unsigned long fallbackScore = 0;

    for (int i = 0; i < PROFILE_COUNT; i++) {
        if (!profiles[i].calibrated) continue;
        if (!canUseFallbackFor(profiles[i].color)) continue;

        unsigned long score = profileDistanceScore(x, profiles[i]);
        unsigned long limit = fallbackThresholdFor(profiles[i].color);

        if (score <= limit && (fallbackIndex == -1 || score < fallbackScore)) {
            fallbackIndex = i;
            fallbackScore = score;
        }
    }

    if (fallbackIndex != -1) {
        if (printResult) {
            Serial.print(profiles[fallbackIndex].name);
            Serial.println(" (fallback)");
        }

        return profiles[fallbackIndex].color;
    }

    if (looksLikeRedAnyway(x)) {
        if (printResult) Serial.println("RED (heuristic)");

        return COLOR_RED;
    }

    if (printResult) Serial.println("UNKNOWN");

    return COLOR_UNKNOWN;
}

bool isSortObjectColour(DetectedColor c) {
    return c == COLOR_RED ||
           c == COLOR_YELLOW ||
           c == COLOR_BLUE ||
           c == COLOR_DARK ||
           c == COLOR_LIGHT;
}

bool detectObjectArrival() {
    if (millis() < arrivalCooldownUntil) {
        return false;
    }

    DetectedColor c = classifyCurrentColor(true);

    if (c == COLOR_RED || c == COLOR_YELLOW || c == COLOR_BLUE || c == COLOR_LIGHT) {
        colourArrivalStreak++;
        darkArrivalStreak = 0;
    } else if (c == COLOR_DARK) {
        darkArrivalStreak++;
        colourArrivalStreak = 0;
    } else {
        colourArrivalStreak = 0;
        darkArrivalStreak = 0;
    }

    if (colourArrivalStreak >= ARRIVAL_CONFIRM_COUNT_COLOR) {
        Serial.println("Object arrival suspected, stopping belt.");
        colourArrivalStreak = 0;
        darkArrivalStreak = 0;
        return true;
    }

    if (darkArrivalStreak >= ARRIVAL_CONFIRM_COUNT_DARK) {
        Serial.println("Dark object arrival suspected, stopping belt.");
        colourArrivalStreak = 0;
        darkArrivalStreak = 0;
        return true;
    }

    return false;
}

DetectedColor readStationaryColourWindow() {
    int redCount = 0;
    int yellowCount = 0;
    int blueCount = 0;
    int beltCount = 0;
    int darkCount = 0;
    int lightCount = 0;
    int unknownCount = 0;
    int sampleNo = 0;

    Serial.println("Reading stationary colour window...");

    digitalWrite(LED_PIN, COLOR_LED_ON);

    unsigned long startedAt = millis();

    while (millis() - startedAt < COLOR_READ_TIME_MS) {
        DetectedColor c = classifyCurrentColor(true);

        if (sampleNo >= WINDOW_IGNORE_FIRST_SAMPLES) {
            switch (c) {
                case COLOR_RED:
                    redCount++;
                    break;

                case COLOR_YELLOW:
                    yellowCount++;
                    break;

                case COLOR_BLUE:
                    blueCount++;
                    break;

                case COLOR_BELT:
                    beltCount++;
                    break;

                case COLOR_DARK:
                    darkCount++;
                    break;

                case COLOR_LIGHT:
                    lightCount++;
                    break;

                default:
                    unknownCount++;
                    break;
            }
        } else {
            Serial.println("Ignoring first sample while things settle.");
        }

        sampleNo++;

        delay(COLOR_SAMPLE_GAP_MS);
    }

    digitalWrite(LED_PIN, COLOR_LED_OFF);

    Serial.print("Counts -> RED: ");
    Serial.print(redCount);
    Serial.print(" YELLOW: ");
    Serial.print(yellowCount);
    Serial.print(" BLUE: ");
    Serial.print(blueCount);
    Serial.print(" BELT: ");
    Serial.print(beltCount);
    Serial.print(" DARK: ");
    Serial.print(darkCount);
    Serial.print(" LIGHT: ");
    Serial.print(lightCount);
    Serial.print(" UNKNOWN: ");
    Serial.println(unknownCount);

    if (lightCount >= MIN_LIGHT_COUNT) {
        Serial.println("Final: LIGHT");
        return COLOR_LIGHT;
    }

    // Small rescue rule: the light block sometimes lands half as UNKNOWN.
    if (lightCount >= 2 && unknownCount >= 1 && lightCount > darkCount) {
        Serial.println("Final: LIGHT (rescued)");
        return COLOR_LIGHT;
    }

    if (redCount >= MIN_BLOCK_COUNT && redCount >= yellowCount && redCount >= blueCount) {
        Serial.println("Final: RED");
        return COLOR_RED;
    }

    if (yellowCount >= MIN_BLOCK_COUNT && yellowCount >= redCount && yellowCount >= blueCount) {
        Serial.println("Final: YELLOW");
        return COLOR_YELLOW;
    }

    if (blueCount >= MIN_BLOCK_COUNT && blueCount >= redCount && blueCount >= yellowCount) {
        Serial.println("Final: BLUE");
        return COLOR_BLUE;
    }

    bool realColourStrongEnough = redCount >= MIN_BLOCK_COUNT ||
                                  yellowCount >= MIN_BLOCK_COUNT ||
                                  blueCount >= MIN_BLOCK_COUNT;

    if (!realColourStrongEnough && darkCount >= MIN_DARK_COUNT) {
        Serial.println("Final: DARK");
        return COLOR_DARK;
    }

    if (beltCount > 0) {
        Serial.println("Final: BELT");
        return COLOR_BELT;
    }

    Serial.println("Final: UNKNOWN");

    return COLOR_UNKNOWN;
}

void setup() {
    Serial.begin(9600);

    pinMode(S0_PIN, OUTPUT);
    pinMode(S1_PIN, OUTPUT);
    pinMode(S2_PIN, OUTPUT);
    pinMode(S3_PIN, OUTPUT);
    pinMode(OUT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(LED_PIN, COLOR_LED_OFF);

    // 20% output scaling for the TCS3200.
    digitalWrite(S0_PIN, LOW);
    digitalWrite(S1_PIN, HIGH);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(CIN1, OUTPUT);
    pinMode(CIN2, OUTPUT);
    pinMode(PWMC, OUTPUT);

    pinMode(DIN1, OUTPUT);
    pinMode(DIN2, OUTPUT);
    pinMode(PWMD, OUTPUT);

    pinMode(STBY1, OUTPUT);
    pinMode(STBY2, OUTPUT);

    digitalWrite(STBY1, HIGH);
    digitalWrite(STBY2, HIGH);

    stopMotor(AIN1, AIN2, PWMA);
    stopMotor(BIN1, BIN2, PWMB);
    stopMotor(CIN1, CIN2, PWMC);
    stopMotor(DIN1, DIN2, PWMD);

    beltRunning = false;

    diverterServo.attach(SERVO_PIN);
    diverterServo.write(SERVO_CENTER);
    delay(500);

    for (int i = 0; i < MAX_SORT_JOBS; i++) {
        clearJob(i);
    }

    homeAllActuators();
    calibrateAllProfiles();

    runBelt();

    Serial.println("System ready");
}

void loop() {
    processPusherJobs();
    processServoJobs();

    if (pusherState != PUSHER_IDLE) {
        delay(10);
        return;
    }

    if (!waitingForBlockToClear) {
        if (!detectObjectArrival()) {
            delay(30);
            return;
        }

        beginSensorStop();
        delay(OBJECT_SETTLE_MS);

        DetectedColor finalColor = readStationaryColourWindow();

        if (finalColor == COLOR_BELT || finalColor == COLOR_UNKNOWN) {
            Serial.println("Stationary read was unreliable; restarting belt.");

            finishSensorStopAndShiftJobs();
            runBelt();

            arrivalCooldownUntil = millis() + ARRIVAL_COOLDOWN_MS;

            delay(POST_ACTION_RESTART_DELAY_MS);
            return;
        }

        Serial.print("Block detected: ");
        Serial.println(colorToString(finalColor));

        finishSensorStopAndShiftJobs();

        enqueueSortJobAt(finalColor, millis() + POST_ACTION_RESTART_DELAY_MS);

        waitingForBlockToClear = true;
        beltSeenAgainCount = 0;

        runBelt();

        arrivalCooldownUntil = millis() + ARRIVAL_COOLDOWN_MS;

        delay(POST_ACTION_RESTART_DELAY_MS);
        return;
    }

    DetectedColor rearmColor = classifyCurrentColor(true);

    if (rearmColor == COLOR_BELT) {
        beltSeenAgainCount++;

        Serial.print("Re-arm check: ");
        Serial.println(beltSeenAgainCount);

        if (beltSeenAgainCount >= BELT_CONFIRM_COUNT) {
            waitingForBlockToClear = false;
            beltSeenAgainCount = 0;

            Serial.println("Ready for next block");
        }
    } else {
        beltSeenAgainCount = 0;
    }

    delay(30);
}