#include <BluetoothSerial.h>
#include <Preferences.h>
#include <math.h>

BluetoothSerial SerialBT;
Preferences prefs;

/* PINS */
// LEDS
#define FAST_LED_PIN 21
#define TURN_LED_PIN 22

// BT restore button: one side to GPIO4, other side to GND
// INPUT_PULLUP => pressed = LOW
#define BT_RESTORE_PIN 4

// Motor driver
#define STBY 23
#define PWMA 19
#define AINI 13
#define AIN2 12
#define PWМВ 18
#define BIN1 14
#define BIN2 27

// QTR analog sensors
const int SENSOR_COUNT = 8;
int qtrPins[SENSOR_COUNT] = {26, 25, 33, 32, 35, 34, 39, 36};
int weights[SENSOR_COUNT] = {7, 5, 3, 1, -1, -3, -5, -7};

/* PWM */
#define PWM_FREQ 20000
#define PM_RES 8

/* STATES */
enum RobotState { STOPPED, RUNNING, CALIBRATING };
enum DriveMode { FAST_MODE, TURN_MODE };
enum ThresholdMode { THRESHOLD_MANUAL, THRESHOLD_AUTO };

RobotState robotState = STOPPED;
DriveMode driveMode = TURN_MODE;
ThresholdMode thresholdMode = THRESHOLD_MANUAL;

/* FLAGS */
bool bluetoothEnabled = false;
bool bootBtDefault = true;
bool raceMode = false;
bool scanEnabled = false;
bool hasCalibration = false;

/* PID */
float skp = 5.0, ski = 0.0, skd = 1.5; // FAST mode PID
float ckp = 8.0, cki = 0.0, ckd = 3.0; // TURN mode PID

/* THRESHOLDS / TRACK LOGIC */
int manualThreshold = 3000;
int autoThresholds[SENSOR_COUNT];
int calMin[SENSOR_COUNT];
int calMax[SENSOR_COUNT];

float st = 0.70; // FAST entry threshold
float ct = 3.80; // TURN force threshold
float dt = 1.20; // TURN force derivative threshold
int fastDelay = 180; // stable time before FAST mode

/* SPEED */
int maxSpeed = 240;
int minSpeed = 80;
float speedFactor = 25.0;
int turnPenalty = 25;
int recoverySpeed = 120;

/* PID MEMORY */
float lastError = 0.0;
float lastSeenError = 0.0;
float sumError = 0.0;
float lastDerivative = 0.0;
int lastBase = 0;
float lastCorrection = 0.0;

/* TIMERS */
unsigned long fastCandidateTimer = 0;
unsigned long scanTimer = 0;

// Button debounce / hold
unsigned long btButtonPressStart = 0;
bool btButtonLatched = false;
const unsigned long BT_BUTTON_HOLD_MS = 250;

/* BUFFERS */
String usbBuffer = "";
String btBuffer = "";

/* SENSOR FRAME */
struct SensorFrame {
    int raw[SENSOR_COUNT];
    int processed[SENSOR_COUNT];
    uint8_t bin[SENSOR_COUNT];
    long weighted = 0;
    long total = 0;
    float error = 0.0;
    bool lineDetected = false;
};

SensorFrame frame;

/* FORWARD DECLARATIONS */
void sendMsg(const String &m);
String onOff(bool v);
String driveModeText();
String thresholdModeText();
int mapClampInt(long x, long inMin, long inMax, long outMin, long outMax);
void blinkPinBlocking(int pin, int count, int onMs = 70, int offMs = 60);
void blinkBothBlocking(int count, int onMs = 70, int offMs = 60);
void surpriseMsg(const String &m);
bool startBluetoothNow();
void stopBluetoothNow(const String &reason = "");
void handleBtRestoreButton();
void saveIntArray(const char *prefix, int *arr);
void loadIntArray(const char *prefix, int *arr);
void saveParams(bool announce = true);
void loadParams();
void showAutoThresholdLine();
void showCalibration();
void showStatus();
void showHelp();
void resetPID();
float computePID(float e, float kp, float ki, float kd);
void stopMotors();
void setMotor(int L, int R);
void victoryDance();
void updateModeLeds();
void captureFrame();
void emitScan();
void setDriveMode(DriveMode m);
void updateDriveMode(float absE, float absD);
void lineFollower();
void clearCalibration();
void runCalibration(unsigned long durationMs = 6000);
void setAndSaveInt(const char *name, int &var, int value);
void setAndSaveFloat(const char *name, float &var, float value, int digits = 3);
void handleCmd(String c);
void readStream(Stream &p, String &buffer);

/* HELPERS */
String onOff(bool v) { return v ? "ON" : "OFF"; }
String driveModeText() { return (driveMode == FAST_MODE) ? "FAST" : "TURN"; }
String thresholdModeText() { return (thresholdMode == THRESHOLD_AUTO) ? "AUTO" : "MANUAL"; }

int mapClampInt(long x, long inMin, long inMax, long outMin, long outMax) {
    if (inMax <= inMin) return (int)outMin;
    long y = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    if (outMin < outMax) {
        if (y < outMin) y = outMin;
        if (y > outMax) y = outMax;
    } else {
        if (y > outMin) y = outMin;
        if (y < outMax) y = outMax;
    }
    return (int)y;
}

void blinkPinBlocking(int pin, int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(pin, HIGH);
        delay(onMs);
        digitalWrite(pin, LOW);
        if (i < count - 1) delay(offMs);
    }
}

void blinkBothBlocking(int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(FAST_LED_PIN, HIGH);
        digitalWrite(TURN_LED_PIN, HIGH);
        delay(onMs);
        digitalWrite(FAST_LED_PIN, LOW);
        digitalWrite(TURN_LED_PIN, LOW);
        if (i < count - 1) delay(offMs);
    }
}

void surpriseMsg(const String &m) { sendMsg(m); }

/* MESSAGE */
void sendMsg(const String &m) {
    Serial.println(m);
    if (bluetoothEnabled) SerialBT.println(m);
}

/* BT CONTROL */
bool startBluetoothNow() {
    if (bluetoothEnabled) return true;
    bool ok = SerialBT.begin("ESP32_LF");
    if (ok) {
        bluetoothEnabled = true;
        Serial.println("BT ON");
        SerialBT.println("BT ON");
        blinkPinBlocking(FAST_LED_PIN, 2, 50, 40);
    } else {
        Serial.println("BT START FAILED");
    }
    return ok;
}

void stopBluetoothNow(const String &reason) {
    if (!bluetoothEnabled) {
        if (reason.length()) Serial.println(reason);
        return;
    }
    if (reason.length()) {
        Serial.println(reason);
        SerialBT.println(reason);
        delay(40);
    }
    SerialBT.end();
    bluetoothEnabled = false;
}

void handleBtRestoreButton() {
    bool pressed = (digitalRead(BT_RESTORE_PIN) == LOW);
    if (!pressed) {
        btButtonPressStart = 0;
        btButtonLatched = false;
        return;
    }
    if (btButtonLatched) return;
    if (btButtonPressStart == 0) {
        btButtonPressStart = millis();
        return;
    }
    if (millis() - btButtonPressStart >= BT_BUTTON_HOLD_MS) {
        btButtonLatched = true;
        btButtonPressStart = 0;
        if (!bluetoothEnabled) {
            if (robotState == RUNNING) {
                robotState = STOPPED;
                raceMode = false;
                stopMotors();
            }
            bootBtDefault = true;
            saveParams(false);
            if (startBluetoothNow()) {
                surpriseMsg("BT RESTORED BY BUTTON // back online");
            }
        } else {
            sendMsg("BT ALREADY ON");
        }
    }
}

/* PREFERENCES SAVE / LOAD */
void saveIntArray(const char *prefix, int *arr) {
    char key[10];
    for (int i = 0; i < SENSOR_COUNT; i++) {
        snprintf(key, sizeof(key), "%s%d", prefix, i);
        prefs.putInt(key, arr[i]);
    }
}

void loadIntArray(const char *prefix, int *arr) {
    char key[10];
    for (int i = 0; i < SENSOR_COUNT; i++) {
        snprintf(key, sizeof(key), "%s%d", prefix, i);
        arr[i] = prefs.getInt(key, arr[i]);
    }
}

void saveParams(bool announce) {
    prefs.begin("lfbot", false);
    prefs.putFloat("skp", skp);
    prefs.putFloat("ski", ski);
    prefs.putFloat("skd", skd);
    prefs.putFloat("ckp", ckp);
    prefs.putFloat("cki", cki);
    prefs.putFloat("ckd", ckd);
    prefs.putInt("mth", manualThreshold);
    prefs.putFloat("st", st);
    prefs.putFloat("ct", ct);
    prefs.putFloat("dt", dt);
    prefs.putInt("fd", fastDelay);
    prefs.putInt("ms", maxSpeed);
    prefs.putInt("mn", minSpeed);
    prefs.putFloat("sf", speedFactor);
    prefs.putInt("tp", turnPenalty);
    prefs.putInt("rs", recoverySpeed);
    prefs.putInt("thsrc", (int)thresholdMode);
    prefs.putInt("hascal", hasCalibration ? 1 : 0);
    prefs.putInt("bootbt", bootBtDefault ? 1 : 0);
    saveIntArray("mnv", calMin);
    saveIntArray("mxv", calMax);
    saveIntArray("ath", autoThresholds);
    prefs.end();
    if (announce) sendMsg("PARAMETERS SAVED");
}

void loadParams() {
    prefs.begin("lfbot", true);
    skp = prefs.getFloat("skp", skp);
    ski = prefs.getFloat("ski", ski);
    skd = prefs.getFloat("skd", skd);
    ckp = prefs.getFloat("ckp", ckp);
    cki = prefs.getFloat("cki", cki);
    ckd = prefs.getFloat("ckd", ckd);
    manualThreshold = prefs.getInt("mth", manualThreshold);
    st = prefs.getFloat("st", st);
    ct = prefs.getFloat("ct", ct);
    dt = prefs.getFloat("dt", dt);
    fastDelay = prefs.getInt("fd", fastDelay);
    maxSpeed = prefs.getInt("ms", maxSpeed);
    minSpeed = prefs.getInt("mn", minSpeed);
    speedFactor = prefs.getFloat("sf", speedFactor);
    turnPenalty = prefs.getInt("tp", turnPenalty);
    recoverySpeed = prefs.getInt("rs", recoverySpeed);
    thresholdMode = (ThresholdMode)prefs.getInt("thsrc", (int)thresholdMode);
    hasCalibration = prefs.getInt("hascal", hasCalibration ? 1 : 0);
    bootBtDefault = prefs.getInt("bootbt", bootBtDefault ? 1 : 0);
    loadIntArray("mnv", calMin);
    loadIntArray("mxv", calMax);
    loadIntArray("ath", autoThresholds);
    prefs.end();
    if (!hasCalibration && thresholdMode == THRESHOLD_AUTO) {
        thresholdMode = THRESHOLD_MANUAL;
    }
    sendMsg("PARAMETERS LOADED");
}

/* STATUS */
void showAutoThresholdLine() {
    String s = "AutoThresholds=";
    for (int i = 0; i < SENSOR_COUNT; i++) {
        s += String(autoThresholds[i]);
        if (i < SENSOR_COUNT - 1) s += ",";
    }
    sendMsg(s);
}

void showCalibration() {
    sendMsg("CALIBRATION DATA");
    sendMsg("HasCalibration=" + String(hasCalibration));
    for (int i = 0; i < SENSOR_COUNT; i++) {
        int span = calMax[i] - calMin[i];
        sendMsg("S" + String(i) + ": min=" + String(calMin[i]) + " max=" + String(calMax[i]) + " auto=" + String(autoThresholds[i]) + " span=" + String(span));
    }
}

void showStatus() {
    sendMsg("STATUS");
    sendMsg("RobotState=" + String(robotState == STOPPED ? "STOPPED" : (robotState == RUNNING ? "RUNNING" : "CALIBRATING")));
    sendMsg("DriveMode=" + driveModeText());
    sendMsg("BluetoothLive=" + onOff(bluetoothEnabled));
    sendMsg("BootBTDefault=" + onOff(bootBtDefault));
    sendMsg("RaceMode=" + onOff(raceMode));
    sendMsg("Scan=" + onOff(scanEnabled));
    sendMsg("ThresholdSource=" + thresholdModeText());
    sendMsg("ManualThreshold=" + String(manualThreshold));
    sendMsg("HasCalibration=" + onOff(hasCalibration));
    showAutoThresholdLine();
    sendMsg("st=" + String(st, 3));
    sendMsg("ct=" + String(ct, 3));
    sendMsg("dt=" + String(dt, 3));
    sendMsg("fastDelay=" + String(fastDelay));
    sendMsg("maxSpeed=" + String(maxSpeed));
    sendMsg("minSpeed=" + String(minSpeed));
    sendMsg("speedFactor=" + String(speedFactor, 2));
    sendMsg("turnPenalty=" + String(turnPenalty));
    sendMsg("recoverySpeed=" + String(recoverySpeed));
    sendMsg("FAST PID: " + String(skp, 3) + "," + String(ski, 3) + "," + String(skd, 3));
    sendMsg("TURN PID: " + String(ckp, 3) + "," + String(cki, 3) + "," + String(ckd, 3));
}

void showHelp() {
    sendMsg("--- COMMAND LIST ---");
    sendMsg("g/go -> run");
    sendMsg("r/race -> BT OFF + run");
    sendMsg("s -> stop");
    sendMsg("dance -> victory dance");
    sendMsg("scan -> toggle live scan");
    sendMsg("scan on -> scan ON");
    sendMsg("scan off -> scan OFF");
    sendMsg("stat -> show status");
    sendMsg("calshow -> show detailed auto calibration");
    sendMsg("help -> show help");
    sendMsg("save -> save parameters");
    sendMsg("load -> load parameters");
    sendMsg("reboot -> restart ESP32");
    sendMsg("bt on -> BT ON now + save boot BT ON");
    sendMsg("bt off -> BT OFF now + save boot BT OFF");
    sendMsg("cal -> auto-calibrate 6000 ms (BT OFF)");
    sendMsg("cal#### -> custom ms (example cal8000)");
    sendMsg("clearcal -> clear saved calibration");
    sendMsg("useauto -> use saved auto thresholds");
    sendMsg("useman -> use manual threshold");
    sendMsg("---SPEED---");
    sendMsg("ms### -> max speed");
    sendMsg("mn### -> min speed");
    sendMsg("sf#.# -> speed factor");
    sendMsg("tp### -> turn penalty");
    sendMsg("rs### -> recovery speed");
    sendMsg("--- THRESHOLD / MODE ----");
    sendMsg("th### -> manual threshold");
    sendMsg("st#.# -> FAST entry error threshold");
    sendMsg("ct#.# -> TURN force error threshold");
    sendMsg("dt#.# -> TURN force dError threshold");
    sendMsg("fd### -> FAST entry delay ms");
    sendMsg("- PID ----");
    sendMsg("skp ski skd -> FAST PID");
    sendMsg("ckp cki ckd -> TURN PID");
}

/* PID */
void resetPID() {
    sumError = 0.0;
    lastError = 0.0;
    lastDerivative = 0.0;
}

float computePID(float e, float kp, float ki, float kd) {
    sumError += e;
    sumError = constrain(sumError, -100.0f, 100.0f);
    float d = e - lastError;
    lastError = e;
    return (kp * e) + (ki * sumError) + (kd * d);
}

/* MOTORS */
void stopMotors() {
    ledcWrite(PWMA, 0);
    ledcWrite(PWMB, 0);
}

void setMotor(int L, int R) {
    L = constrain(L, -255, 255);
    R = constrain(R, -255, 255);
    if (L >= 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    } else {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        L = -L;
    }
    if (R >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        R = -R;
    }
    ledcWrite(PWMA, L);
    ledcWrite(PWMB, R);
}

/* VICTORY DANCE */
void victoryDance() {
    robotState = STOPPED;
    raceMode = false;
    stopMotors();
    surpriseMsg("VICTORY DANCE // line conquered");
    for (int i = 0; i < 3; i++) {
        digitalWrite(FAST_LED_PIN, HIGH);
        digitalWrite(TURN_LED_PIN, LOW);
        setMotor(150, -150);
        delay(120);
        digitalWrite(FAST_LED_PIN, LOW);
        digitalWrite(TURN_LED_PIN, HIGH);
        setMotor(-150, 150);
        delay(120);
    }
    setMotor(180, 180);
    digitalWrite(FAST_LED_PIN, HIGH);
    digitalWrite(TURN_LED_PIN, HIGH);
    delay(180);
    stopMotors();
    digitalWrite(FAST_LED_PIN, LOW);
    digitalWrite(TURN_LED_PIN, LOW);
    blinkBothBlocking(2, 80, 60);
    surpriseMsg("DANCE COMPLETE");
}

/* LEDs */
void updateModeLeds() {
    if (robotState == CALIBRATING) {
        bool phase = ((millis() / 120) % 2) == 0;
        digitalWrite(FAST_LED_PIN, phase);
        digitalWrite(TURN_LED_PIN, !phase);
        return;
    }
    if (robotState != RUNNING) {
        digitalWrite(FAST_LED_PIN, LOW);
        digitalWrite(TURN_LED_PIN, LOW);
        return;
    }
    digitalWrite(FAST_LED_PIN, driveMode == FAST_MODE ? HIGH : LOW);
    digitalWrite(TURN_LED_PIN, driveMode == TURN_MODE ? HIGH : LOW);
}

/* SENSOR READ */
void captureFrame() {
    frame.weighted = 0;
    frame.total = 0;
    frame.error = 0.0;
    frame.lineDetected = false;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        int raw = analogRead(qtrPins[i]);
        frame.raw[i] = raw;
        int processed = 0;
        uint8_t b = 0;
        if (thresholdMode == THRESHOLD_AUTO && hasCalibration && calMax[i] > autoThresholds[i]) {
            if (raw >= autoThresholds[i]) {
                processed = mapClampInt(raw, autoThresholds[i], calMax[i], 0, 1000);
                b = 1;
            } else {
                processed = 0;
                b = 0;
            }
        } else {
            processed = raw - manualThreshold;
            if (processed < 0) processed = 0;
            b = (raw >= manualThreshold) ? 1 : 0;
        }
        frame.processed[i] = processed;
        frame.bin[i] = b;
        frame.weighted += (long)processed * weights[i];
        frame.total += processed;
    }
    if (frame.total > 0) {
        frame.lineDetected = true;
        frame.error = (float)frame.weighted / (float)frame.total;
    }
}

/* SCAN OUTPUT */
void emitScan() {
    if (!scanEnabled) return;
    if (millis() - scanTimer < 120) return;
    scanTimer = millis();
    String rawStr = "RAW ";
    String actStr = "ACT ";
    String binStr = "BIN ";
    for (int i = 0; i < SENSOR_COUNT; i++) {
        rawStr += String(frame.raw[i]) + " ";
        actStr += String(frame.processed[i]) + " ";
        binStr += String(frame.bin[i]) + " ";
    }
    sendMsg(rawStr);
    sendMsg(actStr);
    sendMsg(binStr);
    String info = "INFO LINE=" + String(frame.lineDetected ? 1 : 0);
    info += " ERR=" + String(frame.error, 3);
    info += " dE=" + String(lastDerivative, 3);
    info += " TOT=" + String(frame.total);
    if (robotState == RUNNING) {
        info += " BASE=" + String(lastBase);
        info += " COR=" + String(lastCorrection, 2);
        info += " MODE=" + driveModeText();
    }
    info += " THR=" + thresholdModeText();
    sendMsg(info);
}

/* DRIVE MODE UPDATE */
void setDriveMode(DriveMode m) {
    if (driveMode != m) {
        driveMode = m;
        resetPID();
    }
}

void updateDriveMode(float absE, float absD) {
    if (absE > ct || absD > dt) {
        setDriveMode(TURN_MODE);
        fastCandidateTimer = millis();
        return;
    }
    if (absE < st && absD < (dt * 0.5f)) {
        if (driveMode == TURN_MODE) {
            if (millis() - fastCandidateTimer >= (unsigned long)fastDelay) {
                setDriveMode(FAST_MODE);
            }
        }
    } else {
        setDriveMode(TURN_MODE);
        fastCandidateTimer = millis();
    }
}

/* LINE FOLLOWER */
void lineFollower() {
    captureFrame();
    if (!frame.lineDetected) {
        setDriveMode(TURN_MODE);
        int dir = (lastSeenError >= 0.0f) ? 1 : -1;
        setMotor(dir * recoverySpeed, dir * recoverySpeed);
        lastBase = 0;
        lastCorrection = 0.0;
        lastDerivative = 0.0;
        updateModeLeds();
        emitScan();
        return;
    }
    float error = frame.error;
    float dError = error - lastError;
    lastDerivative = dError;
    lastSeenError = error;
    updateDriveMode(fabsf(error), fabsf(dError));
    float correction;
    if (driveMode == FAST_MODE) {
        correction = computePID(error, skp, ski, skd);
    } else {
        correction = computePID(error, ckp, cki, ckd);
    }
    int base = maxSpeed - (int)(fabsf(error) * speedFactor);
    if (driveMode == TURN_MODE) base -= turnPenalty;
    base = constrain(base, minSpeed, maxSpeed);
    correction = constrain(correction, (float)-base, (float)base);
    int L = base + (int)correction;
    int R = base - (int)correction;
    setMotor(L, R);
    lastBase = base;
    lastCorrection = correction;
    updateModeLeds();
    emitScan();
}

/* CALIBRATION */
void clearCalibration() {
    hasCalibration = false;
    thresholdMode = THRESHOLD_MANUAL;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        calMin[i] = 0;
        calMax[i] = 0;
        autoThresholds[i] = 0;
    }
    saveParams(false);
    sendMsg("CALIBRATION CLEARED");
}

void runCalibration(unsigned long durationMs) {
    surpriseMsg("CALIBRATION START // teach me the track");
    sendMsg("Move robot by hand over black and white.");
    sendMsg("BT will turn OFF during calibration.");
    delay(200);
    bootBtDefault = false;
    saveParams(false);
    stopBluetoothNow("BT OFF FOR CALIBRATION");
    robotState = CALIBRATING;
    raceMode = false;
    stopMotors();
    resetPID();
    for (int i = 0; i < SENSOR_COUNT; i++) {
        calMin[i] = 4095;
        calMax[i] = 0;
        autoThresholds[i] = 0;
    }
    unsigned long startMs = millis();
    unsigned long lastPrint = 0;
    while (millis() - startMs < durationMs) {
        updateModeLeds();
        for (int i = 0; i < SENSOR_COUNT; i++) {
            int v = analogRead(qtrPins[i]);
            if (v < calMin[i]) calMin[i] = v;
            if (v > calMax[i]) calMax[i] = v;
        }
        if (millis() - lastPrint > 500) {
            lastPrint = millis();
            Serial.println("CAL " + String((durationMs - (millis() - startMs)) / 1000.0f, 1) + "s left");
        }
        delay(2);
    }
    hasCalibration = true;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (calMax[i] < calMin[i]) {
            calMin[i] = 0;
            calMax[i] = 0;
        }
        autoThresholds[i] = calMin[i] + (calMax[i] - calMin[i]) / 2;
    }
    saveParams(false);
    robotState = STOPPED;
    stopMotors();
    digitalWrite(FAST_LED_PIN, LOW);
    digitalWrite(TURN_LED_PIN, LOW);
    blinkBothBlocking(3, 80, 70);
    Serial.println("CALIBRATION DONE");
    Serial.println("BT remains OFF. Press button on GPIO4 to restore BT.");
    showCalibration();
}

/* PARAM SET HELPERS */
void setAndSaveInt(const char *name, int &var, int value) {
    var = value;
    saveParams(false);
    sendMsg(String(name) + "=" + String(var));
}

void setAndSaveFloat(const char *name, float &var, float value, int digits) {
    var = value;
    saveParams(false);
    sendMsg(String(name) + "=" + String(var, digits));
}

/* COMMAND HANDLER */
void handleCmd(String c) {
    c.trim();
    c.toLowerCase();
    if (c.length() == 0) return;
    if (c == "go") c = "g";
    if (c == "victory") c = "dance";
    if (c == "help" || c == "h") {
        showHelp();
        return;
    }
    if (c == "g") {
        raceMode = false;
        robotState = RUNNING;
        fastCandidateTimer = millis();
        resetPID();
        blinkPinBlocking(FAST_LED_PIN, 1, 70, 40);
        surpriseMsg("RUN // line hunt begins");
        return;
    }
    if (c == "r" || c == "race") {
        raceMode = true;
        bootBtDefault = false;
        saveParams(false);
        blinkPinBlocking(TURN_LED_PIN, 2, 70, 50);
        stopBluetoothNow("BT OFF FOR RACE");
        robotState = RUNNING;
        fastCandidateTimer = millis();
        resetPID();
        Serial.println("RACE RUN // radio silent");
        return;
    }
    if (c == "s") {
        robotState = STOPPED;
        raceMode = false;
        stopMotors();
        blinkBothBlocking(1, 60, 40);
        sendMsg("STOPPED");
        return;
    }
    if (c == "dance") {
        victoryDance();
        return;
    }
    if (c == "scan") {
        scanEnabled = !scanEnabled;
        sendMsg("SCAN=" + onOff(scanEnabled));
        return;
    }
    if (c == "scan on") {
        scanEnabled = true;
        sendMsg("SCAN=ON");
        return;
    }
    if (c == "scan off") {
        scanEnabled = false;
        sendMsg("SCAN=OFF");
        return;
    }
    if (c == "stat") {
        showStatus();
        return;
    }
    if (c == "save") {
        saveParams(true);
        return;
    }
    if (c == "load") {
        loadParams();
        return;
    }
    if (c == "reboot") {
        sendMsg("REBOOTING");
        delay(100);
        ESP.restart();
        return;
    }
    if (c == "calshow") {
        showCalibration();
        return;
    }
    if (c == "clearcal") {
        clearCalibration();
        return;
    }
    if (c == "useauto") {
        if (!hasCalibration) {
            sendMsg("NO SAVED CALIBRATION");
            return;
        }
        thresholdMode = THRESHOLD_AUTO;
        saveParams(false);
        surpriseMsg("AUTO THRESHOLDS ONLINE");
        return;
    }
    if (c == "useman") {
        thresholdMode = THRESHOLD_MANUAL;
        saveParams(false);
        surpriseMsg("MANUAL THRESHOLD ONLINE");
        return;
    }
    if (c == "bt on" || c == "bton") {
        bootBtDefault = true;
        saveParams(false);
        startBluetoothNow();
        sendMsg("BOOT BT=ON");
        return;
    }
    if (c == "bt off" || c == "btoff") {
        bootBtDefault = false;
        saveParams(false);
        stopBluetoothNow("BT OFF");
        Serial.println("BOOT BT OFF");
        return;
    }
    if (c.startsWith("cal")) {
        unsigned long durationMs = 6000;
        String tail = c.substring(3);
        tail.trim();
        if (tail.length() > 0) {
            long temp = tail.toInt();
            if (temp >= 2000) durationMs = (unsigned long)temp;
        }
        runCalibration(durationMs);
        return;
    }
    if (c.startsWith("th")) {
        setAndSaveInt("manualThreshold", manualThreshold, c.substring(2).toInt());
        return;
    }
    if (c.startsWith("st")) {
        setAndSaveFloat("st", st, c.substring(2).toFloat(), 3);
        return;
    }
    if (c.startsWith("ct")) {
        setAndSaveFloat("ct", ct, c.substring(2).toFloat(), 3);
        return;
    }
    if (c.startsWith("dt")) {
        setAndSaveFloat("dt", dt, c.substring(2).toFloat(), 3);
        return;
    }
    if (c.startsWith("fd")) {
        setAndSaveInt("fastDelay", fastDelay, c.substring(2).toInt());
        return;
    }
    if (c.startsWith("ms")) {
        setAndSaveInt("maxSpeed", maxSpeed, c.substring(2).toInt());
        return;
    }
    if (c.startsWith("mn")) {
        setAndSaveInt("minSpeed", minSpeed, c.substring(2).toInt());
        return;
    }
    if (c.startsWith("sf")) {
        setAndSaveFloat("speedFactor", speedFactor, c.substring(2).toFloat(), 3);
        return;
    }
    if (c.startsWith("tp")) {
        setAndSaveInt("turnPenalty", turnPenalty, c.substring(2).toInt());
        return;
    }
    if (c.startsWith("rs")) {
        setAndSaveInt("recoverySpeed", recoverySpeed, c.substring(2).toInt());
        return;
    }
    if (c.startsWith("skp")) {
        setAndSaveFloat("skp", skp, c.substring(3).toFloat(), 4);
        return;
    }
    if (c.startsWith("ski")) {
        setAndSaveFloat("ski", ski, c.substring(3).toFloat(), 4);
        return;
    }
    if (c.startsWith("skd")) {
        setAndSaveFloat("skd", skd, c.substring(3).toFloat(), 4);
        return;
    }
    if (c.startsWith("ckp")) {
        setAndSaveFloat("ckp", ckp, c.substring(3).toFloat(), 4);
        return;
    }
    if (c.startsWith("cki")) {
        setAndSaveFloat("cki", cki, c.substring(3).toFloat(), 4);
        return;
    }
    if (c.startsWith("ckd")) {
        setAndSaveFloat("ckd", ckd, c.substring(3).toFloat(), 4);
        return;
    }
    if (c.startsWith("nkp") || c.startsWith("nki") || c.startsWith("nkd")) {
        sendMsg("NORMAL MODE REMOVED. Use sk for FAST and ck for TURN.");
        return;
    }
    sendMsg("UNKNOWN COMMAND: " + c);
}

/* STREAM READ */
void readStream(Stream &p, String &buffer) {
    while (p.available()) {
        char ch = p.read();
        if (ch == '\n' || ch == '\r') {
            if (buffer.length() > 0) {
                handleCmd(buffer);
                buffer = "";
            }
        } else {
            buffer += ch;
        }
    }
}

/* SETUP */
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    pinMode(FAST_LED_PIN, OUTPUT);
    pinMode(TURN_LED_PIN, OUTPUT);
    pinMode(BT_RESTORE_PIN, INPUT_PULLUP);
    for (int i = 0; i < SENSOR_COUNT; i++) {
        pinMode(qtrPins[i], INPUT);
    }
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    ledcAttach(PWMA, PWM_FREQ, PM_RES);
    ledcAttach(PWMB, PWM_FREQ, PM_RES);
    stopMotors();
    digitalWrite(FAST_LED_PIN, LOW);
    digitalWrite(TURN_LED_PIN, LOW);
    loadParams();
    bool forceBtByButtonAtBoot = (digitalRead(BT_RESTORE_PIN) == LOW);
    if (forceBtByButtonAtBoot) {
        bootBtDefault = true;
        saveParams(false);
    }
    if (bootBtDefault || forceBtByButtonAtBoot) {
        startBluetoothNow();
    } else {
        Serial.println("BT stays OFF at boot.");
        Serial.println("Press button on GPIO4 to restore BT, or use USB serial 'bt on'.");
    }
    fastCandidateTimer = millis();
    sendMsg("READY");
}

/* LOOP */
void loop() {
    handleBtRestoreButton();
    readStream(Serial, usbBuffer);
    if (bluetoothEnabled) {
        readStream(SerialBT, btBuffer);
    }
    switch (robotState) {
        case RUNNING:
            lineFollower();
            break;
        case STOPPED:
            stopMotors();
            updateModeLeds();
            if (scanEnabled) {
                captureFrame();
                lastDerivative = 0.0;
                emitScan();
            }
            break;
        case CALIBRATING:
            updateModeLeds();
            break;
    }
}