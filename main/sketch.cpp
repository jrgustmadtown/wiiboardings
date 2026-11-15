// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <esp_console.h>
#include <math.h>

constexpr uint8_t kStatusLedPin = 48;  // Change to your LED GPIO (e.g., 48 or 38 on ESP32-S3, or any free GPIO like 13)

void updateStatusLed() {
    static uint32_t lastLedUpdate = 0;
    if (millis() - lastLedUpdate < 50) {
        return;
    }
    lastLedUpdate = millis();

    if (!systemEnabled) {
        neopixelWrite(kStatusLedPin, 40, 0, 0);      // Dim Red = Disarmed / Off
    } else if (leanModeEnabled) {
        neopixelWrite(kStatusLedPin, 0, 60, 150);    // Blue = Armed (Lean Mode)
    } else {
        neopixelWrite(kStatusLedPin, 0, 150, 20);    // Green = Armed (Step Mode)
    }
}

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

enum class DominantSide : uint8_t { None, Left, Right };

struct MotorPulses {
    uint32_t leftUs;
    uint32_t rightUs;
};

// Pin 0 = Left Motor (GPIO 18), Pin 1 = Right Motor (GPIO 19)
constexpr uint8_t kLeftMotorPin = 26;
constexpr uint8_t kRightMotorPin = 27;
constexpr uint8_t kMotorPwmPins[] = {kLeftMotorPin, kRightMotorPin};

constexpr uint32_t kMotorPwmFrequencyHz = 50;
constexpr uint8_t kMotorPwmResolutionBits = 16;
constexpr uint32_t kMotorPwmPeriodUs = 1000000UL / kMotorPwmFrequencyHz;
constexpr uint32_t kMinimumReversePulseUs = 1000;
constexpr uint32_t kNeutralPulseUs = 1500;
constexpr uint32_t kMaximumForwardPulseUs = 2000;

// Differential steering tuning
constexpr float kMaxTiltDegrees = 60.0f;           // Tilt angle for full steering authority
constexpr float kSteeringDeadzoneDegrees = 5.0f;   // Ignore small hand jitters
constexpr int32_t kMaxSteeringOffsetUs = 250;      // Max differential offset added/subtracted per wheel
constexpr bool kInvertSteering = false;            // Flip to true if left/right turns are reversed

constexpr uint32_t kMinimumStepLoadGrams = 10000;
constexpr uint32_t kStepSidePercent = 75;
constexpr uint32_t kStepWindowMs = 1000;
constexpr uint32_t kLevelUpIntervalMs = 500;
constexpr uint32_t kLevelDownIntervalMs = 250;
constexpr uint32_t kBalanceBoardTimeoutMs = 1000;
constexpr uint32_t kMonitorOutputIntervalMs = 800;
constexpr uint8_t kMaximumSpeedLevel = 10;
constexpr uint32_t kPwmBenchTestPulseUs = 1600;
constexpr uint32_t kPwmBenchTestPulseMs = 300;
constexpr int32_t kMaxLeanThrottleOffsetUs = 500;  
constexpr int32_t kMaxLeanSteeringOffsetUs = 250;

static bool motorPwmReady = false;
static bool pwmBenchTestActive = false;
static bool balanceBoardOccupied = false;
static bool systemEnabled = false;
static bool leanModeEnabled = false;
static int32_t leanFrontBackOffsetUs = 0;          
static int32_t leanLeftRightOffsetUs = 0;
static uint8_t currentSpeedLevel = 0;
static uint32_t lastSpeedLevelChangeAt = 0;
static uint32_t lastBalanceBoardDataAt = 0;
static DominantSide lastCountedSide = DominantSide::None;
static uint32_t totalStepCount = 0;
static uint32_t recentStepTimes[kMaximumSpeedLevel] = {};
static uint8_t recentStepCount = 0;
static uint8_t nextStepSlot = 0;
static uint32_t lastMonitorPrintAt[BP32_MAX_GAMEPADS] = {};
static bool hasMonitorPrinted[BP32_MAX_GAMEPADS] = {};
static float currentTiltDegrees = 0.0f;

bool shouldPrintControllerMonitor(ControllerPtr ctl, bool force = false) {
    const uint8_t index = ctl->index();
    if (index >= BP32_MAX_GAMEPADS) {
        return true;
    }

    const uint32_t now = millis();
    if (!force && hasMonitorPrinted[index] && now - lastMonitorPrintAt[index] < kMonitorOutputIntervalMs) {
        return false;
    }

    hasMonitorPrinted[index] = true;
    lastMonitorPrintAt[index] = now;
    return true;
}

MotorPulses computeLeanPulses(bool isOccupied) {
    if (!systemEnabled || !isOccupied) {
        return {kNeutralPulseUs, kNeutralPulseUs};
    }

    const int32_t rawLeftUs =
        static_cast<int32_t>(kNeutralPulseUs) + leanFrontBackOffsetUs + leanLeftRightOffsetUs;
    const int32_t rawRightUs =
        static_cast<int32_t>(kNeutralPulseUs) + leanFrontBackOffsetUs - leanLeftRightOffsetUs;

    MotorPulses pulses;
    pulses.leftUs = constrain(rawLeftUs, kMinimumReversePulseUs, kMaximumForwardPulseUs);
    pulses.rightUs = constrain(rawRightUs, kMinimumReversePulseUs, kMaximumForwardPulseUs);
    return pulses;
}

MotorPulses computeDifferentialPulses(uint8_t speedLevel, float tiltDegrees, bool isOccupied) {
    
    if (!systemEnabled || !isOccupied) {
        return {kNeutralPulseUs, kNeutralPulseUs};
    }

    if (speedLevel > kMaximumSpeedLevel) {
        speedLevel = kMaximumSpeedLevel;
    }

    // 1. Base forward throttle from Balance Board cadence
    const int32_t baseThrottleUs =
        kNeutralPulseUs +
        static_cast<int32_t>((kMaximumForwardPulseUs - kNeutralPulseUs) * speedLevel / kMaximumSpeedLevel);

    // 2. Apply deadzone and clamp tilt angle
    if (fabsf(tiltDegrees) < kSteeringDeadzoneDegrees) {
        tiltDegrees = 0.0f;
    } else if (tiltDegrees > 0.0f) {
        tiltDegrees -= kSteeringDeadzoneDegrees;
    } else {
        tiltDegrees += kSteeringDeadzoneDegrees;
    }

    const float effectiveMaxTilt = kMaxTiltDegrees - kSteeringDeadzoneDegrees;
    tiltDegrees = constrain(tiltDegrees, -effectiveMaxTilt, effectiveMaxTilt);

    if (kInvertSteering) {
        tiltDegrees = -tiltDegrees;
    }

    // 3. Calculate differential offset (-kMaxSteeringOffsetUs to +kMaxSteeringOffsetUs)
    const int32_t steeringOffsetUs =
        static_cast<int32_t>((tiltDegrees / effectiveMaxTilt) * kMaxSteeringOffsetUs);

    // 4. Mix throttle and steering:
    // Turning Right (positive offset) -> Left wheel speeds up, Right wheel slows down
    // Turning Left (negative offset)  -> Left wheel slows down, Right wheel speeds up
    int32_t rawLeftUs = baseThrottleUs + steeringOffsetUs;
    int32_t rawRightUs = baseThrottleUs - steeringOffsetUs;
    

    MotorPulses pulses;
    pulses.leftUs = constrain(rawLeftUs, kMinimumReversePulseUs, kMaximumForwardPulseUs);
    pulses.rightUs = constrain(rawRightUs, kMinimumReversePulseUs, kMaximumForwardPulseUs);
    return pulses;
}

MotorPulses computeActivePulses() {
    if (leanModeEnabled) {
        return computeLeanPulses(balanceBoardOccupied);
    }
    return computeDifferentialPulses(currentSpeedLevel, currentTiltDegrees, balanceBoardOccupied);
}

void writeMotorPinPulseUs(uint8_t pin, uint32_t pulseUs) {
    if (!motorPwmReady) {
        return;
    }
    pulseUs = constrain(pulseUs, kMinimumReversePulseUs, kMaximumForwardPulseUs);

    const uint32_t maximumDuty = (1UL << kMotorPwmResolutionBits) - 1;
    const uint32_t duty = static_cast<uint32_t>(
        static_cast<uint64_t>(pulseUs) * maximumDuty / kMotorPwmPeriodUs);
    ledcWrite(pin, duty);
}

void writeMotorPulses(MotorPulses pulses) {
    if (pwmBenchTestActive) {
        return;
    }
    writeMotorPinPulseUs(kLeftMotorPin, pulses.leftUs);
    writeMotorPinPulseUs(kRightMotorPin, pulses.rightUs);
}

void writeNeutralPulses() {
    writeMotorPinPulseUs(kLeftMotorPin, kNeutralPulseUs);
    writeMotorPinPulseUs(kRightMotorPin, kNeutralPulseUs);
}

void setupMotorPwm() {
    const bool firstPinReady = ledcAttach(kLeftMotorPin, kMotorPwmFrequencyHz, kMotorPwmResolutionBits);
    const bool secondPinReady = ledcAttach(kRightMotorPin, kMotorPwmFrequencyHz, kMotorPwmResolutionBits);
    motorPwmReady = firstPinReady && secondPinReady;
    lastSpeedLevelChangeAt = millis();
    if (motorPwmReady) {
        writeNeutralPulses();
    } else {
        Console.println("Motor PWM initialization failed; outputs remain disabled");
    }
}

void runPwmBenchTest() {
    if (!motorPwmReady) {
        return;
    }
    if (pwmBenchTestActive) {
        Console.println("PWM bench test is already running");
        return;
    }

    pwmBenchTestActive = true;
    writeNeutralPulses();
    for (uint8_t pin : kMotorPwmPins) {
        Console.printf("PWM bench test: GPIO%u (%s), pulse=%lu us\n",
                       static_cast<unsigned>(pin),
                       pin == kLeftMotorPin ? "LEFT" : "RIGHT",
                       static_cast<unsigned long>(kPwmBenchTestPulseUs));
        writeMotorPinPulseUs(pin, kPwmBenchTestPulseUs);
        delay(kPwmBenchTestPulseMs);
        writeMotorPinPulseUs(pin, kNeutralPulseUs);
        delay(500);
    }
    writeNeutralPulses();
    pwmBenchTestActive = false;
    Console.println("PWM bench test complete; outputs neutral");
}

static int pwmTestConsoleCommand(int argc, char** argv) {
    (void)argc;
    (void)argv;
    runPwmBenchTest();
    return 0;
}

static void registerPwmTestConsoleCommand() {
    const esp_console_cmd_t command = {
        .command = "pwm_test",
        .help = "Pulse each motor output once at a low forward value",
        .hint = nullptr,
        .func = &pwmTestConsoleCommand,
        .argtable = nullptr,
    };
    const esp_err_t result = esp_console_cmd_register(&command);
    if (result != ESP_OK) {
        Console.printf("Failed to register pwm_test command: %d\n", static_cast<int>(result));
    }
}

void resetMotorOutput() {
    balanceBoardOccupied = false;
    leanFrontBackOffsetUs = 0; 
    leanLeftRightOffsetUs = 0;
    currentSpeedLevel = 0;
    lastSpeedLevelChangeAt = millis();
    lastBalanceBoardDataAt = 0;
    lastCountedSide = DominantSide::None;
    totalStepCount = 0;
    recentStepCount = 0;
    nextStepSlot = 0;
    if (!pwmBenchTestActive) {
        writeNeutralPulses();
    }
}

void recordStep(uint32_t now) {
    recentStepTimes[nextStepSlot] = now;
    nextStepSlot = (nextStepSlot + 1) % kMaximumSpeedLevel;
    if (recentStepCount < kMaximumSpeedLevel) {
        recentStepCount++;
    }
    totalStepCount++;
}

uint8_t countRecentSteps(uint32_t now) {
    uint8_t stepsInWindow = 0;
    for (uint8_t i = 0; i < recentStepCount; i++) {
        if (now - recentStepTimes[i] <= kStepWindowMs) {
            stepsInWindow++;
        }
    }
    return stepsInWindow;
}

void updateMotorSpeed(uint32_t now, uint8_t targetSpeedLevel) {
    if (pwmBenchTestActive) {
        return;
    }
    if (targetSpeedLevel > kMaximumSpeedLevel) {
        targetSpeedLevel = kMaximumSpeedLevel;
    }
    if (targetSpeedLevel == currentSpeedLevel) {
        return;
    }

    const uint32_t rampInterval = targetSpeedLevel > currentSpeedLevel ? kLevelUpIntervalMs
                                                                       : kLevelDownIntervalMs;
    if (now - lastSpeedLevelChangeAt < rampInterval) {
        return;
    }

    if (targetSpeedLevel > currentSpeedLevel) {
        currentSpeedLevel++;
    } else {
        currentSpeedLevel--;
    }
    lastSpeedLevelChangeAt = now;
    writeMotorPulses(computeActivePulses());
}

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
    if (ctl->getModelName() == "Wii") {
        currentTiltDegrees = 0.0f;
        systemEnabled = false;  
        leanModeEnabled = false;
    }
    resetMotorOutput();
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(), ctl->dpad(), ctl->buttons(), ctl->axisX(), ctl->axisY(), ctl->axisRX(), ctl->axisRY(),
        ctl->brake(), ctl->throttle(), ctl->miscButtons(), ctl->gyroX(), ctl->gyroY(), ctl->gyroZ(),
        ctl->accelX(), ctl->accelY(), ctl->accelZ());
}

float getWiiRemoteTilt(ControllerPtr ctl) {
    // Note: If holding the Wii Remote horizontally like a steering wheel (D-pad on left, 1/2 on right),
    // roll angle uses accelY and accelZ: atan2f(accelY, accelZ).
    // If pointing it forward like a TV remote, roll angle uses accelX and accelZ: atan2f(accelX, accelZ).
    const float accelZ = static_cast<float>(ctl->accelZ());
    const float accelX = static_cast<float>(ctl->accelX());

    return atan2f(accelZ, accelX) * 57.2957795f;
}

void dumpWiiRemoteTilt(ControllerPtr ctl) {
    const MotorPulses pulses = computeActivePulses();
    Console.printf("Wii remote idx=%d, tilt=%.1f deg, pwm_L=%lu us, pwm_R=%lu us, accel_x=%d, accel_y=%d, accel_z=%d\n",
                   ctl->index(), currentTiltDegrees,
                   static_cast<unsigned long>(pulses.leftUs),
                   static_cast<unsigned long>(pulses.rightUs),
                   ctl->accelX(), ctl->accelY(), ctl->accelZ());
}

void dumpMouse(ControllerPtr ctl) {
    Console.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(), ctl->buttons(), ctl->scrollWheel(), ctl->deltaX(), ctl->deltaY());
}

void dumpKeyboard(ControllerPtr ctl) {
    static const char* key_names[] = {
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
    };
    static const char* modifier_names[] = {
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
    };
    Console.printf("idx=%d, Pressed keys: ", ctl->index());
    for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            Console.printf("%s,", key_names[key - 4]);
        }
    }
    for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            Console.printf("%s,", modifier_names[key - 0xe0]);
        }
    }
    Console.printf("\n");
}

void dumpBalanceBoard(ControllerPtr ctl, uint32_t topLeft, uint32_t topRight, uint32_t bottomLeft,
                      uint32_t bottomRight, const char* lean, const char* lastStep, uint32_t steps,
                      uint8_t stepsInWindow, MotorPulses pulses, bool stepEvent, bool balanceBoardOccupied) {
    if (!shouldPrintControllerMonitor(ctl, stepEvent)) {
        return;
    }

    const uint32_t left = topLeft + bottomLeft;
    const uint32_t right = topRight + bottomRight;
    const uint32_t front = topLeft + topRight;
    const uint32_t back = bottomLeft + bottomRight;
    const uint32_t total = topLeft + topRight + bottomLeft + bottomRight;

    Console.printf(
        "idx=%d, total=%lu g, left=%lu g (%u%%), right=%lu g (%u%%), front=%lu g, back=%lu g, "
        "lean=%s, last_step=%s, steps=%lu, steps_2s=%u, speed_level=%u, tilt=%.1f deg, pwm_L=%lu us, pwm_R=%lu us, boardOccupied=%lu\n",
        ctl->index(), static_cast<unsigned long>(total), static_cast<unsigned long>(left),
        static_cast<unsigned>(total ? left * 100 / total : 0),
        static_cast<unsigned long>(right), static_cast<unsigned>(total ? right * 100 / total : 0),
        static_cast<unsigned long>(front), static_cast<unsigned long>(back), lean, lastStep,
        static_cast<unsigned long>(steps), static_cast<unsigned>(stepsInWindow),
        static_cast<unsigned>(currentSpeedLevel), currentTiltDegrees,
        static_cast<unsigned long>(pulses.leftUs), static_cast<unsigned long>(pulses.rightUs), balanceBoardOccupied);
}

void processGamepad(ControllerPtr ctl) {
    if (ctl->getModelName() == "Wii") {
        currentTiltDegrees = getWiiRemoteTilt(ctl);
        static bool prevButton2Pressed = false;
        const bool button2Pressed = ctl->b();  // Wii Remote '2' button maps to y()
        if (button2Pressed && !prevButton2Pressed) {
            systemEnabled = !systemEnabled;
            Console.printf("System %s\n", systemEnabled ? "ENABLED" : "DISABLED");
            if (!systemEnabled) {
                resetMotorOutput();
            }
        }
        prevButton2Pressed = button2Pressed;

        static bool prevButton1Pressed = false;
        const bool button1Pressed = ctl->a();  // Wii Remote '1' button
        if (button1Pressed && !prevButton1Pressed) {
            leanModeEnabled = !leanModeEnabled;
            resetMotorOutput();
            Console.printf("Mode switched to %s\n", leanModeEnabled ? "LEAN MODE" : "STEP MODE");
        }
        prevButton1Pressed = button1Pressed;
    }
    if (ctl->a()) {
        static int colorIdx = 0;
        switch (colorIdx % 3) {
            case 0: ctl->setColorLED(255, 0, 0); break;
            case 1: ctl->setColorLED(0, 255, 0); break;
            case 2: ctl->setColorLED(0, 0, 255); break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        static int led = 0;
        led++;
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        ctl->playDualRumble(0, 250, 0x80, 0x40);
    }

    if (shouldPrintControllerMonitor(ctl)) {
        if (ctl->getModelName() == "Wii") {
            dumpWiiRemoteTilt(ctl);
        } else {
            dumpGamepad(ctl);
        }
    }
}

void processMouse(ControllerPtr ctl) {
    dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {
    if (!ctl->isAnyKeyPressed())
        return;

    if (ctl->isKeyPressed(Keyboard_A)) {
        Console.println("Key 'A' pressed");
    }
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        Console.println("Key 'LEFT SHIFT' pressed");
    }
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        Console.println("Key 'Left Arrow' pressed");
    }

    dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    constexpr uint32_t minimumLoadGrams = 300;
    constexpr uint32_t minimumDeadzoneGrams = 200;
    const uint32_t now = millis();
    lastBalanceBoardDataAt = now;

    static bool initialized = false;
    static uint32_t topLeft = 0;
    static uint32_t topRight = 0;
    static uint32_t bottomLeft = 0;
    static uint32_t bottomRight = 0;
    const uint32_t rawTopLeft = ctl->topLeft();
    const uint32_t rawTopRight = ctl->topRight();
    const uint32_t rawBottomLeft = ctl->bottomLeft();
    const uint32_t rawBottomRight = ctl->bottomRight();
    const uint32_t rawLeft = rawTopLeft + rawBottomLeft;
    const uint32_t rawRight = rawTopRight + rawBottomRight;
    const uint32_t rawTotal = rawLeft + rawRight;
    const uint32_t rawLeftPercent = rawTotal ? rawLeft * 100 / rawTotal : 0;
    const uint32_t rawRightPercent = rawTotal ? rawRight * 100 / rawTotal : 0;

    if (!initialized) {
        topLeft = rawTopLeft;
        topRight = rawTopRight;
        bottomLeft = rawBottomLeft;
        bottomRight = rawBottomRight;
        initialized = true;
    } else {
        topLeft = (2 * topLeft + rawTopLeft) / 3;
        topRight = (2 * topRight + rawTopRight) / 3;
        bottomLeft = (2 * bottomLeft + rawBottomLeft) / 3;
        bottomRight = (2 * bottomRight + rawBottomRight) / 3;
    }

    const uint32_t left = topLeft + bottomLeft;
    const uint32_t right = topRight + bottomRight;
    const uint32_t front = topLeft + topRight;
    const uint32_t back = bottomLeft + bottomRight;
    const uint32_t total = left + right;
    const int32_t leftRightShift = static_cast<int32_t>(right) - static_cast<int32_t>(left);
    const int32_t frontBackShift = static_cast<int32_t>(front) - static_cast<int32_t>(back);
    const uint32_t deadzone = total / 8 > minimumDeadzoneGrams ? total / 8 : minimumDeadzoneGrams;
    const uint32_t frontBackDeadzone = deadzone * 115 / 100;
    const int32_t leftRightMagnitude = leftRightShift < 0 ? -leftRightShift : leftRightShift;
    const int32_t frontBackMagnitude = frontBackShift < 0 ? -frontBackShift : frontBackShift;

    const char* lean = "NO LOAD";
    if (total >= minimumLoadGrams) {
        if (leftRightMagnitude <= static_cast<int32_t>(deadzone) &&
            frontBackMagnitude <= static_cast<int32_t>(frontBackDeadzone)) {
            lean = "CENTER";
        } else if (leftRightMagnitude >= frontBackMagnitude) {
            lean = leftRightShift > 0 ? "RIGHT" : "LEFT";
        } else {
            lean = frontBackShift > 0 ? "FRONT" : "BACK";
        }
    }

    const char* lastStep = "NONE";
    bool stepEvent = false;
    if (!systemEnabled || rawTotal < kMinimumStepLoadGrams) {
        resetMotorOutput();
        lastBalanceBoardDataAt = now;
        lastStep = "NO LOAD";
        balanceBoardOccupied = false;
    } else {balanceBoardOccupied = true;
        balanceBoardOccupied = true;
        if (leanModeEnabled) {
            // Proportional front/back throttle outside frontBackDeadzone
            const int32_t maxFrontBackShift = static_cast<int32_t>(total / 2);
            if (frontBackMagnitude <= static_cast<int32_t>(frontBackDeadzone) ||
                maxFrontBackShift <= static_cast<int32_t>(frontBackDeadzone)) {
                leanFrontBackOffsetUs = 0;
            } else {
                const int32_t effectiveShift = frontBackMagnitude - static_cast<int32_t>(frontBackDeadzone);
                const int32_t effectiveSpan = maxFrontBackShift - static_cast<int32_t>(frontBackDeadzone);
                const int32_t signedOffset = constrain(
                    effectiveShift * kMaxLeanThrottleOffsetUs / effectiveSpan, 0, kMaxLeanThrottleOffsetUs);
                leanFrontBackOffsetUs = frontBackShift > 0 ? signedOffset : -signedOffset;
            }

            // Proportional left/right differential turn outside deadzone
            const int32_t maxLeftRightShift = static_cast<int32_t>(total / 2);
            if (leftRightMagnitude <= static_cast<int32_t>(deadzone) ||
                maxLeftRightShift <= static_cast<int32_t>(deadzone)) {
                leanLeftRightOffsetUs = 0;
            } else {
                const int32_t effectiveShift = leftRightMagnitude - static_cast<int32_t>(deadzone);
                const int32_t effectiveSpan = maxLeftRightShift - static_cast<int32_t>(deadzone);
                const int32_t signedOffset = constrain(
                    effectiveShift * kMaxLeanSteeringOffsetUs / effectiveSpan, 0, kMaxLeanSteeringOffsetUs);
                leanLeftRightOffsetUs = leftRightShift > 0 ? signedOffset : -signedOffset;
            }
        } else {
            if (rawLeftPercent >= kStepSidePercent && lastCountedSide != DominantSide::Left) {
                lastCountedSide = DominantSide::Left;
                recordStep(now);
                lastStep = "LEFT";
                stepEvent = true;
            } else if (rawRightPercent >= kStepSidePercent && lastCountedSide != DominantSide::Right) {
                lastCountedSide = DominantSide::Right;
                recordStep(now);
                lastStep = "RIGHT";
                stepEvent = true;
            }
            updateMotorSpeed(now, countRecentSteps(now)*2);
        }
    }

    const uint8_t stepsInWindow = countRecentSteps(now);
    const MotorPulses outputPulses = computeActivePulses();
    dumpBalanceBoard(ctl, topLeft, topRight, bottomLeft, bottomRight, lean, lastStep, totalStepCount,
                     stepsInWindow, outputPulses, stepEvent, balanceBoardOccupied);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
    writeMotorPulses(computeActivePulses());
}

void setup() {
    setupMotorPwm();
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);
    registerPwmTestConsoleCommand();
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    if (lastBalanceBoardDataAt != 0 && millis() - lastBalanceBoardDataAt > kBalanceBoardTimeoutMs) {
        resetMotorOutput();
    }

    updateStatusLed();

    delay(20);
}