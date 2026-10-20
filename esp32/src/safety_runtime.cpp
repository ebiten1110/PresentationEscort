#include "safety_runtime.h"

#include "safety_watchdog.h"
#include "head_control.h"
#include "walking.h"

static SafetyWatchdog watchdog;

static void holdCurrentHeadPosition() {
    // Do NOT center servos on a timeout: centering is additional movement.
    // Smooth head pursuit is stopped at its last commanded current position.
    setHeadPoseTarget(getCurrentHeadYaw(), getCurrentHeadPitch());
}

static void stopMotionSafely() {
    stopWalking();
    holdCurrentHeadPosition();
}

void initSafetyRuntime() {
    // The watchdog is DISARMED at construction. Existing production V8.8.3
    // motion commands WILL NOT work with this new firmware unless the Pi
    // implements explicit arm + periodic heartbeat. Do not flash prematurely.
    Serial.println("[Safety] Protocol=V1 TimeoutMs=750");
    printSafetyStatus();
}

void updateSafetyRuntime() {
    if (watchdog.update(millis())) {
        Serial.println("[Safety] FAULT=HEARTBEAT_TIMEOUT");
        stopMotionSafely();
        printSafetyStatus();
    }
}

bool isSafetyMotionAllowed() {
    // Always take the STOP action when timeout is detected, even if
    // the deadline passes during processing of the serial input queue.
    updateSafetyRuntime();
    return watchdog.state() == SafetyWatchdog::State::ARMED;
}

void printSafetyStatus() {
    Serial.print("[Safety] State: ");
    Serial.println(watchdog.stateName());
}

void safetyDisarmAndStop() {
    watchdog.stop();
    stopMotionSafely();
    printSafetyStatus();
}

bool handleSafetyProtocolCommand(const String& command) {
    if (command == "SAFETY_PING") {
        // A queued late ping must not silently transition to a fault
        // without calling stopWalking().
        updateSafetyRuntime();
        // Do not flood serial logs at 5-10 Hz.
        watchdog.ping(millis());
        return true;
    }
    if (command == "SAFETY_ARM") {
        updateSafetyRuntime();
        // Refuse arm during an unfinished body pulse.
        if (isWalkingBusy() || !watchdog.arm(millis())) {
            Serial.println("[Safety] ARM_REJECTED");
        } else {
            Serial.println("[Safety] ARM_ACCEPTED");
        }
        printSafetyStatus();
        return true;
    }
    if (command == "SAFETY_DISARM") {
        safetyDisarmAndStop();
        return true;
    }
    if (command == "SAFETY_ESTOP") {
        watchdog.estop();
        stopMotionSafely();
        Serial.println("[Safety] FAULT=REMOTE_ESTOP");
        printSafetyStatus();
        return true;
    }
    if (command == "SAFETY_RESET") {
        // Explicit operator action only, NEVER sent automatically by Pi.
        if (isWalkingBusy() || !watchdog.reset()) {
            Serial.println("[Safety] RESET_REJECTED");
        } else {
            Serial.println("[Safety] RESET_ACCEPTED");
        }
        printSafetyStatus();
        return true;
    }
    if (command == "SAFETY_STATUS") {
        printSafetyStatus();
        return true;
    }
    return false;
}
