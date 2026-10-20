#ifndef PRESENTATION_ESCORT_SAFETY_WATCHDOG_H
#define PRESENTATION_ESCORT_SAFETY_WATCHDOG_H

#include <stdint.h>

// Arduino-independent state machine, unit-tested by a native C++ compiler.
// Does not replace a physical emergency power cutoff or guarantee balance.
class SafetyWatchdog {
public:
    enum class State : uint8_t {
        DISARMED,
        ARMED,
        LATCHED,
    };

    static constexpr uint32_t TIMEOUT_MS = 750U;

    SafetyWatchdog() : state_(State::DISARMED), last_ping_ms_(0U) {}

    State state() const { return state_; }

    const char* stateName() const {
        switch (state_) {
            case State::DISARMED: return "DISARMED";
            case State::ARMED: return "ARMED";
            default: return "LATCHED";
        }
    }

    bool arm(uint32_t now_ms) {
        if (state_ != State::DISARMED) return false;
        state_ = State::ARMED;
        last_ping_ms_ = now_ms;
        return true;
    }

    bool update(uint32_t now_ms) {
        if (state_ == State::ARMED
            && static_cast<uint32_t>(now_ms - last_ping_ms_) > TIMEOUT_MS) {
            state_ = State::LATCHED;
            return true;  // Just transitioned to fault.
        }
        return false;
    }

    bool ping(uint32_t now_ms) {
        update(now_ms);  // A late ping MUST NOT clear a fault.
        if (state_ != State::ARMED) return false;
        last_ping_ms_ = now_ms;
        return true;
    }

    bool motionAllowed(uint32_t now_ms) {
        update(now_ms);
        return state_ == State::ARMED;
    }

    void stop() {
        // STOP is always honored. It disarms without clearing a latched fault.
        if (state_ == State::ARMED) state_ = State::DISARMED;
    }

    void estop() { state_ = State::LATCHED; }

    bool reset() {
        if (state_ != State::LATCHED) return false;
        state_ = State::DISARMED;  // Still needs a separate explicit ARM.
        return true;
    }

private:
    State state_;
    uint32_t last_ping_ms_;
};

#endif
