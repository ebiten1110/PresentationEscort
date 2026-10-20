#include "../src/safety_watchdog.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

int main() {
    using State = SafetyWatchdog::State;
    SafetyWatchdog w;
    assert(w.state() == State::DISARMED);
    assert(!w.motionAllowed(0));
    assert(strcmp(w.stateName(), "DISARMED") == 0);

    assert(w.arm(100));
    assert(w.motionAllowed(100));
    assert(w.ping(250));
    assert(w.motionAllowed(999)); // 749ms from last heartbeat.
    assert(w.update(1000) == false); // Boundary 750ms.
    assert(w.update(1001) == true);
    assert(w.state() == State::LATCHED);
    assert(!w.motionAllowed(1001));
    assert(!w.ping(1002)); // Late heartbeat must NOT re-arm.
    assert(!w.arm(1003));
    w.stop();  // STOP may not unlock latched faults.
    assert(w.state() == State::LATCHED);
    assert(w.reset());
    assert(w.state() == State::DISARMED);
    assert(!w.motionAllowed(1004));
    assert(w.arm(1100));
    w.stop();
    assert(w.state() == State::DISARMED);
    assert(!w.motionAllowed(1101));

    assert(w.arm(1200));
    w.estop();
    assert(w.state() == State::LATCHED);
    assert(!w.motionAllowed(1201));
    assert(w.reset());
    assert(!w.motionAllowed(1202));

    // Unsigned subtraction is wraparound-safe for durations much < 2^31ms.
    assert(w.arm(UINT32_MAX - 250U));
    assert(w.ping(UINT32_MAX - 30U));
    assert(w.motionAllowed(100U));
    assert(w.update(730U)); // >750ms after wrap.
    assert(w.state() == State::LATCHED);
    return 0;
}
