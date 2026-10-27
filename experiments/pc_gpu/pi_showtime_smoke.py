"""Fast full-stack software smoke test. NEVER opens a camera or real motors.

The test exercises: V2 detection -> operator selection -> presenter lock ->
V8.8.3 planning -> PiMotionGateway -> VirtualEsp32 V1 ARM/status/STOP.
Real hardware operation is intentionally outside this smoke test.
"""
from __future__ import annotations

import time

from detection_protocol import DetectionResult
from pi_presenter_lock_demo import LockedSimulationRuntime


def frame(seq: int, *, visible: bool = True) -> DetectionResult:
    people = (
        [{"x1": 208, "y1": 25, "x2": 278, "y2": 205, "confidence": 0.88}]
        if visible else []
    )
    return DetectionResult(
        found=visible, frame_width=320, frame_height=240,
        protocol_version=2, frame_seq=seq, people=people,
        inference_ms=40.0,
    )


def wait_mode(runtime: LockedSimulationRuntime, expected: str, timeout: float = 0.45) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if runtime.status()["mode"] == expected:
            return
        time.sleep(0.02)
    raise AssertionError(f"Expected {expected}, got {runtime.status()['mode']}")


def check(name: str, condition: bool) -> None:
    if not condition:
        raise AssertionError(name)
    print(f"[PASS] {name}", flush=True)


def run_smoke() -> None:
    runtime = LockedSimulationRuntime()
    runtime.start()
    try:
        first = frame(1)
        runtime.process_detection(first, 75.0)
        check("no automatic presenter acquisition", not runtime.status()["auto_ready"])
        check("AUTO blocked before operator selection", not runtime.action("auto")["ok"])

        chosen = runtime.target_action({"index": 0, "frame_seq": 1})
        check("operator explicitly selects presenter", chosen["ok"])
        for seq in range(2, 6):
            runtime.process_detection(frame(seq), 75.0)
        check("locked valid presenter enables readiness", runtime.status()["auto_ready"])

        arm = runtime.action("auto")
        check("ESP32 safety ARM handshake initiated", arm["ok"])
        wait_mode(runtime, "AUTO")
        check("virtual ESP32 ARM acknowledged", runtime.gateway.firmware_safety_status() == "ARMED")

        for seq in range(6, 12):
            runtime.process_detection(frame(seq), 75.0)
            time.sleep(0.04)

        commands = list(runtime.sink.commands)
        check("tracking logic reaches virtual ESP32",
              any(c.startswith("HEAD_POSE_SET:") for c in commands))
        check("forward/backward motion remains blocked",
              not any(c in ("TRACK_FORWARD", "TRACK_BACKWARD") for c in commands))

        runtime.process_detection(frame(12, visible=False), 75.0)
        wait_mode(runtime, "SAFE")
        check("missing presenter triggers STOP", "STOP" in runtime.sink.commands)
        check("lost presenter invalidates target lock",
              not runtime.status()["presenter_selected"])
        check("AUTO cannot rearm without re-selection",
              not runtime.action("auto")["ok"])

        runtime.action("estop")
        wait_mode(runtime, "ESTOP")
        check("explicit ESTOP latches", not runtime.action("auto")["ok"])
        print("\nFULL-STACK SOFTWARE SMOKE: PASS (VIRTUAL MOTORS ONLY)", flush=True)
    finally:
        runtime.close()


if __name__ == "__main__":
    try:
        run_smoke()
    except (AssertionError, RuntimeError, OSError) as exc:
        print(f"\nFULL-STACK SOFTWARE SMOKE: FAIL: {exc}", flush=True)
        raise SystemExit(1)
