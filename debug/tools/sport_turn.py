#!/usr/bin/env python3
"""Turn Go2 in place through the native Sport Move API."""

import argparse
import math
import threading
import time


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--direction",
        choices=("left", "right"),
        default="right",
        help="Turn direction; Unitree positive yaw rate is left.",
    )
    parser.add_argument(
        "--yaw-rate",
        type=float,
        default=0.8,
        help="Absolute yaw rate in rad/s (default: 0.8).",
    )
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument(
        "--angle-deg",
        type=float,
        help="Target rotation in degrees (default: 15).",
    )
    motion.add_argument(
        "--duration",
        type=float,
        help="Fixed command duration in seconds; disables angle feedback.",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Sport Move publishing rate (default: 50 Hz).",
    )
    parser.add_argument(
        "--interface",
        default="eth0",
        help="Network interface connected to the robot's internal network.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send motion commands; otherwise print the plan.",
    )
    return parser.parse_args()


def command_plan(args):
    yaw_rate = abs(float(args.yaw_rate))
    rate_hz = float(args.rate_hz)
    if not 0.5 <= yaw_rate <= 1.0:
        raise ValueError(
            "--yaw-rate must be between 0.5 and 1.0 rad/s; lower values "
            "can fall inside the GO2 Sport yaw deadband"
        )
    if not 10.0 <= rate_hz <= 200.0:
        raise ValueError("--rate-hz must be between 10 and 200")

    if args.duration is not None:
        duration = float(args.duration)
        angle_deg = None
        if not 0.05 <= duration <= 10.0:
            raise ValueError("--duration must be between 0.05 and 10 s")
    else:
        angle_deg = 15.0 if args.angle_deg is None else float(args.angle_deg)
        if not 0.5 <= angle_deg <= 180.0:
            raise ValueError("--angle-deg must be between 0.5 and 180")
        duration = None

    signed_rate = yaw_rate if args.direction == "left" else -yaw_rate
    return signed_rate, angle_deg, duration, rate_hz


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StateReader:
    def __init__(self, subscriber_type, message_type):
        self._lock = threading.Lock()
        self._sample = None
        self._subscriber = subscriber_type("rt/sportmodestate", message_type)
        self._subscriber.Init(self._receive, 10)

    def _receive(self, message):
        sample = (
            float(message.imu_state.rpy[2]),
            float(message.yaw_speed),
            int(message.mode),
            int(message.error_code),
        )
        with self._lock:
            self._sample = sample

    def wait(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._sample is not None:
                    return self._sample
            time.sleep(0.01)
        raise RuntimeError("no /sportmodestate data received")

    def latest(self):
        with self._lock:
            if self._sample is None:
                raise RuntimeError("no /sportmodestate data received")
            return self._sample

    def close(self):
        self._subscriber.Close()


def stop_robot(client):
    client.Move(0.0, 0.0, 0.0)
    time.sleep(0.10)
    return client.StopMove()


def execute(args, signed_rate, angle_deg, duration, rate_hz):
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelSubscriber,
    )
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    ChannelFactoryInitialize(0, args.interface)
    client = SportClient()
    client.SetTimeout(3.0)
    client.Init()
    reader = StateReader(ChannelSubscriber, SportModeState_)
    initial_yaw, _, mode, error_code = reader.wait(2.0)
    print(
        "state: mode=%d error_code=%d initial_yaw=%.3f deg"
        % (mode, error_code, math.degrees(initial_yaw)),
        flush=True,
    )

    if duration is not None:
        max_duration = duration
        target_rad = None
    else:
        target_rad = math.radians(angle_deg)
        # Allow for startup and stopping transients without turning forever.
        max_duration = max(2.0, 3.0 * target_rad / abs(signed_rate))

    period = 1.0 / rate_hz
    command_count = 0
    peak_yaw_speed = 0.0
    next_status = time.monotonic()
    started = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - started
            if target_rad is not None:
                yaw, yaw_speed, _, _ = reader.latest()
                traveled = abs(wrap_angle(yaw - initial_yaw))
                peak_yaw_speed = max(peak_yaw_speed, abs(yaw_speed))
                if traveled >= target_rad or elapsed >= max_duration:
                    break
            elif elapsed >= max_duration:
                break

            result = client.Move(0.0, 0.0, signed_rate)
            if result != 0:
                raise RuntimeError("Sport Move send failed with code %d" % result)
            command_count += 1

            now = time.monotonic()
            if now >= next_status:
                print(
                    "move: z=%+.3f rad/s elapsed=%.2f s commands=%d"
                    % (signed_rate, now - started, command_count),
                    flush=True,
                )
                next_status = now + 1.0
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print("interrupted; stopping robot", flush=True)
    finally:
        stop_result = stop_robot(client)

    time.sleep(0.45)
    final_yaw, final_yaw_speed, final_mode, final_error = reader.latest()
    reader.close()
    traveled = wrap_angle(final_yaw - initial_yaw)
    print(
        "done: yaw=%.3f deg delta=%+.3f deg peak_state_yaw_speed=%.3f "
        "final_yaw_speed=%.3f mode=%d error_code=%d stop=%d"
        % (
            math.degrees(final_yaw),
            math.degrees(traveled),
            peak_yaw_speed,
            final_yaw_speed,
            final_mode,
            final_error,
            stop_result,
        ),
        flush=True,
    )
    if abs(math.degrees(traveled)) < 0.5:
        raise RuntimeError("robot yaw did not change; Sport Move was not executed")


def main():
    args = parse_args()
    signed_rate, angle_deg, duration, rate_hz = command_plan(args)
    direction = "left" if signed_rate > 0.0 else "right"
    target = (
        "angle=%.1f deg" % angle_deg
        if angle_deg is not None
        else "duration=%.3f s" % duration
    )
    print(
        "Sport yaw plan: direction=%s z=%+.3f rad/s %s rate=%.1f Hz"
        % (direction, signed_rate, target, rate_hz)
    )
    if not args.execute:
        print("dry run; pass --execute or use run_sport_turn.sh")
        return
    execute(args, signed_rate, angle_deg, duration, rate_hz)


if __name__ == "__main__":
    main()
