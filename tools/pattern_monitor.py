#!/usr/bin/env python3
"""Discover and record ROS 2 topics while plotting a motor test in real time."""

import argparse
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import time


def parse_status(value):
    parts = [part.strip() for part in value.split(";")]
    fields = {"event": parts[0]}
    fields.update(part.split("=", 1) for part in parts[1:] if "=" in part)
    return fields


def to_jsonable(value):
    if hasattr(value, "get_fields_and_field_types"):
        return {name: to_jsonable(getattr(value, name))
                for name in value.get_fields_and_field_types()}
    if isinstance(value, (list, tuple)) or hasattr(value, "tolist"):
        return [to_jsonable(item) for item in value]
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="/", help="ROS topic namespace to monitor")
    parser.add_argument("--output", type=Path, help="JSONL record path (default: timestamped file)")
    parser.add_argument("--interval", type=float, default=0.2, help="plot refresh interval in seconds")
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be positive")

    try:
        import matplotlib.pyplot as plt
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        parser.error(f"missing dependency {exc.name}; source ROS 2 and install python3-matplotlib")

    output = args.output or Path.cwd() / ("pattern_topics_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".jsonl")
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("pattern_monitor")
    subscriptions = {}
    topic_types = {}
    counts = {}
    run_origin = None
    xs, ys = [], []
    rpm_t = deque(maxlen=1500)
    rpm_target_l = deque(maxlen=1500)
    rpm_target_r = deque(maxlen=1500)
    rpm_actual_l = deque(maxlen=1500)
    rpm_actual_r = deque(maxlen=1500)
    path_t = deque(maxlen=1500)
    path_error = deque(maxlen=1500)
    events = []
    status = {}
    diagnostic = "Waiting for diagnostics"
    latest = {"target": [math.nan, math.nan], "actual": [math.nan, math.nan]}
    started = time.monotonic()
    last_discovery = 0.0
    last_plot = 0.0
    dirty = True

    plt.ion()
    fig, (ax_path, ax_rpm, ax_error) = plt.subplots(1, 3, figsize=(17, 6))
    fig.canvas.manager.set_window_title("ROS 2 pattern monitor")
    fig.subplots_adjust(bottom=0.25, top=0.85, wspace=0.32)
    path_line, = ax_path.plot([], [], color="#1269a8", linewidth=2, label="wheel odometry")
    current_dot, = ax_path.plot([], [], "o", color="#1269a8")
    event_dots = ax_path.scatter([], [], marker="x", color="#d62728", s=90, label="warning / fault")
    ax_path.set(title="Travel path (wheel odometry)", xlabel="x from start (m)", ylabel="y from start (m)")
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.grid(True)
    ax_path.legend(loc="best")
    rpm_lines = [ax_rpm.plot([], [], label=label, color=color, linestyle=style)[0] for label, color, style in (
        ("left target", "#1f77b4", "--"), ("left actual", "#1f77b4", "-"),
        ("right target", "#ff7f0e", "--"), ("right actual", "#ff7f0e", "-"))]
    ax_rpm.set(title="Wheel RPM", xlabel="monitor time (s)", ylabel="RPM")
    ax_rpm.grid(True)
    ax_rpm.legend(loc="best")
    error_line, = ax_error.plot([], [], color="#9467bd", label="reported path error")
    ax_error.axhline(0, color="gray", linewidth=0.8)
    ax_error.set(title="Pattern path error", xlabel="monitor time (s)", ylabel="error (m)")
    ax_error.grid(True)
    ax_error.legend(loc="best")
    header = fig.text(0.03, 0.95, "Waiting for motion_test/status", fontsize=11)
    footer = fig.text(0.03, 0.08, "Discovering topics...", fontsize=9, family="monospace")
    event_text = fig.text(0.03, 0.15, "", fontsize=9, color="#b22222")

    def relative_topic(name):
        return name == args.namespace.rstrip("/") or name.startswith(args.namespace.rstrip("/") + "/")

    def add_event(label):
        nonlocal dirty
        events.append((xs[-1] if xs else math.nan, ys[-1] if ys else math.nan, label))
        print("EVENT:", label, flush=True)
        dirty = True

    with output.open("w", encoding="utf-8", buffering=1) as record:
        def callback(topic, message):
            nonlocal run_origin, status, diagnostic, dirty
            elapsed = time.monotonic() - started
            counts[topic] = counts.get(topic, 0) + 1
            record.write(json.dumps({"received_utc": datetime.utcnow().isoformat(timespec="milliseconds") + "Z",
                                     "elapsed_s": elapsed, "topic": topic, "type": topic_types[topic],
                                     "message": to_jsonable(message)}, ensure_ascii=False) + "\n")
            base = topic.rsplit("/", 1)[-1]
            if base == "odom" and hasattr(message, "pose"):
                p = message.pose.pose.position
                if run_origin is None:
                    run_origin = (p.x, p.y)
                xs.append(p.x - run_origin[0])
                ys.append(p.y - run_origin[1])
                dirty = True
            elif base in ("motor_rpm_command", "wheel_speed_feedback") and len(message.data) >= 2:
                latest["target" if base == "motor_rpm_command" else "actual"] = list(message.data[:2])
                rpm_t.append(elapsed)
                rpm_target_l.append(latest["target"][0]); rpm_target_r.append(latest["target"][1])
                rpm_actual_l.append(latest["actual"][0]); rpm_actual_r.append(latest["actual"][1])
                dirty = True
            elif topic.endswith("/motion_test/status"):
                old = status
                status = parse_status(message.data)
                if status.get("event") == "started":
                    xs.clear(); ys.clear(); run_origin = None
                    rpm_t.clear(); rpm_target_l.clear(); rpm_target_r.clear()
                    rpm_actual_l.clear(); rpm_actual_r.clear(); path_t.clear(); path_error.clear()
                    events.clear()
                if status.get("state") == "aborted" and old.get("state") != "aborted":
                    add_event("TEST ABORTED: " + status.get("reason", "unknown"))
                try:
                    path_error.append(float(status["path_error_m"]))
                    path_t.append(elapsed)
                except (KeyError, ValueError):
                    pass
                dirty = True
            elif base == "diagnostics" and hasattr(message, "status"):
                for item in message.status:
                    diagnostic = item.message
                    if item.level >= 1 and (not events or events[-1][2] != item.message):
                        add_event(item.message)
                dirty = True
            elif base == "motor_fault":
                add_event("MOTOR FAULT: " + str(list(message.data)))

        def discover():
            for topic, types in node.get_topic_names_and_types():
                if topic in subscriptions or not relative_topic(topic) or not types:
                    continue
                if len(types) != 1:
                    print(f"SKIP {topic}: ambiguous types {types}", flush=True)
                    continue
                try:
                    message_type = get_message(types[0])
                    endpoints = node.get_publishers_info_by_topic(topic)
                    # BEST_EFFORT receives both reliable and best-effort publishers.
                    durability = DurabilityPolicy.TRANSIENT_LOCAL if endpoints and all(
                        endpoint.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                        for endpoint in endpoints) else DurabilityPolicy.VOLATILE
                    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=durability)
                    topic_types[topic] = types[0]
                    subscriptions[topic] = node.create_subscription(
                        message_type, topic, lambda msg, name=topic: callback(name, msg), qos)
                    print(f"MONITOR {topic} [{types[0]}]", flush=True)
                except Exception as exc:
                    print(f"SKIP {topic}: {exc}", flush=True)

        print(f"Recording all discovered topics to {output}", flush=True)
        try:
            while plt.fignum_exists(fig.number) and rclpy.ok():
                now = time.monotonic()
                if now - last_discovery >= 1.0:
                    discover()
                    last_discovery = now
                rclpy.spin_once(node, timeout_sec=0.02)
                if dirty and now - last_plot >= args.interval:
                    path_line.set_data(xs, ys)
                    current_dot.set_data(xs[-1:] if xs else [], ys[-1:] if ys else [])
                    warning_points = [(x, y) for x, y, _ in events if math.isfinite(x)]
                    event_dots.set_offsets(warning_points if warning_points else [[math.nan, math.nan]])
                    if xs:
                        ax_path.relim(); ax_path.autoscale_view()
                    for line, values in zip(rpm_lines, (rpm_target_l, rpm_actual_l, rpm_target_r, rpm_actual_r)):
                        line.set_data(rpm_t, values)
                    if rpm_t:
                        ax_rpm.relim(); ax_rpm.autoscale_view()
                    error_line.set_data(path_t, path_error)
                    if path_t:
                        ax_error.relim(); ax_error.autoscale_view()
                    header.set_text("Pattern: {kind}  state: {state}  segment: {segment}  |  {diagnostic}".format(
                        kind=status.get("kind", "?"), state=status.get("state", "?"),
                        segment=status.get("segment", "?"), diagnostic=diagnostic))
                    footer.set_text(f"Topics: {len(subscriptions)}  Messages: {sum(counts.values())}  "
                                    f"Odometry samples: {len(xs)}  Log: {output}")
                    event_text.set_text("Latest issue: " + events[-1][2] if events else "")
                    fig.canvas.draw_idle()
                    dirty = False
                    last_plot = now
                plt.pause(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            print(f"Saved {sum(counts.values())} messages from {len(counts)} topics to {output}", flush=True)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
