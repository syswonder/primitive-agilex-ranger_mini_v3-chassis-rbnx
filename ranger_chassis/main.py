#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ranger_chassis_rbnx — AgileX Ranger Mini chassis primitive
(capability_id=ranger_chassis).

Owns `robonix/primitive/chassis/*`. Wraps the upstream `ranger_ros2`
launch which talks to the Ranger Mini's CAN bus via `ugv_sdk`.

Lifecycle:
    on_init      — spawn ranger_mini_v2.launch.xml, wait for first
                   /odom message, declare chassis/odom + chassis/twist_in.
    on_shutdown  — kill ranger subprocess.

Config (from manifest):
    port_name        default "can0"          — CAN interface (override per host)
    robot_model      default "ranger_mini_v2"
    odom_frame       default "odom"
    base_frame       default "base_link"
    update_rate      default 50
    odom_topic_name  default "odom"
    publish_odom_tf  default false           — let robot_state_publisher own /tf
    cmd_vel_topic    default "/cmd_vel"      — what the chassis subscribes to
    sentinel_timeout_s default 30.0
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from robonix_api import Capability, Ok, Err

logging.basicConfig(
    level=os.environ.get("RANGER_LOG_LEVEL", "INFO"),
    format="[ranger] %(message)s",
)
log = logging.getLogger("ranger")

cap = Capability(id="ranger_chassis", namespace="robonix/primitive/chassis")

_pkg_root: Path = Path(__file__).resolve().parent.parent
_ranger_proc: subprocess.Popen | None = None


def _spawn_ranger(cfg: dict) -> None:
    global _ranger_proc
    args = [
        "ros2", "launch", "ranger_bringup", "ranger_mini_v2.launch.xml",
        f"port_name:={cfg.get('port_name', 'can_ranger')}",
        f"robot_model:={cfg.get('robot_model', 'ranger_mini_v2')}",
        f"odom_frame:={cfg.get('odom_frame', 'odom')}",
        f"base_frame:={cfg.get('base_frame', 'base_link')}",
        f"update_rate:={int(cfg.get('update_rate', 50))}",
        f"odom_topic_name:={cfg.get('odom_topic_name', 'odom')}",
        f"publish_odom_tf:={'true' if cfg.get('publish_odom_tf', False) else 'false'}",
    ]
    log_path = _pkg_root / "rbnx-build" / "data" / "ranger.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning ranger driver → %s", log_path)
    log.debug("launch args: %s", " ".join(args))
    _ranger_proc = subprocess.Popen(
        args, stdout=log_fh, stderr=log_fh, start_new_session=True,
    )


def _kill_ranger() -> None:
    p = _ranger_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def _wait_for_odom(topic: str, timeout_s: float) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from nav_msgs.msg import Odometry
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("ranger_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(Odometry, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first odom on %s — up to %.1fs", topic, timeout_s)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if seen.is_set():
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return seen.is_set()


@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: spawn ranger driver, wait for odom, declare."""
    odom_topic = "/" + cfg.get("odom_topic_name", "odom").lstrip("/")
    cmd_vel_topic = cfg.get("cmd_vel_topic", "/cmd_vel")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    try:
        _spawn_ranger(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn ranger failed: {e}")

    if not _wait_for_odom(odom_topic, sentinel_timeout):
        _kill_ranger()
        return Err(f"no Odometry on {odom_topic} within {sentinel_timeout:.1f}s")

    cap.declare_ros2(
        "robonix/primitive/chassis/odom",
        topic=odom_topic,
        qos="reliable",
    )
    cap.declare_ros2(
        "robonix/primitive/chassis/twist_in",
        topic=cmd_vel_topic,
        qos="reliable",
    )
    log.info("init complete: odom=%s twist_in=%s", odom_topic, cmd_vel_topic)
    return Ok()


@cap.on_shutdown
def shutdown():
    _kill_ranger()
    return Ok()


if __name__ == "__main__":
    cap.run()
