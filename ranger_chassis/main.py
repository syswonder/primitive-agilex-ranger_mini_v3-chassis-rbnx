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

from robonix_api import Err, Ok, Primitive

logging.basicConfig(
    level=os.environ.get("RANGER_LOG_LEVEL", "INFO"),
    format="[ranger] %(message)s",
)
log = logging.getLogger("ranger")

cap = Primitive(id="ranger_chassis", namespace="robonix/primitive/chassis")

_pkg_root: Path = Path(__file__).resolve().parent.parent
_ranger_proc: subprocess.Popen | None = None


def _setup_can(cfg: dict) -> None:
    """Bring the chassis CAN interface up before launching the driver — the
    primitive owns this so the host doesn't need the link pre-configured. The
    SDK only opens an already-up SocketCAN device, so we down → set bitrate →
    up the configured `port_name`. Idempotent; needs CAP_NET_ADMIN (passwordless
    `sudo ip link …`, or run as root). Logs + continues on failure so the
    underlying 'Failed to connect to CAN port' still surfaces if the host
    genuinely can't bring it up."""
    import shutil
    port = cfg.get("port_name", "can_ranger")
    bitrate = str(int(cfg.get("can_bitrate", 500000)))
    sudo = ["sudo", "-n"] if os.geteuid() != 0 and shutil.which("sudo") else []
    for c in (["ip", "link", "set", port, "down"],
              ["ip", "link", "set", port, "type", "can", "bitrate", bitrate],
              ["ip", "link", "set", port, "up"]):
        r = subprocess.run(sudo + c, capture_output=True, text=True)
        if r.returncode != 0:
            log.warning("CAN setup '%s' failed: %s", " ".join(c), (r.stderr or "").strip())
    log.info("CAN %s configured up @ %s bps", port, bitrate)


def _pump_output(stream) -> None:
    """Forward the ranger driver's merged stdout/stderr into scribe via the
    package logger — one unified log stream, no side-car ranger.log file."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[ranger_base] %s", line)


def _spawn_ranger(cfg: dict) -> None:
    global _ranger_proc
    _setup_can(cfg)
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
    log.info("spawning ranger driver")
    log.debug("launch args: %s", " ".join(args))
    _ranger_proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
    )
    threading.Thread(target=_pump_output, args=(_ranger_proc.stdout,), daemon=True).start()


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

    cap.declare_ros2_topic(
        "robonix/primitive/chassis/odom",
        topic=odom_topic,
        qos="reliable",
    )
    cap.declare_ros2_topic(
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
