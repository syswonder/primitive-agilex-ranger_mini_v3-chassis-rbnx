#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ranger_chassis_rbnx — atlas bridge (driver-init lifecycle).

Owns `primitive/chassis/*` for the AgileX Ranger Mini v2.

Spawn order:
  1. start.sh launches THIS process — no ROS spawn.
  2. main() RegisterCapability + declares ONLY `primitive/chassis/driver`,
     blocks awaiting Driver(CMD_INIT, config_json).
  3. rbnx boot calls Init with the manifest's config: block.
  4. Init handler spawns `ros2 launch ranger_bringup ranger_mini_v2.launch.xml`
     with config-driven args, waits for the first /odom message, then
     declares chassis/odom + chassis/twist_in on atlas.

move (rpc-mode contract) is TODO — needs its own gRPC handler
implementing chassis/srv/ExecuteMoveCommand. For the rtabmap-only
first bring-up the chassis isn't actively driven; topics-only
surface is sufficient. `primitive/chassis/state` is intentionally
NOT implemented — it's the legacy AMCL-bundling contract being
phased out (localization belongs to service/map/pose, not chassis).

Config (passed via Driver(CMD_INIT, config_json)):
    port_name        default "can_ranger"   (udev-renamed CAN; see /etc/systemd/network)
    robot_model      default "ranger_mini_v2"
    odom_frame       default "odom"
    base_frame       default "base_link"
    update_rate      default 50
    odom_topic_name  default "odom"
    publish_odom_tf  default false
    sentinel_timeout_s default 30.0
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from concurrent import futures
from pathlib import Path

logging.basicConfig(level=os.environ.get("RANGER_LOG_LEVEL", "INFO"),
                    format="[ranger_chassis] %(message)s")
log = logging.getLogger("ranger_chassis")


def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()

import grpc  # noqa: E402
import atlas_pb2 as pb  # noqa: E402
import atlas_pb2_grpc as pb_grpc  # noqa: E402
import lifecycle_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402

CMD_INIT = 0
CMD_ACTIVATE = 1
CMD_DEACTIVATE = 2
CMD_SHUTDOWN = 3


_state_lock = threading.Lock()
_atlas_stub: pb_grpc.AtlasStub | None = None
_cap_id: str = ""
_pkg_root: Path = Path(__file__).resolve().parent.parent
_ranger_proc: subprocess.Popen | None = None
_initialized = False


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
        args,
        stdout=log_fh, stderr=log_fh,
        start_new_session=True,
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


def _decl_topic(contract_id: str, topic: str, qos_profile: str = "reliable") -> None:
    if _atlas_stub is None:
        return
    _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=_cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_ROS2,
        endpoint=topic,
        params=pb.TransportParams(ros2=pb.Ros2Params(qos_profile=qos_profile)),
    ))


class _ChassisDriverServicer(contracts_grpc.PrimitiveChassisDriverServicer):
    def Driver(self, request, context):
        cmd = int(request.command)
        if cmd == CMD_INIT:
            try:
                cfg = json.loads(request.config_json) if request.config_json else {}
            except json.JSONDecodeError as e:
                return lifecycle_pb2.Driver_Response(
                    ok=False, state="error", error=f"bad config_json: {e}"
                )
            return self._init(cfg)
        if cmd == CMD_ACTIVATE:
            # primitives do all bring-up in CMD_INIT; ACTIVATE
            # is a framework no-op that flips the cap to ACTIVE
            # so consumers may begin calling.
            return lifecycle_pb2.Driver_Response(ok=True, state="active", error="")
        if cmd == CMD_DEACTIVATE:
            # framework no-op back to INACTIVE; v1 doesn't evict.
            return lifecycle_pb2.Driver_Response(ok=True, state="inactive", error="")
        if cmd == CMD_SHUTDOWN:
            _kill_ranger()
            return lifecycle_pb2.Driver_Response(ok=True, state="terminated", error="")
        return lifecycle_pb2.Driver_Response(
            ok=False, state="error", error=f"invalid command {cmd}"
        )

    def _init(self, cfg: dict):
        global _initialized
        with _state_lock:
            if _initialized:
                return lifecycle_pb2.Driver_Response(ok=True, state="inactive", error="")

        odom_topic_name = cfg.get("odom_topic_name", "odom")
        odom_topic = f"/{odom_topic_name.lstrip('/')}"
        twist_in_topic = cfg.get("twist_in_topic", "/cmd_vel")
        sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

        try:
            _spawn_ranger(cfg)
        except Exception as e:  # noqa: BLE001
            return lifecycle_pb2.Driver_Response(
                ok=False, state="error", error=f"spawn ranger failed: {e}"
            )

        if not _wait_for_odom(odom_topic, sentinel_timeout):
            _kill_ranger()
            return lifecycle_pb2.Driver_Response(
                ok=False, state="error",
                error=f"no Odometry on {odom_topic} within {sentinel_timeout:.1f}s "
                      f"(check CAN bus + chassis power)",
            )

        try:
            _decl_topic("robonix/primitive/chassis/odom", odom_topic)
            _decl_topic("robonix/primitive/chassis/twist_in", twist_in_topic)
        except grpc.RpcError as e:
            if e.code() != grpc.StatusCode.ALREADY_EXISTS:
                return lifecycle_pb2.Driver_Response(
                    ok=False, state="error", error=f"declare failed: {e.details()}"
                )

        with _state_lock:
            _initialized = True
        log.info("init complete: odom=%s twist_in=%s", odom_topic, twist_in_topic)
        return lifecycle_pb2.Driver_Response(ok=True, state="inactive", error="")


def _start_driver_grpc(port: int) -> None:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    contracts_grpc.add_PrimitiveChassisDriverServicer_to_server(
        _ChassisDriverServicer(), server
    )
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    log.info("LifecycleDriver gRPC serving on 0.0.0.0:%d", port)


def _decl_driver_iface(port: int) -> None:
    if _atlas_stub is None:
        return
    _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=_cap_id,
        contract_id="robonix/primitive/chassis/driver",
        transport=pb.TRANSPORT_GRPC,
        endpoint=f"127.0.0.1:{port}",
        params=pb.TransportParams(grpc=pb.GrpcParams(
            proto_file="robonix_contracts.proto",
            service_name="PrimitiveChassisDriver",
            method="Driver",
        )),
    ))


def _heartbeat_loop() -> None:
    while True:
        time.sleep(15.0)
        if _atlas_stub is None:
            continue
        try:
            _atlas_stub.Heartbeat(pb.HeartbeatRequest(capability_id=_cap_id))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat: %s", e)


def _on_signal(signum, _frame):
    log.info("signal %d — shutting down", signum)
    _kill_ranger()
    sys.exit(0)


def main() -> None:
    global _atlas_stub, _cap_id
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    driver_port = int(os.environ.get("RANGER_DRIVER_PORT", "50234"))
    _cap_id = os.environ.get(
        "ROBONIX_CAPABILITY_ID", "com.robonix.ranger.chassis"
    )

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    _start_driver_grpc(driver_port)

    channel = grpc.insecure_channel(atlas_addr)
    _atlas_stub = pb_grpc.AtlasStub(channel)
    pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
    md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
    try:
        _atlas_stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=_cap_id,
            namespace="robonix/primitive/chassis",
            capability_md_path=md_path,
        ))
        _decl_driver_iface(driver_port)
        log.info("registered cap %s, driver iface on :%d (awaiting INIT)",
                 _cap_id, driver_port)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("cap %s already registered (re-deploy); ok", _cap_id)
        else:
            log.warning("atlas registration failed: %s", e)

    threading.Thread(target=_heartbeat_loop, daemon=True).start()
    log.info("ready — awaiting Driver(CMD_INIT)")
    try:
        while True:
            time.sleep(60.0)
    except KeyboardInterrupt:
        pass
    finally:
        _kill_ranger()


if __name__ == "__main__":
    main()
