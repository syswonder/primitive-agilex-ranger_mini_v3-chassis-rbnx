# ranger_chassis_rbnx

Robonix package wrapping the **AgileX Ranger Mini v3** chassis. Owns the `primitive/chassis/*` namespace.

## Capability surface

| Contract                            | Mode      | Transport | Source / handler                            |
| ----------------------------------- | --------- | --------- | ------------------------------------------- |
| `robonix/primitive/chassis/driver`  | rpc       | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle |
| `robonix/primitive/chassis/odom`    | topic_out | ROS 2     | `/odom` (nav_msgs/Odometry)                 |
| `robonix/primitive/chassis/twist_in`| topic_in  | ROS 2     | `/cmd_vel` (geometry_msgs/Twist)            |

`primitive/chassis/move` (rpc-mode) needs its own gRPC handler implementing `chassis/srv/ExecuteMoveCommand` — TODO. We do NOT implement `primitive/chassis/state`: that legacy contract bundled map-frame pose into a chassis-owned RPC, which is a layering inversion (chassis primitives are device leaves; localization belongs to the localization service). For "where is the robot in the map?" subscribe to `service/map/pose` instead.

## Driver-init lifecycle

`start.sh` brings up the atlas bridge — no ROS spawn. The bridge opens a gRPC server (default port 50234), registers the capability and declares only `primitive/chassis/driver`, then blocks awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` calls Init, the handler parses config (CAN port, robot model, frames, update rate), spawns `ros2 launch ranger_bringup ranger_mini_v2.launch.xml`, waits for the first `nav_msgs/Odometry` on `/<odom_topic_name>`, and declares `chassis/odom` + `chassis/twist_in` on atlas.

If the chassis isn't powered or CAN isn't up, the sentinel times out and Init returns `state="error"` (NOT deferred — the chassis owns its process; we know it's stuck if we just spawned it and got nothing).

## Layout

```
ranger_chassis_rbnx/
├── package_manifest.yaml
├── ranger_chassis/atlas_bridge.py
├── scripts/
│   ├── build.sh        colcon build vendored sources + rbnx codegen
│   └── start.sh        source ROS, exec atlas_bridge
└── src/
    ├── ranger_ros2/    VENDORED agilexrobotics/ranger_ros2
    └── ugv_sdk/        VENDORED agilexrobotics/ugv_sdk (CAN bus client)
```

## CAN bus naming

The CAN interface name is config-driven — set `port_name:` in your deploy manifest's `config:` block to whatever the host calls its CAN device (`can0`, `can_ranger`, `can_chassis`, …). The package never hardcodes a device name, so it works on any host regardless of how its CAN interfaces are named.

When multiple CAN devices share a host you may want to rename the Ranger's interface to something stable via `udev` / `systemd-networkd` so the name doesn't shift across boots, then point `port_name:` at the rename. That's a host-side operation — the package only sees the name you give it via config.

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "port_name":         "can0",
  "robot_model":       "ranger_mini_v2",
  "odom_frame":        "odom",
  "base_frame":        "base_link",
  "update_rate":       50,
  "odom_topic_name":   "odom",
  "publish_odom_tf":   false,
  "twist_in_topic":    "/cmd_vel",
  "sentinel_timeout_s": 30.0
}
```

`publish_odom_tf` defaults to false; the canonical `base_link → odom` TF should come from soma's robot_state_publisher chain (URDF). Setting this to true here means competing publishers on `/tf` — only useful if soma isn't deployed.

## License

This package: Apache-2.0. Vendored ranger_ros2 / ugv_sdk: see their respective LICENSE files.
