# ranger_chassis_rbnx

Robonix package wrapping the **AgileX Ranger Mini v2** chassis. Owns
the `primitive/chassis/*` namespace.

## Capability surface

| Contract                            | Mode      | Transport | Source / handler                            |
| ----------------------------------- | --------- | --------- | ------------------------------------------- |
| `robonix/primitive/chassis/driver`  | rpc       | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle |
| `robonix/primitive/chassis/odom`    | topic_out | ROS 2     | `/odom` (nav_msgs/Odometry)                 |
| `robonix/primitive/chassis/twist_in`| topic_in  | ROS 2     | `/cmd_vel` (geometry_msgs/Twist)            |

`primitive/chassis/state` and `primitive/chassis/move` (rpc-mode) need
their own gRPC handlers implementing `chassis/srv/{GetRobotState,
ExecuteMoveCommand}` — TODO; not on the critical path for the rtabmap
bring-up which only consumes odom.

## Driver-init lifecycle

`start.sh` brings up the atlas bridge — no ROS spawn. The bridge:

1. opens a gRPC server (default port 50234),
2. registers the capability and declares only `primitive/chassis/driver`,
3. blocks awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` calls Init, the handler:

1. parses config (CAN port, robot model, frames, update rate),
2. spawns `ros2 launch ranger_bringup ranger_mini_v2.launch.xml`,
3. waits for the first `nav_msgs/Odometry` on `/<odom_topic_name>`,
4. declares `chassis/odom` + `chassis/twist_in` on atlas.

If the chassis isn't powered or CAN isn't up, the sentinel times out and
Init returns `state="error"` (NOT deferred — the chassis owns its
process; we know it's stuck if we just spawned it and got nothing).

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

The Jetson uses udev to rename the Ranger's CAN interface from `can0`
to `can_ranger` so it's stable across reboots and side-by-side with
other CAN devices. See `/etc/systemd/network/` on the robot. Override
with `port_name: can0` in the deploy manifest if you're testing on a
host that hasn't been re-configured.

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "port_name":         "can_ranger",
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

`publish_odom_tf` defaults to false; the canonical
`base_link → odom` TF should come from soma's robot_state_publisher
chain (URDF). Setting this to true here means competing publishers on
`/tf` — only useful if soma isn't deployed.

## License

This package: Apache-2.0.
Vendored ranger_ros2 / ugv_sdk: see their respective LICENSE files.
