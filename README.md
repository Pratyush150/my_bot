# ros2-diffdrive-robot

A complete ROS 2 differential-drive robot: Xacro description, Gazebo simulation, and a
serial driver that talks to a microcontroller motor controller and reads encoders back.

It covers the whole path from a velocity command to a turning wheel and back to
measured speed. That path is where most small robot projects break, and having it end
to end in one workspace makes each break easy to isolate.

**Scope: this is a learning-derived reference implementation, not a production robot.**
It was built by working through the Articulated Robotics ROS 2 material, and the
`serial_motor_demo` package is adapted from that project's open-source code. It is here
because the workspace is a clean, working example of the full command-to-encoder loop —
not because it is original system design. See [Attribution](#attribution) below.

---

## What it actually does

**Description.** A Xacro robot description split into separate files: chassis and
wheels, LiDAR, camera, and the Gazebo differential-drive plugin configuration. Inertial
macros compute real inertia tensors from mass and geometry rather than leaving
placeholder values, so the model does not misbehave the moment physics is enabled.

**Simulation.** A launch file that starts `robot_state_publisher`, brings up `gzserver`
and `gzclient` with the ROS init and factory plugins, and spawns the robot from the
`/robot_description` topic after a delay so the server is ready first. The delay is not
elegance; it is the fix for a race that otherwise fails intermittently.

**Hardware driver.** A `MotorDriver` node that opens a serial connection to a
microcontroller motor controller and exposes two control modes over ROS 2 topics:

- **PWM mode** — raw motor values, open loop. Useful for wiring and direction checks.
- **Closed-loop mode** — commanded angular velocity in rad/s, with the microcontroller
  running the wheel PID.

The node subscribes to `motor_command`, and publishes `motor_vels` (measured rad/s,
derived from encoder deltas over elapsed time) and `encoder_vals` (raw counts). Serial
access is mutex-guarded because the command callback and the encoder read timer run on
different threads and interleaved writes on a serial line produce corrupted frames that
are miserable to debug.

**Parameters that matter.** `encoder_cpr` (counts per revolution — wrong here means all
reported velocities are wrong by a constant factor), `loop_rate`, `serial_port`,
`baud_rate`, and `serial_debug` to echo raw traffic when the protocol misbehaves.

---

## Topics and messages

| Topic | Type | Direction |
|---|---|---|
| `motor_command` | `serial_motor_demo_msgs/MotorCommand` | subscribed |
| `motor_vels` | `serial_motor_demo_msgs/MotorVels` | published |
| `encoder_vals` | `serial_motor_demo_msgs/EncoderVals` | published |

```
MotorCommand: bool is_pwm, float32 mot_1_req_rad_sec, float32 mot_2_req_rad_sec
MotorVels:    float32 mot_1_rad_sec, float32 mot_2_rad_sec
EncoderVals:  int32 mot_1_enc_val,  int32 mot_2_enc_val
```

In PWM mode the two float fields are cast to integer PWM values. One message type for
both modes keeps the interface small at the cost of a slightly odd field name.

---

## Build and run

Requires ROS 2 (Humble or newer), Gazebo, and `pyserial`.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Pratyush150/ros2-diffdrive-robot.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Simulation

```bash
ros2 launch my_bot launch_sim.launch.py
```

Brings up the robot in Gazebo with LiDAR and camera. Drive it with any `cmd_vel`
publisher, for example `teleop_twist_keyboard`.

### Description only, in RViz

```bash
ros2 launch my_bot rsp.launch.py
rviz2   # add a RobotModel display, set Fixed Frame to base_link
```

### Real hardware

```bash
ros2 run serial_motor_demo driver \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=57600 \
  -p encoder_cpr:=<your counts per rev> \
  -p loop_rate:=30

ros2 run serial_motor_demo gui   # Tkinter control panel
```

The microcontroller must be running firmware that speaks the expected serial protocol
(the `ros_arduino_bridge` command set). Start in PWM mode and confirm both wheels turn
in the direction you expect before enabling closed-loop control — a swapped encoder sign
in closed loop produces runaway, not a wobble.

---

## File map

```
src/
├── my_bot/
│   ├── description/
│   │   ├── robot.urdf.xacro        top-level description, includes the rest
│   │   ├── robot_core.xacro        chassis, wheels, caster, joints
│   │   ├── inertial_macros.xacro   box / cylinder / sphere inertia from mass + geometry
│   │   ├── lidar.xacro             LiDAR link, joint, and Gazebo sensor plugin
│   │   ├── camera.xacro            camera link, joint, and Gazebo sensor plugin
│   │   └── gazebo_control.xacro    differential drive plugin configuration
│   ├── launch/
│   │   ├── rsp.launch.py           robot_state_publisher only
│   │   └── launch_sim.launch.py    rsp + gzserver + gzclient + spawn
│   ├── worlds/                     Gazebo world files
│   └── config/                     parameter files
└── serial_motor_demo/
    ├── serial_motor_demo/
    │   ├── driver.py               serial node: commands out, encoders in
    │   └── gui.py                  Tkinter control and monitoring panel
    └── serial_motor_demo_msgs/
        └── msg/                    MotorCommand, MotorVels, EncoderVals
```

---

## What this is and is not

**It is** a working reference for the full ROS 2 differential-drive path: description,
simulation, serial hardware interface, encoder feedback. Good for learning the stack and
as a starting point for a small robot.

**It is not** a production robot platform. There is no odometry publisher fusing the
encoder data into `/odom`, no ros2_control integration in this package, no watchdog on
the serial link, and no navigation stack. The description dimensions are the ones from
the tutorial build, not measurements of your chassis.

If you need a hardware interface that plugs into `ros2_control` and
`diff_drive_controller` instead of a standalone driver node, see
[ros2-inspection-robot-hw](https://github.com/Pratyush150/ros2-inspection-robot-hw).

---

## Attribution

The `serial_motor_demo` package and the overall workspace structure are adapted from
Josh Newans' [Articulated Robotics](https://articulatedrobotics.xyz/) material and the
associated open-source repositories, used under their original licenses. The Xacro
description follows that build. Modifications here are configuration, launch fixes, and
documentation.

Saying so matters more than the credit does: if you are evaluating my work, this repo
shows that I can build and debug the full stack, not that I invented it.

---

## Related work

Actively developed engineering tools:

| Repo | What it does |
|---|---|
| [px4-mavlink-companion](https://github.com/Pratyush150/px4-mavlink-companion) | MAVLink bridge, stale-telemetry watchdog, offboard control, serial auto-discovery |
| [flight-log-analyzer](https://github.com/Pratyush150/flight-log-analyzer) | PX4 ULog / ArduPilot log analysis producing a ranked findings report |
| [jetson-realtime-detection](https://github.com/Pratyush150/jetson-realtime-detection) | Real-time detection and tracking with per-stage latency profiling |
| [lidar-slam-toolkit](https://github.com/Pratyush150/lidar-slam-toolkit) | LiDAR SLAM configs plus extrinsics, time-sync and drift diagnostics |
| [drone-control-toolkit](https://github.com/Pratyush150/drone-control-toolkit) | PID with anti-windup, cascaded loops, LQR, EKF and complementary estimators |
| [ros2-drone-bringup](https://github.com/Pratyush150/ros2-drone-bringup) | ROS 2 bringup for a PX4 aircraft: geodesy, missions, geofence, SITL |

---

## License

MIT for the material authored here. Adapted upstream components retain their original
licenses; see the per-package `LICENSE` files.

Copyright (c) 2026 Pratyush Vatsa
