# canopyag

ROS 2 stack for the canopy.ag harvesting robot: a vertical gantry carrying a
SCARA arm, plus a crate that rides the same gantry.

Target: **ROS 2 Humble / Ubuntu 22.04 / Gazebo Fortress**

```
world
└── gantry
    ├── z_carriage  prismatic   0 .. 1.22 m     carriage          CAN ID 1  (real motor)
    │   └── joint_1  revolute   ±1.65 rad       mid_link          CAN ID 3  (real motor)
    │       └── joint_2  continuous             endeffector_link  virtual
    └── z_crate     prismatic   0 .. 1.215 m    crate             virtual
```

## Packages

| package | what's in it |
|---|---|
| `canopyag_description` | URDF/xacro built from `config/robot_parameters.yaml`, meshes, `controllers.yaml`, `hardware.yaml` |
| `canopyag_bringup` | `gazebo.launch.py`, `hardware.launch.py`, the vcan0 launch test |
| `canopyag_hardware` | the ros2_control plugin for the MKS SERVO57D CAN drivers, `mks_sim.py` (simulated drivers), `ghost_state.py` |
| `canopyag_moveit_config` | MoveIt config and the scripted demo path |

## Build

```bash
cd ~/canopy_ag
colcon build --symlink-install
source install/setup.bash
```

## Run

| what | command |
|---|---|
| model in RViz, a slider per joint | `ros2 launch canopyag_description view_robot.launch.py` |
| MoveIt demo path on mock hardware | `ros2 launch canopyag_moveit_config demo.launch.py` (`loop:=true`) |
| Gazebo | `ros2 launch canopyag_bringup gazebo.launch.py` |
| real robot | `ros2 launch canopyag_bringup hardware.launch.py rviz:=true` |
| real robot stack on simulated drivers | `ros2 run canopyag_hardware mks_sim.py --ids 1 3`, then the line above with `can_interface:=vcan0` |

Everything goes through ros2_control, so the controllers and MoveIt are the
same everywhere. Only the hardware plugin changes: Gazebo, mock, or
`canopyag_hardware` on CAN.

## Files you edit

| file | what | after changing it |
|---|---|---|
| `canopyag_description/config/robot_parameters.yaml` | the whole robot model. Written by the importer; your `limit`, `dynamics`, `collision_box`, `note` and `material` survive a re-import | `colcon test` |
| `canopyag_description/config/hardware.yaml` | joint ↔ motor: `mode` (`can` / `virtual`), CAN ID, gearing, direction, speed cap. Values to change are marked `# <-- EDIT` | restart `hardware.launch.py` |
| `canopyag_description/config/controllers.yaml` | which joints each controller drives | `colcon test` |
| `canopyag_moveit_config/config/*` and `scripts/demo_path.py` | MoveIt groups, limits, demo waypoints | `colcon test` |

## Re-importing the CAD

Run the importer on the `urdf/` file **inside** the export folder, so it
finds `meshes/` next to it:

```bash
python3 src/canopyag_description/scripts/import_solidworks_urdf.py urdf_v5/urdf/urdf_v5.urdf
colcon test
```

If a joint was added, removed or renamed, update `controllers.yaml`,
`hardware.yaml`, the SRDF, `joint_limits.yaml`, `moveit_controllers.yaml`
and `demo_path.py`. The tests fail on any file that names a joint that no
longer exists. When a part changes shape, update its `collision_box`: MoveIt
and Gazebo collide with the boxes, not the meshes.

## The real robot

**Zero at startup.** There is no homing yet. When the hardware starts, each
joint's current position becomes 0 (URDF home). So switch the drivers on
with the robot at home. After a driver power cycle, move it home and restart
the launch.

**Real and virtual joints.** In `hardware.yaml`, `mode: can` is a real
motor and `mode: virtual` just copies command to state. The full arm
controller and MoveIt keep working while only some axes have motors.

**Ghost in RViz.** With `rviz:=true` you see the measured robot (solid) and
the commanded robot (see-through). If the two overlap, the hardware tracks.
A ghost running ahead means the axis lags (`max_motor_rpm` too low). Moving
the other way means `reversed` is wrong. Going too far or not far enough
means `motor_revs_per_unit` is wrong.

**Moving it.** Both arm controllers start inactive. Check that the robot is
at home, then activate **one** of them:

```bash
ros2 control switch_controllers --activate forward_position_controller
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.05, 0.0]}"          # z_carriage, joint_1, joint_2
```

`hw_log_level:=debug` logs every command in motor counts and joint units.

**Faults.** A motor that stops replying, a stall, a rejected move or a CAN
bus-off makes the plugin send an emergency stop (F7) to every motor and stop.
The log names the joint and the reason. Restart the launch to recover.
Stall protection stays latched until you clear it, after checking the mechanics:

```bash
cd ~/nema_test && python3 -c "from mks_servo import *; b=CanBus(); MKSServo(b, 3).release_protection()"
```

**CAN bus.** `~/nema_test/setup_can.sh` brings up `can0` at 500 kbit/s.
With the power off, CAN_H to CAN_L must measure 60 Ω (a 120 Ω terminator at
each end of the chain). Every driver needs `Mode = SR_vFOC`,
`CanRate = 500K`, a unique `CanID`, and `Respon` / `Active` on. The adapter
cannot recover from bus-off by itself:
`sudo ip link set can0 down && sudo ip link set can0 up`.

### Joint table

| joint | CAN ID | transmission | `motor_revs_per_unit` | direction |
|---|---|---|---|---|
| `z_carriage` | 1 | 5:1 + GT2 belt, 20T pulley (8 mm per motor rev) | 125 rev/m | to check |
| `joint_1` | 3 | 16:1 | 2.546 rev/rad | to check |
| `joint_2` | TODO | TODO | TODO | |
| `z_crate` | 2 | TODO | TODO | |

## Testing

`colcon test` runs everything. The CAN tests need a virtual bus, once per boot:

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
colcon test && colcon test-result --verbose
```

That includes one full launch of the real-robot stack against `mks_sim.py`
per fault: motor dropped, stall, duplicate CAN ID, slow replies, a driver
that ignores or rejects a new target mid-move. Without vcan0 those are
skipped. `mks_sim.py --help` lists the fault flags if you want to try them by hand.

## Next

1. **Check position moves on a real driver.** Run
   `python3 ~/nema_test/pos_diag.py --motor motor3` and a small F5 test.
   The plugin streams F5 (absolute position) commands, so check:
   - Does 0x31 read in the same coordinates F5 uses?
   - Does a new F5 sent during a move retarget smoothly? If the driver
     **rejects** it, streaming does not work and the plan changes to speed
     mode with a position loop in the plugin.
2. **joint_1 alone:** set `z_carriage` to `mode: virtual`. Try 0.05 rad
   steps, then ±90°, and check the direction and the 16:1 scaling against a
   mark on the link.
3. **z_carriage:** small steps, then check the 125 rev/m with a tape measure.
4. Fill in the TODO rows (joint_2, z_crate) and switch them to `mode: can` one at a time.
5. MoveIt on the real robot: `demo.launch.py` still runs on mock hardware
   only and needs a hardware option.
6. Real joint limits, efforts and velocities. The current ones are guesses.
7. Homing against the limit switches, to replace zero-at-startup.
