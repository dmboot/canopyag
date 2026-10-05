# canopyag

ROS 2 stack for the canopy.ag harvesting robot: a vertical gantry carrying a
SCARA arm, plus a crate that rides the same gantry.

Target: **ROS 2 Humble / Ubuntu 22.04 / Gazebo Fortress**

```
world
└── gantry
    ├── z_carriage  prismatic   0 .. 1.22 m     carriage          CAN ID 1  virtual for now
    │   └── joint_1  revolute   ±1.65 rad       mid_link          CAN ID 3  real motor
    │       └── joint_2  continuous             endeffector_link  CAN ID 4  real motor
    └── z_crate     prismatic   0 .. 1.215 m    crate             CAN ID 2  virtual for now
```

"Virtual" joints have no motor yet: the plugin copies their command to their
state, so MoveIt and the controllers work as if they were there. Which
joints are real is set in `hardware.yaml` (see **How do I…**).

## Build

```bash
cd ~/canopy_ag
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs `source ~/canopy_ag/install/setup.bash`.

## Drive the real robot

**1. Robot at home, then power the drivers on.** There is no homing yet:
when the launch starts, every joint's current position becomes 0. After a
driver power cycle, put the robot back at home and restart the launch.

**2. Bring up the CAN bus** (once per boot or USB replug):

```bash
~/nema_test/setup_can.sh
python3 ~/nema_test/status.py          # every motor must reply
```

**3. Terminal 1: start the robot** and leave it running:

```bash
ros2 launch canopyag_bringup hardware.launch.py rviz:=true
```

Wait for `Successful 'activate' of hardware 'canopyag'`. RViz shows the
measured robot (solid) and the commanded robot (see-through "ghost").

**4. Terminal 2: activate a controller.** They all start inactive on
purpose. Pick one way of moving the robot:

| how | activate | then |
|---|---|---|
| MoveIt demo path | `ros2 control switch_controllers --activate arm_controller` | `ros2 launch canopyag_moveit_config moveit.launch.py demo:=true speed:=0.3` |
| MoveIt, plan by hand in RViz | `ros2 control switch_controllers --activate arm_controller` | `ros2 launch canopyag_moveit_config moveit.launch.py` (start step 3 without `rviz:=true`) |
| sliders | `ros2 control switch_controllers --activate arm_controller` | `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`, pick `arm_controller` |
| one step at a time | `ros2 control switch_controllers --activate forward_position_controller` | `ros2 topic pub --once -w 2 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.05, 0.0]}"` |

- The sliders need `sudo apt install ros-humble-rqt-joint-trajectory-controller` once.
- The step command is `[z_carriage m, joint_1 rad, joint_2 rad]`, absolute.
  Keep `-w 2`: without it the message can go out before the controller is
  connected and get lost.
- To change methods:
  `ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller`.
  The two controllers drive the same joints and cannot both be active.

**5. Stop:** Ctrl+C in terminal 1. The motors ramp down and hold.

### When something goes wrong

| you see | what happened | do |
|---|---|---|
| `FAULT: joint '…' (CAN ID n) has not answered` | a driver stopped replying (power, wiring, CAN) | fix it, robot to home, restart terminal 1 |
| `FAULT: … STALL PROTECTION tripped` | the motor was blocked | check the mechanics, clear the stall (below), restart |
| `FAULT: … F5 REJECTED` | driver refused a move (not enabled, wrong `Mode`) | check the driver menu, restart |
| `does not reply to 0x31` at startup | a driver is off or has the wrong CanID / CanRate | `python3 ~/nema_test/status.py` |
| `answered 2 times` at startup | two drivers share a CanID | give each driver a unique CanID |
| `BUS-OFF` | CAN bus broke down | check wiring, then `sudo ip link set can0 down && sudo ip link set can0 up` |
| goal ABORTED, `State tolerances failed` | the motor fell behind the trajectory | slower `speed:=`, or see **make it faster** below |
| ghost and robot move apart in RViz | motor not following: lag, direction or scaling | see the `hardware.yaml` rows below |

On every fault the plugin sends an emergency stop (F7) to all motors and
stops; the only way back is restarting terminal 1.

Clearing a latched stall (replace 3 with the CAN ID):

```bash
cd ~/nema_test && python3 -c "from mks_servo import *; b=CanBus(); MKSServo(b, 3).release_protection()"
```

## How do I…

Restart terminal 1 after editing `hardware.yaml` (no rebuild needed). Run
`colcon build --symlink-install` after editing anything in
`canopyag_moveit_config`.

| I want to | edit | how |
|---|---|---|
| **flip a joint's direction** | `canopyag_description/config/hardware.yaml` | that joint's `reversed: true` / `false` |
| **fix how far a joint moves** | `hardware.yaml` | `motor_revs_per_unit`: gear ratio G on a revolute joint is `G / 2π` (16:1 = 2.546); belt or lead screw with L metres per motor rev is `1 / L` |
| **add a motor / take one out** | `hardware.yaml` | `mode: can` (real) or `mode: virtual`, and the right `can_id` |
| **make the demo faster** | launch argument | `speed:=` up to `1.0` |
| **make it faster than `speed:=1.0`** | `canopyag_moveit_config/config/joint_limits.yaml` (`max_velocity`, `max_acceleration`) **and** `canopyag_description/config/robot_parameters.yaml` (`limit.velocity`) | then check the two motor limits below |
| …so the motor keeps up | `hardware.yaml` `max_motor_rpm` | at least `max_velocity × motor_revs_per_unit × 72` (16:1: 1.5 rad/s → 275 rpm, 3 rad/s → 550 rpm), at most 3000 |
| …and accelerates in time | `hardware.yaml` `acceleration` | the driver ramps at `20000 / (256 − acceleration)` motor rpm/s. At 16:1, 230 ≈ 5 rad/s², 245 ≈ 12 rad/s². Keep `max_acceleration` below that |
| change the demo waypoints | `canopyag_moveit_config/scripts/demo_path.py`, `WAYPOINTS` | `(name, (z_carriage, joint_1, joint_2), scaling)` |
| see what the plugin sends | terminal 1 | add `hw_log_level:=debug`: every F5 with target, lag and rpm, and every 10 s the CAN rates and late replies |
| import a new CAD export | `canopyag_description/scripts/import_solidworks_urdf.py` | see **Re-importing the CAD** |

Current motors (`hardware.yaml`):

| joint | CAN ID | transmission | `motor_revs_per_unit` | `reversed` | checked |
|---|---|---|---|---|---|
| `joint_1` | 3 | 16:1 | 2.546 rev/rad | false | direction and scaling |
| `joint_2` | 4 | assumed 16:1 | 2.546 rev/rad | true | direction; **scaling still to check** |
| `z_carriage` | 1 | 5:1 + GT2 belt, 20T pulley (8 mm per motor rev) | 125 rev/m | false | not yet (virtual) |
| `z_crate` | 2 | TODO | TODO | | not yet (virtual) |

## Test without the robot

| what | command |
|---|---|
| model in RViz, a slider per joint (no controllers) | `ros2 launch canopyag_description view_robot.launch.py` |
| MoveIt demo on mock hardware | `ros2 launch canopyag_moveit_config demo.launch.py` (`loop:=true`, `speed:=`) |
| Gazebo | `ros2 launch canopyag_bringup gazebo.launch.py` |
| the real-robot stack on simulated drivers | see below |
| all tests | `colcon test && colcon test-result --verbose` |

**Simulated drivers.** `mks_sim.py` pretends to be the MKS drivers on a
virtual CAN bus. Create the bus once per boot:

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
```

Then, **in every terminal**, `export ROS_DOMAIN_ID=77` first. That keeps the
simulation on its own ROS network, so its commands can never reach the real
robot if that is running too.

```bash
ros2 run canopyag_hardware mks_sim.py --ids 3 4
ros2 launch canopyag_bringup hardware.launch.py can_interface:=vcan0 rviz:=true
```

and everything in **Drive the real robot** from step 4 works the same.
`mks_sim.py --help` lists fault flags (motor drops out, stall, duplicate
ID, slow replies).

`colcon test` runs the plugin against `mks_sim.py` once per fault, on its
own ROS domain; without vcan0 those cases are skipped.

## Re-importing the CAD

Run the importer on the `urdf/` file **inside** the export folder, so it
finds `meshes/` next to it:

```bash
python3 src/canopyag_description/scripts/import_solidworks_urdf.py urdf_v5/urdf/urdf_v5.urdf
colcon build --symlink-install && colcon test
```

It rewrites `robot_parameters.yaml` but keeps what you set by hand
(`limit`, `dynamics`, `collision_box`, `note`, `material`). If a joint was
added, removed or renamed, update `controllers.yaml`, `hardware.yaml`, the
SRDF, `joint_limits.yaml`, `moveit_controllers.yaml` and `demo_path.py`; the
tests fail on any file naming a joint that no longer exists. When a part
changes shape, update its `collision_box`: MoveIt collides with the boxes,
not the meshes.

## How it works

| package | what's in it |
|---|---|
| `canopyag_description` | URDF/xacro built from `robot_parameters.yaml`, meshes, `controllers.yaml`, `hardware.yaml` |
| `canopyag_bringup` | `hardware.launch.py`, `gazebo.launch.py`, the vcan0 launch test |
| `canopyag_hardware` | the ros2_control plugin for the MKS SERVO57D drivers, `mks_sim.py`, `ghost_state.py` |
| `canopyag_moveit_config` | MoveIt config, `moveit.launch.py` (MoveIt only, attaches to any running controllers), `demo.launch.py` (mock hardware + MoveIt), `demo_path.py` |

Everything goes through ros2_control, so the controllers and MoveIt are the
same on mock hardware, in Gazebo and on the robot. Only the hardware plugin
changes.

The plugin runs its own 100 Hz CAN loop. For each real motor, every cycle,
it sends an absolute position move (F5) that aims slightly ahead of the
trajectory (by the driver's braking distance), at the trajectory's speed plus
a catch-up term, and reads the encoder back (0x31). It talks to one driver at a
time and waits for each reply, because these drivers produce CAN errors when
two of them answer at once.

CAN wiring: CAN_H to CAN_L measures 60 Ω with the power off, with a 120 Ω
terminator at each physical end of the chain (adapter and last driver). Every
driver: `Mode = SR_vFOC`, `CanRate = 500K`, a unique `CanID`, `Respon` and
`Active` on.

## Next

1. **joint_2 scaling:** command 1.571 rad (90°) and check it against a mark.
   If it is off, its gear ratio is not 16:1; fix `motor_revs_per_unit`.
2. **z_carriage:** set `mode: can`, small steps, then check 125 rev/m with a
   tape measure, and the direction.
3. **z_crate:** fill in its transmission, then the same.
4. Real joint limits, efforts and velocities. The current ones are guesses.
5. Homing against the limit switches, to replace zero-at-startup.
