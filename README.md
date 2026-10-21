# canopyag

ROS 2 stack for the canopy.ag harvesting robot: a vertical gantry carrying a
SCARA arm, plus a crate that rides the same gantry.

Target: **ROS 2 Humble / Ubuntu 22.04 / Gazebo Fortress**

```
world
└── gantry
    ├── z_carriage  prismatic   0 .. 0.446 m    carriage          CAN ID 1  real motor
    │   └── joint_1  revolute   ±2.094 rad      mid_link          CAN ID 3  real motor
    │       └── joint_2  revolute   ±π rad          endeffector_link  CAN ID 4  real motor
    │           └── joint_3  continuous         end_effector      -         virtual (gripper rotation)
    └── z_crate     prismatic   0 .. 0.4804 m   crate             CAN ID 2  real motor
```

"Virtual" joints have no motor yet: the plugin copies their command to their
state, so the controllers and canopyag_motion work as if they were there. Which
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
| **to a world point, picked in a window** | `ros2 control switch_controllers --activate arm_controller` | `ros2 launch canopyag_motion motion.launch.py panel:=true` (see **Move to a point**) |
| to a world point, from the shell | `ros2 control switch_controllers --activate arm_controller` | `ros2 launch canopyag_motion motion.launch.py`, then in terminal 3 `ros2 run canopyag_motion move_to.py 0.45 0.55 0.5 --speed 0.3` |
| the demo path | `ros2 control switch_controllers --activate arm_controller` | `ros2 launch canopyag_motion motion.launch.py demo:=true speed:=0.3` (`side:=left`, `loop:=true`) |
| sliders | `ros2 control switch_controllers --activate arm_controller` | `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`, pick `arm_controller` |
| one step at a time | `ros2 control switch_controllers --activate forward_position_controller` | `ros2 topic pub --once -w 2 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.05, 0.0]}"` |

- The sliders need `sudo apt install ros-humble-rqt-joint-trajectory-controller` once.
- The step command is `[z_carriage m, joint_1 rad, joint_2 rad]`, absolute.
  `forward_position_controller` leaves `joint_3` out; `arm_controller` drives it.
  Keep `-w 2`: without it the message can go out before the controller is
  connected and get lost.
- To change methods:
  `ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller`.
  The two controllers drive the same joints and cannot both be active.

**5. Stop:** Ctrl+C in terminal 1. The motors ramp down and hold.

### Move to a point

`canopyag_motion` moves the **tool point** (`motion.yaml`: on `joint_3`'s
axis, half way down the gripper) to a point in the `world` frame, as fast
as the joint limits allow, every joint at the same time. It works out the
joints itself (both elbow solutions, the fastest wins), keeps link 2 out of
the carriage and gantry collision boxes, and sends the crate along under the
carriage. `joint_3` turns the gripper about the tool point: a point goal
leaves it where it is, a joint goal with a fourth value turns it. Four ways
in, all through `motion_server.py`:

| from | how |
|---|---|
| **the goal panel** | `goal_panel.py`: a slider (and a number box) per axis, Go, Stop, see below. `panel:=true` on `motion.launch.py` or `demo.launch.py` opens it; on its own `ros2 run canopyag_motion goal_panel.py` |
| a shell | `ros2 run canopyag_motion move_to.py X Y Z [--speed 0.3]`; joints instead: `move_to.py --joints Z_CARRIAGE JOINT_1_DEG JOINT_2_DEG [JOINT_3_DEG]`. Waits until there; Ctrl+C brakes to a stop |
| code | the action `move_to` (`canopyag_interfaces/action/MoveTo`): `point` or `joints`, `speed`; result `success`, `message`, `joints`, `duration` |
| a click | topic `goal_point` (`geometry_msgs/PointStamped`, any frame tf knows). RViz's **Publish Point** tool sends there in `motion.rviz`; it needs something under the mouse to click on |

The tool reaches 0.11–0.62 m around joint_1, which sits 0.24 m in front of
the gantry origin; z 0.24–0.68 m. A new goal while the robot moves takes over from
where it is, at its speed. An unreachable or colliding goal is refused with
the reason, and the running motion goes on. RViz shows the goal (orange) and
the path the tool point will take (blue) on `goal_markers`.

**The goal panel.** The sliders cover the box around everything the tool
can reach (now x −0.11…0.87, y −0.63…0.63, z 0.24…0.68 m, worked out from the
model at start-up), so not every point in it is reachable. It starts at
where the tool is. While you pick, the point is planned from where the robot
is now, exactly as `motion_server.py` will: the panel says *reachable* with
the joints and the time, or why not (too far, joint_1 limit, collision),
and RViz shows the point on `goal_preview` (green with the path to it, or
red). Nothing moves until you press **Go**.

| control | does |
|---|---|
| speed | 5–100 % of the `motion.yaml` limits for every Go (starts at 30 %) |
| **Go** | sends the point; the panel shows the time left, then `arrived` or the reason it failed |
| **Stop** | cancels: the robot brakes to a stop |
| **Random** | a reachable point anywhere in the box, to try long paths |
| **Rest** | the arm to the demo's rest pose (carriage 5 cm up, arm straight) |
| Send on release | every slider release or typed value is a Go, so new goals take over from the running motion |

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
| `CAN cycle takes … ms` warning | too many motors for `can_rate_hz`: the requests no longer fit in one cycle | lower `can_rate_hz` in `hardware.yaml` to the number it suggests |

On every fault the plugin sends an emergency stop (F7) to all motors and
stops; the only way back is restarting terminal 1.

Clearing a latched stall (replace 3 with the CAN ID):

```bash
cd ~/nema_test && python3 -c "from mks_servo import *; b=CanBus(); MKSServo(b, 3).release_protection()"
```

## How do I…

Restart terminal 1 after editing `hardware.yaml`, and `motion_server.py`
(and the goal panel) after editing `motion.yaml` (no rebuild needed for
either). Run `colcon build --symlink-install` after adding files.

| I want to | edit | how |
|---|---|---|
| **flip a joint's direction** | `canopyag_description/config/hardware.yaml` | that joint's `reversed: true` / `false` |
| **fix how far a joint moves** | `hardware.yaml` | `motor_revs_per_unit`: gear ratio G on a revolute joint is `G / 2π` (9.6:1 = 1.528, 12:1 = 1.910, 16:1 = 2.546); belt or lead screw with L metres per motor rev is `1 / L` |
| **add a motor / take one out** | `hardware.yaml` | `mode: can` (real) or `mode: virtual`, and the right `can_id` |
| **make the demo faster** | launch argument | `speed:=` up to `1.0` |
| **make it faster than `speed:=1.0`** | `canopyag_motion/config/motion.yaml` (`limits`) | then check the two motor limits below |
| …so the motor keeps up | `hardware.yaml` `max_motor_rpm` | at least `max_velocity × motor_revs_per_unit × 72` (9.6:1: 3.5 rad/s → 385 rpm), at most 3000 |
| …and accelerates in time | `hardware.yaml` `acceleration` | the driver ramps at `20000 / (256 − acceleration)` motor rpm/s. At 9.6:1, 230 ≈ 8 rad/s², 240 ≈ 14 rad/s². Keep `max_acceleration` below that |
| change the demo fruit | `canopyag_motion/scripts/demo.py`: `FRUIT` | world points of the tool, right side; `side:=left` mirrors them in y |
| pick on the other side | launch argument | `side:=left` (default `right`, seen from the front of the gantry) |
| **move faster** (`canopyag_motion`) | `canopyag_motion/config/motion.yaml` `limits` | velocity and acceleration per joint; keep them under the motor limits below |
| set where the tool point is | `motion.yaml` `tool` | a point in `endeffector_link`'s frame; keep it on `joint_3`'s axis |
| change how close the arm may come to itself | `motion.yaml` `collision.margin` | m added around every collision box (now 2 cm). Folded under link 1 with `joint_3` at 0, the gripper points back at the gantry and only fits with joint_1 near straight ahead; with the gripper turned round (`joint_3` = π) the fold is clear anywhere |
| see what the plugin sends | terminal 1 | add `hw_log_level:=debug`: every F5 with target, lag and rpm, and every 10 s the CAN rates and late replies |
| import a new CAD export | `canopyag_description/scripts/import_solidworks_urdf.py` | see **Re-importing the CAD** |

Current motors (`hardware.yaml`):

| joint | CAN ID | transmission | `motor_revs_per_unit` | `reversed` | checked |
|---|---|---|---|---|---|
| `joint_1` | 3 | 9.6:1 | 1.528 rev/rad | false | direction; scaling to re-check after the gearbox change |
| `joint_2` | 4 | 12:1 | 1.910 rev/rad | true | direction; **scaling still to check** |
| `z_carriage` | 1 | 5:1 + GT2 belt, 20T pulley (8 mm per motor rev) | 125 rev/m | false | not yet |
| `z_crate` | 2 | TODO | 125 rev/m (placeholder) | false | not yet |
| `joint_3` | - | no motor yet (`mode: virtual`) | - | - | - |

## Test without the robot

| what | command |
|---|---|
| model in RViz, a slider per joint (no controllers) | `ros2 launch canopyag_description view_robot.launch.py` |
| the demo on mock hardware | `ros2 launch canopyag_motion demo.launch.py` (`side:=left`, `loop:=true`, `speed:=`) |
| **pick goals in a window on mock hardware** | `ros2 launch canopyag_motion demo.launch.py demo:=false panel:=true` |
| send goals on mock hardware | `ros2 launch canopyag_motion demo.launch.py demo:=false`, then `move_to.py` / Publish Point as in **Move to a point** |
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
ros2 run canopyag_hardware mks_sim.py --ids 1 2 3 4
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
python3 src/canopyag_description/scripts/import_solidworks_urdf.py urdf_new/urdf/urdf_new.urdf
colcon build --symlink-install && colcon test
```

It rewrites `robot_parameters.yaml` but keeps what you set by hand
(`limit`, `dynamics`, `collision_box`, `note`, `material`). If a joint was
added, removed or renamed, update `controllers.yaml`, `hardware.yaml` and
`canopyag_motion/config/motion.yaml`; the tests fail on any file naming a
joint that no longer exists. When a part changes shape, update its
`collision_box`: canopyag_motion collides with the boxes, not the meshes.

The export writes a prismatic joint's limits as heights above the gantry base
(lower = the joint's own `xyz` z, because it is exported at the bottom). Since
`limit` is kept, a new travel has to be set by hand: `upper − lower`, from 0.

## How it works

| package | what's in it |
|---|---|
| `canopyag_description` | URDF/xacro built from `robot_parameters.yaml`, meshes, `controllers.yaml`, `hardware.yaml` |
| `canopyag_bringup` | `hardware.launch.py`, `mock.launch.py`, `gazebo.launch.py`, the vcan0 launch test |
| `canopyag_hardware` | the ros2_control plugin for the MKS SERVO57D drivers, `mks_sim.py`, `ghost_state.py` |
| `canopyag_motion` | goals in world coordinates: `motion_server.py`, `goal_panel.py`, `move_to.py`, `demo.py`; `motion.yaml` |
| `canopyag_interfaces` | the `MoveTo` action |

Everything goes through ros2_control, so the controllers and canopyag_motion
are the same on mock hardware, in Gazebo and on the robot. Only the hardware
plugin changes.

**canopyag_motion** has no sampling-based planner; the robot is simple
enough not to need one. The inverse kinematics are the closed-form two-link
solution plus z from the carriage. Every joint gets its own time-optimal
profile (full acceleration, top speed, full braking) and they all start
together. The path is checked against the collision boxes from
`robot_parameters.yaml` every time any point of the arm has moved 1 cm. If it
hits, one revolute joint starts a little later (typically joint_1 waits a
tenth of a second for link 2 to unfold), the shortest wait that clears wins.
If no wait clears it, the arm stops once on the way at an intermediate
joint_1. Planning takes 10–80 ms. The trajectory goes straight to
`arm_controller` and `crate_controller`, both stamped to start at the same
instant.

At the ±120° joint_1 limit the `mid_link` box touches the gantry box, so the
arm stops just short of it. `joint_3` moves at the same time as the rest and
the gripper's box is checked like the others. Links on the same rail (all of
the arm rides the carriage) never move up or down relative to each other, so
between them the margin is left out vertically: the gripper may pass 2.5 cm
under link 1.

The plugin runs its own 100 Hz CAN loop. For each real motor, every cycle,
it sends an absolute position move (F5) that aims slightly ahead of the
trajectory (by the driver's braking distance plus the travel during two
measured CAN cycles), at the trajectory's speed plus a catch-up term, and
reads the encoder back (0x31). It talks to one driver at a time and waits for
each reply, because these drivers produce CAN errors when two of them answer
at once. So the cycle gets longer with every motor: with four it no longer
fits in 10 ms, and the plugin warns (`CAN cycle takes … ms`).

CAN wiring: CAN_H to CAN_L measures 60 Ω with the power off, with a 120 Ω
terminator at each physical end of the chain (adapter and last driver). Every
driver: `Mode = SR_vFOC`, `CanRate = 500K`, a unique `CanID`, `Respon` and
`Active` on.

## Next

1. **joint_1 / joint_2 scaling:** command 1.571 rad (90°) and check it
   against a mark (new 9.6:1 and 12:1 gearboxes).
2. **z_carriage:** small steps, then check 125 rev/m with a tape measure,
   and the direction.
3. **z_crate:** fill in its transmission, then the same.
4. Real joint limits, efforts and velocities. The current ones are guesses.
5. Homing against the limit switches, to replace zero-at-startup.
