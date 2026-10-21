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
    │           └── joint_3  revolute   ±4.712 rad (±270°)  end_effector  CAN ID 5  real motor (wrist yaw)
    └── z_crate     prismatic   0 .. 0.4804 m   crate             CAN ID 2  real motor
```

`joint_3` is the wrist yaw: a NEMA17 on an MKS SERVO42D, 5:1, on the same CAN
bus. It turns the gripper about a vertical axis at the end of link 2. The
gripper points along `end_effector`'s +x; its pitch and the gripper itself
are not in the URDF and will be driven on their own. Its ±270° are set by
hand in `robot_parameters.yaml` for the cables (to measure, see **Next**).

A joint can also be `mode: virtual` in `hardware.yaml` (no motor yet): the
plugin then copies its command to its state, so the controllers and
canopyag_motion work as if it were there. Today every joint is real.

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
| one step at a time | `ros2 control switch_controllers --activate forward_position_controller` | `ros2 topic pub --once -w 2 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.05, 0.0, 0.0]}"` |

- The sliders need `sudo apt install ros-humble-rqt-joint-trajectory-controller` once.
- The step command is `[z_carriage m, joint_1 rad, joint_2 rad, joint_3 rad]`,
  absolute. Stepping joint_1 or joint_2 alone turns the gripper with them:
  only canopyag_motion keeps it pointing to the side.
  Keep `-w 2`: without it the message can go out before the controller is
  connected and get lost.
- To change methods:
  `ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller`.
  The two controllers drive the same joints and cannot both be active.

**5. Stop:** Ctrl+C in terminal 1. The motors ramp down and hold.

### Move to a point

`canopyag_motion` moves the **tool point** (`motion.yaml`: the gripper tip,
0.21 m out from `joint_3`'s axis) to a point in the `world` frame, as fast
as the joint limits allow, every joint at the same time. **The gripper
always points along world +y or -y** (the *heading*): by default to the
side of joint_1 the goal is on, or as the goal says. `joint_3` is not moved
on its own: it turns with joint_1 and joint_2 so that the gripper keeps its
heading the whole way, and turns it over when the heading changes (the
other side, or a start pointing anywhere else, like home). The planner works
out the joints itself (both elbows, either turn of `joint_3`, the fastest
wins) and keeps link 2 and the gripper out of the carriage, gantry and crate
collision boxes. **The crate stays where it is** (at home: 0) unless a goal
sends it somewhere (the panel's crate slider, `MoveTo.crate`); it is only
moved down by itself when the carriage would come closer than 2 cm to it
(`motion.yaml` `crate`). Four ways in, all through `motion_server.py`:

| from | how |
|---|---|
| **the goal panel** | `goal_panel.py`: a slider (and a number box) per axis, Go, Stop, see below. `panel:=true` on `motion.launch.py` or `demo.launch.py` opens it; on its own `ros2 run canopyag_motion goal_panel.py` |
| a shell | `ros2 run canopyag_motion move_to.py X Y Z [--speed 0.3] [--heading=-y]`; joints instead: `move_to.py --joints Z_CARRIAGE JOINT_1_DEG JOINT_2_DEG [JOINT_3_DEG]`. Waits until there; Ctrl+C brakes to a stop |
| code | the action `move_to` (`canopyag_interfaces/action/MoveTo`): `point` or `joints`, `speed`, `heading` (`HEADING_DEFAULT` = 0, `HEADING_POS_Y` = 1, `HEADING_NEG_Y` = -1), `crate` (`[]` or `[z_crate]`), `crate_only` (only the crate moves, the arm carries on); result `success`, `message`, `joints`, `duration` |
| a click | topic `goal_point` (`geometry_msgs/PointStamped`, any frame tf knows). RViz's **Publish Point** tool sends there in `motion.rviz`; it needs something under the mouse to click on |

The wrist axis (`joint_3`) reaches 0.11–0.62 m around joint_1, which sits
0.24 m in front of the gantry origin; the tip is 0.21 m further, to +y or -y;
z 0.25–0.70 m. A joint goal with three values gets `joint_3` from the heading
(the side the wrist ends up on), a fourth value sets `joint_3` itself. A new
goal while the robot moves takes over from where it is, at its speed. An
unreachable or colliding goal is refused with the reason (also "joint_3
does not turn that far"), and the running motion goes on. RViz shows the
goal (orange), the gripper at the goal (orange arrow, from `joint_3`'s axis
to the tip) and the path the tip will take (blue) on `goal_markers`.

**The goal panel.** The sliders cover the box around everything the tool
can reach (now x −0.11…0.87, y −0.84…0.84, z 0.25…0.70 m, worked out from the
model at start-up), so not every point in it is reachable; *gripper to*
picks the heading (auto, +y, -y). It starts at where the tool is. While you
pick, the point is planned from where the robot is now, exactly as
`motion_server.py` will: the panel says *reachable* with
the joints and the time, or why not (too far, joint_1 limit, collision),
and RViz shows the point on `goal_preview` (green with the path to it, or
red). Nothing moves until you press **Go**.

| control | does |
|---|---|
| speed | 5–100 % of the `motion.yaml` limits for every Go (starts at 30 %) |
| **Go** | sends the point; the panel shows the time left, then `arrived` or the reason it failed |
| **Stop** | cancels: the robot brakes to a stop |
| **Random** | a reachable point anywhere in the box, to try long paths |
| **Rest** | the arm to the demo's rest pose (carriage 5 cm up, arm straight, gripper to the side) |
| crate | not part of the point: on release (or a typed value) only the crate goes there, at the speed above, and the arm carries on. Refused if it would come within 2 cm of the carriage's goal; at home the carriage is right above it, so raise the carriage first. It shows where the crate is once it stands still |
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
| **fix how far a joint moves** | `hardware.yaml` | `motor_revs_per_unit`: gear ratio G on a revolute joint is `G / 2π` (5:1 = 0.796, 9.6:1 = 1.528, 12:1 = 1.910, 16:1 = 2.546); belt or lead screw with L metres per motor rev is `1 / L` |
| **add a motor / take one out** | `hardware.yaml` | `mode: can` (real) or `mode: virtual`, and the right `can_id` |
| **make the demo faster** | launch argument | `speed:=` up to `1.0` |
| **make it faster than `speed:=1.0`** | `canopyag_motion/config/motion.yaml` (`limits`) | then check the two motor limits below |
| …so the motor keeps up | `hardware.yaml` `max_motor_rpm` | at least `max_velocity × motor_revs_per_unit × 72` (9.6:1: 3.5 rad/s → 385 rpm), at most 3000 |
| …and accelerates in time | `hardware.yaml` `acceleration` | the driver ramps at `20000 / (256 − acceleration)` motor rpm/s. At 9.6:1, 230 ≈ 8 rad/s², 240 ≈ 14 rad/s². Keep `max_acceleration` below that |
| change the demo fruit | `canopyag_motion/scripts/demo.py`: `FRUIT` | world points of the gripper tip, right side; `side:=left` mirrors them in y |
| pick on the other side | launch argument | `side:=left` (default `right`, seen from the front of the gantry): the gripper points to -y instead of +y |
| **choose where the gripper points** | per goal, or `motion.yaml` `heading.default` | `move_to.py ... --heading=+y` / `--heading=-y`, `MoveTo.heading`, the panel's *gripper to*; the default `auto` is the side of joint_1 the goal is on |
| show the pitch wiggle and grip in the demo | launch argument | `dexterity:=true` (default on in `demo.launch.py`, off in `motion.launch.py`). Needs a `gripper_controller` (FollowJointTrajectory, joints `wrist_pitch`, `gripper`: `demo.py` parameters `gripper_controller`, `gripper_joints`, `grip_open`, `grip_closed`); without it the demo says so once and goes on |
| **move faster** (`canopyag_motion`) | `canopyag_motion/config/motion.yaml` `limits` | velocity and acceleration per joint; keep them under the motor limits below |
| set where the tool point is | `motion.yaml` `tool` | the gripper tip in `end_effector`'s frame, pitch at 0 (now 0.21 m along +x) |
| **limit how far the wrist turns** | `robot_parameters.yaml` `joints.joint_3.limit` `lower` / `upper` | the cable range from home (gripper along link 2); kept on a re-import. canopyag_motion needs at least ±π; less than ±270° makes some folds turn the gripper a full turn on the way |
| make the wrist faster / slower | `motion.yaml` `limits.joint_3` | also the limit for turning the heading. A move that would turn `joint_3` faster runs slower as a whole (note `slowed to …`) |
| change how close the arm may come to itself | `motion.yaml` `collision.margin` | m added around every collision box (now 2 cm). The gripper hangs below link 1 and the carriage, so with the gripper to the side the fold under link 1 is clear; close to the gantry the gripper box is what hits first |
| move the crate | the goal panel's crate slider, or `MoveTo` with `crate: [z]`, `crate_only: true` | arm goals leave it where it is; `motion.yaml` `crate.follow: true` makes it ride `crate.trail` below the carriage again |
| **fit 5 motors on the bus** | `hardware.yaml` `can_rate_hz` | now 50: every cycle is 2 request/reply turns per motor + 1 (about 1.5 ms each on can0), 11 turns ≈ 16.5 ms. If the plugin warns `CAN cycle takes …`, use the rate it suggests |
| see what the plugin sends | terminal 1 | add `hw_log_level:=debug`: every F5 with target, lag and rpm, and every 10 s the CAN rates and late replies |
| import a new CAD export | `canopyag_description/scripts/import_solidworks_urdf.py` | see **Re-importing the CAD** |

Current motors (`hardware.yaml`):

| joint | CAN ID | transmission | `motor_revs_per_unit` | `reversed` | checked |
|---|---|---|---|---|---|
| `joint_1` | 3 | 9.6:1 | 1.528 rev/rad | false | direction, scaling |
| `joint_2` | 4 | 12:1 | 1.910 rev/rad | true | direction, scaling |
| `z_carriage` | 1 | 5:1 + GT2 belt, 20T pulley (8 mm per motor rev) | 125 rev/m | false | not yet |
| `z_crate` | 2 | TODO | 125 rev/m (placeholder) | false | not yet |
| `joint_3` | 5 | NEMA17 on an MKS SERVO42D, 5:1 | 0.796 rev/rad | false | not yet: direction, scaling, cable range |

## Test without the robot

| what | command |
|---|---|
| model in RViz, a slider per joint (no controllers) | `ros2 launch canopyag_description view_robot.launch.py` |
| the demo on mock hardware | `ros2 launch canopyag_motion demo.launch.py` (`side:=left`, `loop:=true`, `speed:=`, `dexterity:=false`) |
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
ros2 run canopyag_hardware mks_sim.py --ids 1 2 3 4 5
ros2 launch canopyag_bringup hardware.launch.py can_interface:=vcan0 rviz:=true
```

and everything in **Drive the real robot** from step 4 works the same.
`mks_sim.py --help` lists fault flags (motor drops out, stall, duplicate
ID, slow replies).

`colcon test` runs the plugin against `mks_sim.py` once per fault (CAN IDs
1, 3 and 5), on its own ROS domain; without vcan0 those cases are skipped.

If `colcon test` shows no new `canopyag_motion` results: `pytest --version`
fails in your shell (a pytest plugin in `~/.local`), so CMake left the test
out. Build and test with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` set, and
reconfigure once: `colcon build --symlink-install --packages-select
canopyag_motion --cmake-force-configure`.

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
| `canopyag_hardware` | the ros2_control plugin for the MKS SERVO57D / SERVO42D drivers, `mks_sim.py`, `ghost_state.py` |
| `canopyag_motion` | goals in world coordinates: `motion_server.py`, `goal_panel.py`, `move_to.py`, `demo.py`; `motion.yaml` |
| `canopyag_interfaces` | the `MoveTo` action |

Everything goes through ros2_control, so the controllers and canopyag_motion
are the same on mock hardware, in Gazebo and on the robot. Only the hardware
plugin changes.

**canopyag_motion** has no sampling-based planner; the robot is simple
enough not to need one. The inverse kinematics stay closed-form: for the
heading ψ (+y = π/2, -y = -π/2) the wrist axis has to be at the goal minus
the tip offset turned to ψ; the two-link solution puts it there, z comes
from the carriage, and `joint_3` = ψ − joint_1 − joint_2 (with the URDF's
fixed turns and axis signs), in every turn that fits its limits. Every
joint gets its own time-optimal profile (full acceleration, top speed, full
braking) and they all start together. `joint_3` is not one of them: at every
instant it is ψ(t) − joint_1(t) − joint_2(t), so the gripper keeps pointing
to the side while the arm swings. When ψ has to change (the other side, or
from home, where the gripper points along link 2) ψ gets a time-optimal
profile of its own, the short way round. `joint_3`'s speed and acceleration
follow from the others; if they come out over its `motion.yaml` limits, the
whole move runs slower, uniformly in time, by the smallest factor that fits
(note `slowed to …`; from a moving start it stops first instead).

The path (`joint_3` and the gripper box included) is checked against the
collision boxes from `robot_parameters.yaml` every time any point of the arm
has moved 1 cm, and `joint_3` against its limits. If it hits, one revolute
joint starts a little later (typically joint_1 waits a tenth of a second for
link 2 to unfold), the shortest wait that clears wins. If no wait clears it,
the arm stops once on the way at an intermediate joint_1. Planning takes
10–100 ms, up to about a second when it needs that stop. The trajectory goes
straight to `arm_controller` (`joint_3` included, so it is time-synced with
the arm) and `crate_controller`, both stamped to start at the same instant.

At the ±120° joint_1 limit the `mid_link` box touches the gantry box, so the
arm stops just short of it. Links on the same rail (all of the arm rides the
carriage) never move up or down relative to each other, so between them the
margin is left out vertically, and pairs that are apart in height are never
checked: the gripper hangs 2.5 cm under link 1 and the carriage and can
never hit them. What it can hit is the gantry and the crate.

The plugin runs its own CAN loop (`can_rate_hz`, now 50 Hz). For each real motor, every cycle,
it sends an absolute position move (F5) that aims slightly ahead of the
trajectory (by the driver's braking distance plus the travel during two
measured CAN cycles), at the trajectory's speed plus a catch-up term, and
reads the encoder back (0x31). It talks to one driver at a time and waits for
each reply, because these drivers produce CAN errors when two of them answer
at once. So the cycle gets longer with every motor: with four it took 12–14.5
ms on can0, with five about 16.5 ms, so `can_rate_hz` is 50 (20 ms). When it
does not fit the plugin warns (`CAN cycle takes … ms`) and suggests a rate.

CAN wiring: CAN_H to CAN_L measures 60 Ω with the power off, with a 120 Ω
terminator at each physical end of the chain (adapter and last driver). Every
driver: `Mode = SR_vFOC`, `CanRate = 500K`, a unique `CanID`, `Respon` and
`Active` on.

## Next

1. **z_carriage:** small steps, then check 125 rev/m with a tape measure,
   and the direction.
2. **z_crate:** fill in its transmission, then the same.
3. **joint_3 (wrist yaw, CAN ID 5):** direction (`reversed`) and the 5:1
   scaling (command 1.571 rad, check 90°), then the cable range from home:
   set `joints.joint_3.limit` in `robot_parameters.yaml` to it.
4. **Gripper tip:** measure it in `end_effector`'s frame with the pitch at 0
   and set `motion.yaml` `tool.xyz` (now 0.21 m along +x, from the CAD).
5. **can_rate_hz with 5 motors:** watch for `CAN cycle takes … ms` and use
   the rate it suggests.
6. **Crate control:** how the crate should move with the arm. For now it
   stays at 0 and moves only by hand (panel slider).
7. Real joint limits, efforts and velocities. The current ones are guesses,
   the wrist's 6 rad/s and 15 rad/s² too.
8. Pitch and gripper: their motors, joints and a `gripper_controller`; then
   `dexterity:=true` on the robot.
9. Homing against the limit switches, to replace zero-at-startup.
