# canopyag

ROS 2 stack for the canopy.ag harvesting robot: a vertical gantry carrying a
2-axis SCARA arm, plus a crate that rides the same gantry.

Target: **ROS 2 Humble / Ubuntu 22.04 / Gazebo Fortress**

## Packages

| package | what's in it |
|---|---|
| `canopyag_description` | URDF/xacro, meshes, parameters, controller config, RViz |
| `canopyag_bringup` | launch files (sim + hardware), Gazebo world |
| `canopyag_hardware` | the CAN `SystemInterface` plugin for the MKS SERVO57D drivers, the `vcan0` driver simulator, the RViz ghost |
| `canopyag_moveit_config` | MoveIt 2 config (Pilz + OMPL) and the scripted demo path on mock hardware |

## Build & run

```bash
cd ~/canopy_ag
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Kinematics check in RViz, with a slider per joint:

```bash
ros2 launch canopyag_description view_robot.launch.py
```

MoveIt on mock hardware, running the demo path (see below):

```bash
ros2 launch canopyag_moveit_config demo.launch.py            # once
ros2 launch canopyag_moveit_config demo.launch.py loop:=true
```

Gazebo with live controllers:

```bash
ros2 launch canopyag_bringup gazebo.launch.py
ros2 control list_controllers
```

## The model

Everything the URDF is built from lives in
`src/canopyag_description/config/robot_parameters.yaml` - links, joints,
origins, axes, limits, masses, inertias, mesh filenames, materials, and where
the robot is mounted in the world. The xacro files contain no robot-specific
values at all: `macros.xacro` just walks the `links:` and `joints:` tables and
emits one element per entry.

Current chain, straight from the CAD:

```
world
└── gantry                                         (fixed mount)
    ├── z_carriage  prismatic  0 .. 1.22 m         carriage       counterweighted
    │   └── joint_1  revolute  ±1.65 rad           mid_link
    │       └── joint_2  continuous                endeffector_link
    │           └── joint_3  continuous            endeffector
    └── z_crate     prismatic  0 .. 1.215 m        crate          counterweighted
```

The crate and the carriage ride the same rail, with the carriage above the
crate. At home both are at 0 and touch, so the gap between crate top and
carriage bottom is simply `z_carriage - z_crate`. The URDF limits alone do not
stop the crate passing the carriage; the collision boxes and the demo script
do (see MoveIt below).

### Collision geometry

Each link has a hand-written `collision_box: {size, xyz}` in
`robot_parameters.yaml`, in the link's own frame. It replaces the mesh for
collision everywhere (MoveIt, Gazebo), survives a re-import, and does not
change when the CAD does - but it does not follow the CAD either: when a part
changes shape (the end effector will), update its box. Tick "Show Robot
Collision" in RViz's MotionPlanning display to see them. Links without a
`collision_box` fall back to the mesh.

### Counterweights

Both axes are counterweighted on the other side of the gantry, so the drives
see (almost) no gravity load. The `counterweights:` section of
`robot_parameters.yaml` records the counterweight mass per prismatic joint.
The links keep their real mass, centre of mass and inertia, so the arm's
dynamics are unchanged; only gravity along the axis is offset.

URDF cannot express this, and Gazebo Harmonic ignores per-link `<gravity>`, so
`gazebo.launch.py` sends each counterweight as a constant upward force
(`mass * 9.8` N, world +Z) on the joint's child link, through the
`ApplyLinkWrench` system loaded by the world. The masses there now are the
CAD mass of each moving subtree (perfect balance); replace them with the
weighed counterweights. The importer prints the net load per axis on every run.

Not modelled: the counterweight's own inertia. It moves opposite the carriage,
so the drive accelerates roughly twice the carriage mass (plus pulleys). That
matters once the axes are effort-controlled, not with the position control
used now.

## Re-importing from SolidWorks

The CAD changes; this is the loop. Export the assembly with the
[sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter), then:

```bash
python3 src/canopyag_description/scripts/import_solidworks_urdf.py \
        /path/to/export/urdf/urdf.urdf
```

That copies the STLs into `meshes/visual/` and rewrites
`robot_parameters.yaml` from the export. Values you tuned by hand survive:
per link `note`, `material`, `visual_xyz`, `visual_rpy`, `collision_box`; per joint `note`,
`limit`, `dynamics`; and the whole `mount`, `meshes`, `materials` and
`counterweights` sections.
Everything else - topology, origins, axes, mass, inertia - comes back fresh
from the export. `--dry-run` reports without writing, `--no-preserve` starts
clean.

The importer also checks each link's centre of mass against its mesh bounding
box and warns when the two disagree, which is how a mesh exported in the wrong
frame shows up before you ever open RViz.

Two things it cannot know about, so check them after a re-import:

- **Joint names.** Spaces are stripped (`joint 1 ` -> `joint_1`). If you
  rename or add a joint in SolidWorks, update `config/controllers.yaml` to
  match; the test below fails the build if they drift apart.
- **The exporter leaves effort and velocity at 0** unless you fill them in in
  the export dialog. Zero means "cannot move" to Gazebo and MoveIt, so the
  importer substitutes usable defaults and prints a note. Replace them with
  real numbers when you have them.
- **A revolute joint with lower == upper == 0** is how the exporter writes a
  joint whose limits were left empty. The importer turns it into a
  `continuous` joint.

```bash
colcon test --packages-select canopyag_description
```

checks that the xacro expands, that the tree is valid, that every mesh
referenced actually exists, and that `controllers.yaml` only names joints that
are in the URDF.

## Known issues in the current export

- **The exporter can leak one part into other links' STLs.** It hides every
  component, then shows one link's parts per STL; a component that refuses to
  hide (lightweight, in Edit Part, pinned by a display state) ends up in every
  STL written before its own. It happened to `mid_link-1`. Symptom in RViz:
  copies of a link that stay behind or swing with the wrong joint. Check the
  file sizes against the previous export.
- **The end effector collision box is the current mesh's bounding box.**
  Update `links.endeffector.collision_box` when the design changes.

## Simulation vs. hardware

Everything goes through **ros2_control**, so the URDF, the controllers and
MoveIt are identical in both. The only thing that changes is which hardware
plugin the `<ros2_control>` block names, and `sim:=` / `mock:=` on the xacro
are the whole switch:

```
          joint_trajectory_controller  (arm_controller, crate_controller)
                          |
                  controller_manager
            /             |               \
   sim:=true         mock:=true             (neither)
   gz_ros2_control   mock_components/       canopyag_hardware -> SocketCAN
                     GenericSystem          + config/hardware.yaml
```

`config/hardware.yaml` is only read in the last case; sim and mock output of
the xacro does not depend on it (a test checks that).

`use_sim_time` is deliberately absent from `controllers.yaml`:
`gz_ros2_control` forces it on in Gazebo, and a node-specific entry there would
override the launch files' `false` on mock and real hardware, leaving the
controller manager waiting for a `/clock` that never comes.

## Hardware bring-up

### Wiring and termination

Chain CAN_H / CAN_L through every driver, with a 120 Ω terminator at **each
end** of the chain. With the power off, CAN_H to CAN_L must measure **60 Ω**.
At 120 Ω (one terminator missing) the adapter goes ERROR-PASSIVE, then
bus-off, and frames get resent thousands of times.

### Driver menu settings (every driver)

| menu | value | why |
|---|---|---|
| `Mode` | `SR_vFOC` | CAN motion commands need an SR mode |
| `CanRate` | `500K` | must match `ip link ... bitrate` |
| `CanID` | unique, see the joint table | every driver ships with ID 1; two equal IDs flood the bus |
| `Respon`, `Active` | enabled | otherwise the drivers never send replies / "done" |
| working current | set for the motor | |

### The bus

The adapter is a CANable on the `gs_usb` driver:

```bash
~/nema_test/setup_can.sh              # = ip link set can0 type can bitrate 500000; up
candump can0                          # in a second terminal: watch the traffic
python3 ~/nema_test/status.py         # every motor must reply
```

`gs_usb` does **not** support `restart-ms`, so a bus-off never clears by
itself. The plugin reports it (it listens for error frames) and faults;
recover with

```bash
sudo ip link set can0 down && sudo ip link set can0 up
```

after fixing the cause (almost always termination or a loose wire).

### Zero at startup

There is no homing yet (roadmap 5). When the hardware activates, each
joint's **current** position is taken as its `initial_position` (0.0 = URDF
home), and the plugin logs that in a banner. So:

- switch the drivers on with the robot **at home**;
- the encoder count restarts at every driver power-up, so after a power
  cycle, move the robot home by hand and restart the launch;
- after any fault the hardware component goes to *finalized* and stays
  there. Restart the launch to recover; it re-zeroes, so home first.

### Running it

```bash
ros2 launch canopyag_bringup hardware.launch.py rviz:=true
```

`arm_controller` and `forward_position_controller` both come up
**inactive** (`crate_controller` is active). Check that the robot is at
home, then activate one of them. They claim the same joints and cannot be
active at once:

```bash
ros2 control switch_controllers --activate forward_position_controller
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.05, 0.0, 0.0]}"      # z_carriage, joint_1, joint_2, joint_3
```

`hw_log_level:=debug` shows every F5 in motor counts, motor revs **and**
joint units, plus the frame rates and missed replies every 10 s.

### Real and virtual joints, and the ghost in RViz

Each joint in `config/hardware.yaml` is `mode: can` (a real driver) or
`mode: virtual` (no motor yet: the plugin copies command to state, like mock
hardware, with no CAN traffic). All five joints stay in the one ros2_control
system either way, so `arm_controller`, the forward controller and MoveIt
work unchanged while only z_carriage and joint_1 have motors. Adding a motor
means changing its line to `mode: can` and filling in its values.

With `rviz:=true` RViz shows two robots:

| display | source | what it is |
|---|---|---|
| Measured (solid) | `/joint_states` | the encoders (virtual joints: their command) |
| Commanded (ghost, see-through) | `ghost_state.py` -> `/ghost/joint_states`, frames `cmd/...` | what the active controller asks for |

If the hardware tracks, the two overlap. A ghost that runs ahead means lag
(`max_motor_rpm` or `acceleration` too low); a ghost that moves the other
way means `reversed` is wrong; a ghost that goes further or less far means
`motor_revs_per_unit` is wrong. `ghost:=false` turns it off.

### Joint mapping

`src/canopyag_description/config/hardware.yaml` is **the one file to
edit**. Every value you are expected to change is marked `# <-- EDIT`.

| joint | CAN ID | mode | transmission | `motor_revs_per_unit` | `reversed` | limits | F5 range |
|---|---|---|---|---|---|---|---|
| `z_carriage` | 1 | can | 5:1 gearbox, GT2 belt (2 mm pitch, 6 mm wide), 20T pulley: 8 mm per motor rev | 125 rev/m | false, to confirm | 0 .. 1.22 m | ±4.1 m |
| `joint_1` | 3 | can | 16:1 gearbox | 16 / 2π = 2.546 rev/rad | false, to confirm | ±1.65 rad (hardware test: ±1.571) | ±201 rad |
| `z_crate` | 2 | virtual | TODO | TODO | | 0 .. 1.215 m | |
| `joint_2` | TODO (4?) | virtual | TODO | TODO | | continuous | ±512 / `motor_revs_per_unit` rad |
| `joint_3` | TODO (5?) | virtual | TODO | TODO | | continuous | ±512 / `motor_revs_per_unit` rad |

`motor_revs_per_unit` is motor revs per rad (revolute) or per metre
(prismatic): `G / 2π` for a gearbox G, `1 / L` for L metres of travel per
motor rev. The F5 range is the reach of the i24 position in an F5 command,
±8 388 607 counts = ±512 motor revs from the encoder's power-up zero. It only
matters for the continuous joints: at 16:1 that is ±32 link turns. A target
past it is clamped with a one-time warning.

Start with low `max_motor_rpm` (600 on z_carriage = 0.08 m/s, 300 on
joint_1 = 1.96 rad/s). A trajectory faster than the cap lags behind, and
`arm_controller` aborts on its path tolerance. That is the safe way to fail.

### How the plugin talks to the bus

- A CAN thread runs at `can_rate_hz` (100 Hz), separate from the 200 Hz
  controller loop. `read()` and `write()` only swap a mutex-protected
  snapshot with it.
- Each cycle, per motor: if the target moved more than 4 counts, send **F5**
  (absolute encoder position), with its speed taken from how fast the
  target is changing (×1.2, clamped to `[min_rpm, max_motor_rpm]`); then
  request **0x31** (encoder). One motor per cycle also gets **0x3E** (stall).
  A target that is not reached while the motor stands still for 250 ms gets
  its F5 again.
- Velocity state is the low-passed difference of the 0x31 positions. Reading
  0x32 as well would add two frames per motor per cycle for a value the
  position already gives.
- Replies are buffered per (CAN ID, command) and never dropped while waiting
  for another one.
- Bus budget: at 500 kbit/s an 8-byte frame is about 130 bits, so the bus
  carries about 3800 frames/s at most. 2 motors at 100 Hz is about 26%, and
  all 5 is about 57%, which the plugin warns about (over 50%) at startup.
  Lower `can_rate_hz` then.

Startup checks (configure) and faults (while active):

| check | result |
|---|---|
| duplicate `can_id` in hardware.yaml, missing or invalid param | `on_init` fails, naming the joint and param |
| a motor does not answer 0x31 at startup | configure fails, naming the joint and CAN ID |
| more than one reply to one 0x31 | configure fails: two drivers share that CanID |
| 0x3E = 1 at startup | configure fails, with the command to clear it by hand |
| no 0x31 reply for `reply_timeout_ms` × `max_missed_replies` (250 ms) | fault |
| F5 status 0 (rejected) or 3 (end limit) | fault |
| 0x3E = 1 while running | fault (never cleared automatically) |
| adapter bus-off | fault |

On a fault, every motor gets **F7** (emergency stop), `read()` returns ERROR
and the log says which joint and why. Humble then finalizes the hardware
component. Humble's `ros2_control_node` exits if the startup configure fails;
the FATAL line just above says why. A normal shutdown or deactivate sends
**F6 at 0 rpm** instead: a ramped stop that keeps holding torque.

Clearing a latched stall, after checking the mechanics:

```bash
cd ~/nema_test && python3 -c "from mks_servo import *; b=CanBus(); MKSServo(b, 3).release_protection()"
```

### F5 on the real drivers

Streaming F5 relies on behaviour that has only been checked on
`mks_sim.py`, not yet on a real driver. `pos_diag.py` and a small F5 test
answer it:

| question | answer |
|---|---|
| 0x31 and F5 use the same coordinates? | not yet tested |
| a new F5 while one is running: smooth retarget, ignored, or rejected (status 0)? | not yet tested |
| what F5 replies while a move is running, and how often | not yet tested |
| does 0x92 shift the F5 coordinates? | not needed: the zero is kept in software |

The plugin copes with "ignored" (it resends the target once the motor
stops short). "Rejected" makes it fault on the first streamed step; the
fallback then is F6 speed mode with a position loop in the plugin.
`mks_sim.py --retarget smooth|ignore|reject` has all three.

### Bring-up order

1. `~/nema_test/setup_can.sh` and `python3 ~/nema_test/status.py`: all motors reply.
2. Answer the F5 questions above: `python3 ~/nema_test/pos_diag.py --motor motor3`, then a small F5 test.
3. joint_1 alone (CAN ID 3): set z_carriage to `mode: virtual`, then use
   `forward_position_controller` for 0.05 rad steps, then ±90°. Check the
   direction and the 16:1 scaling against a mark on the link.
4. z_carriage (CAN ID 1): small steps, then check the 125 rev/m against a
   tape measure. Then the other joints one at a time, as their table rows are filled in.
5. `arm_controller` with the MoveIt demo, at reduced velocity scaling first.

## Testing on vcan0

Everything above runs without a motor against `mks_sim.py`, a simulated
driver on a virtual CAN bus. Create the bus once per boot:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

Start the simulated drivers, then the plugin:

```bash
ros2 run canopyag_hardware mks_sim.py --ids 1 3
ros2 launch canopyag_bringup hardware.launch.py can_interface:=vcan0 rviz:=true
```

`mks_sim.py` answers every command the plugin uses, with correct CRCs, and
models the MKS acceleration rule. Fault injection (times counted from the
driver's first enable):

| flag | effect | plugin |
|---|---|---|
| `--drop ID --fault-after S` | driver goes silent | fault after 250 ms, F7 |
| `--stall ID --fault-after S` | stall protection trips | fault, F7 |
| `--duplicate-id ID` | two drivers answer on ID | configure fails |
| `--no-done` | F5 never sends status 2 | keeps working (streaming does not need "done") |
| `--latency MS` | every reply delayed | keeps working below the 250 ms watchdog |
| `--retarget smooth\|ignore\|reject` | a new F5 mid-move is followed, ignored or rejected | works, works (resend), fault |

```bash
colcon test
colcon test-result --verbose
```

runs the gtests (CRC, the frame test vectors, i24/i48 signs, unit conversion
both ways, limit clamping, param validation), the xacro checks (including
that sim and mock output do not depend on hardware.yaml), the MoveIt checks,
and `canopyag_bringup/test/test_vcan_launch.py`. That last one runs one full
launch per case above: controllers come up active/inactive as on the robot,
`/joint_states` follows forward-controller commands, and every fault ends
with the hardware finalized and F7 on the bus for every motor. It uses its
own `test/hardware_vcan.yaml`, so editing `config/hardware.yaml` does not
break it. Without vcan0 it is skipped.

## MoveIt and the demo path

`canopyag_moveit_config` is hand-written rather than generated - the robot is
small and the CAD still changes. It has two planning groups:

- `arm`: `z_carriage`, `joint_1`, `joint_2`, `joint_3`. One group, so a plan
  moves the carriage and the arm at once and they arrive together.
- `crate`: `z_crate`. Not planned; driven directly by the demo script.

The demo does not use a planner. Each waypoint is a point-to-point move the
script computes itself: a straight line in joint space, one trapezoidal
profile for all four joints so they start and stop together, each joint held
to its own limit in `config/joint_limits.yaml`. MoveIt collision-checks every
20 ms sample (`/check_state_validity`, crate included) and executes it
(`/execute_trajectory`), so the path is the same every run.

Pilz PTP would do the same, but it gives every joint in a group the strictest
limit of any of them: the carriage's 0.25 m/s and 0.5 m/s² become 0.25 rad/s
and 0.5 rad/s² for the revolutes, about 6x too slow. For planning by hand in
RViz, OMPL is the default pipeline and Pilz is still loaded.

`scripts/demo_path.py` has the path as a plain list, `WAYPOINTS`: the arm
works its way up three levels, reaching left and right and turning the end
effector at each, then comes back down. The crate follows on its own slow
trajectory (`CRATE_UP_SPEED`, 3 cm/s) and trails `CRATE_TRAIL` (15 cm) below
the carriage. Three things keep it under the carriage with at least `GAP`
(2 cm) to spare:

1. the crate's target is never above `min(carriage now, carriage goal) -
   CRATE_TRAIL`;
2. before the carriage is sent lower than `crate + GAP`, the crate is brought
   down first (at `CRATE_DOWN_SPEED`) and the arm waits;
3. the crate and carriage collision boxes are not disabled against each
   other, so MoveIt rejects any plan that would push the carriage into the
   crate.

A monitor on `/joint_states` logs any moment the gap drops below `GAP` and
the script prints the smallest gap at the end of each run. Home itself has a
gap of 0, so the path starts and ends at `REST`, with the carriage 5 cm up.

`colcon test --packages-select canopyag_moveit_config` checks that the SRDF,
the planning limits and the MoveIt controller list only name links and joints
that are in the URDF - re-run it after every re-import.

In simulation `gz_ros2_control` hosts the controller_manager inside the sim
process; on hardware there is a separate `ros2_control_node`. That asymmetry
is the only structural difference between the two launch files.

## Roadmap

1. ~~Description package driven by the SolidWorks export~~
2. Fill in real joint limits, efforts and velocities
3. ~~`canopyag_moveit_config`~~ - demo path on mock hardware
4. `canopyag_hardware` - CAN `SystemInterface`, tested against `vcan0`; real-motor bring-up in progress
5. Homing against the limit switches
