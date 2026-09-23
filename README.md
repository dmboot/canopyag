# canopyag

ROS 2 stack for the canopy.ag harvesting robot: a vertical gantry carrying a
2-axis SCARA arm, plus a crate that rides the same gantry.

Target: **ROS 2 Jazzy / Ubuntu 24.04 / Gazebo Harmonic**

## Packages

| package | what's in it |
|---|---|
| `canopyag_description` | URDF/xacro, meshes, parameters, controller config, RViz |
| `canopyag_bringup` | launch files (sim + hardware), Gazebo world |
| `canopyag_hardware` | not written yet - the CAN `SystemInterface` plugin |
| `canopyag_moveit_config` | MoveIt 2 config (Pilz + OMPL) and the scripted demo path on mock hardware |

## Build & run

```bash
cd ~/canopyag
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
                     GenericSystem
```

`use_sim_time` is deliberately absent from `controllers.yaml`:
`gz_ros2_control` forces it on in Gazebo, and a node-specific entry there would
override the launch files' `false` on mock and real hardware, leaving the
controller manager waiting for a `/clock` that never comes.

## MoveIt and the demo path

`canopyag_moveit_config` is hand-written rather than generated - the robot is
small and the CAD still changes. It has two planning groups:

- `arm`: `z_carriage`, `joint_1`, `joint_2`, `joint_3`. One group, so a plan
  moves the carriage and the arm at once and they arrive together.
- `crate`: `z_crate`. Not planned; driven directly by the demo script.

The default planner is **Pilz PTP**: deterministic joint-space moves, the same
path every run, which is what a hard-coded demo wants. OMPL is loaded too, for
dragging the interactive marker around in RViz.

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
4. `canopyag_hardware` - CAN `SystemInterface`, tested against `vcan0` first
5. Homing against the limit switches
