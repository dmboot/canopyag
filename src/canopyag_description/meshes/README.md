# Meshes

Empty on purpose. `options.use_meshes: false` in `config/arm_parameters.yaml`
draws every link as a box, which is enough to check the kinematics.

## Exporting from SolidWorks

Source: `SW/SCARA_V2-Assem/SCARA_V2-Assem.SLDASM`.

For each of the links listed under `links:` in `arm_parameters.yaml`, export
the corresponding sub-assembly (not the individual parts):

| URDF link           | SolidWorks source                          |
|---------------------|--------------------------------------------|
| `base_link`         | SCARA Weldment + 20x40 Extrusion           |
| `lift_link`         | Z-axis Assem (carriage side only)          |
| `upper_arm_link`    | ArmSegmentV3Assem                          |
| `forearm_link`      | ArmSegmentV3Assem (second instance)        |
| `wrist_link`        | 5to1LastAssem                              |
| `tool_link`         | tool plate + pitch bracket                 |
| `gripper_base_link` | ParallelServoGripBody + 2x MG996R Servo    |
| `finger_link`       | ParallelGripDynamicJaw                     |

Two rules that will save you an afternoon:

1. **Move the origin before exporting.** Each mesh must be exported with its
   origin ON that link's parent joint axis, oriented so the joint axis matches
   `axis` in the yaml. SolidWorks exports relative to the part origin, so
   insert a coordinate system at the joint axis and pick it under
   *Save As > STL > Options > Output coordinate system*. If you skip this you
   will spend the next two days fighting `origin_xyz` offsets that can't be
   made to work.
2. **Export mm, and leave `scale="0.001"` alone.** The xacro already scales
   mm -> m. Don't also change the SolidWorks units.

## Two meshes per link

- `visual/<name>.stl` - fine detail is fine, a few hundred kB each.
- `collision/<name>.stl` - **heavily decimated**, ideally convex. Gazebo does
  a narrow-phase check per contact per step; a raw 200k-triangle export will
  drop the real-time factor to single digits. Use *Simplify* in Meshlab or
  just hand-build a convex hull. For `base_link` and the arm segments, a box
  primitive is genuinely better than a mesh - consider leaving those out and
  letting the bbox fallback handle collision.

Then flip `options.use_meshes: true` and relaunch.
