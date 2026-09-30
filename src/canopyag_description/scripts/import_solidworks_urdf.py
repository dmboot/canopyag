#!/usr/bin/env python3
"""Turn a SolidWorks (sw_urdf_exporter) export into config/robot_parameters.yaml.

    python3 scripts/import_solidworks_urdf.py <export>/urdf/urdf.urdf

Copies the STLs into meshes/visual/ and rewrites config/robot_parameters.yaml
from the export's links, joints, origins, axes, limits and inertials.

Values you tuned by hand are kept: for every link and joint that still exists
under the same name, the keys in PRESERVE below are carried over from the old
yaml instead of being overwritten, as are the whole `mount`, `meshes` and
`materials` and `counterweights` sections. So the loop is: re-export from SolidWorks, re-run this,
and only the geometry moves. Pass --no-preserve to start from a clean slate.

The export is not always self-consistent (see the frame check at the bottom of
the output), so read the warnings before launching.
"""

import argparse
import re
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("needs pyyaml:  pip install pyyaml")

# Per-link and per-joint keys that survive a re-import. `note` is free text
# that comes back out as a comment above the entry - use it to record why a
# value was overridden, because YAML comments themselves are not preserved.
PRESERVE = {
    "link": ("note", "material", "visual_xyz", "visual_rpy", "collision_box"),
    "joint": ("note", "limit", "dynamics"),
}

# sw_urdf_exporter leaves effort/velocity at 0 unless you fill them in in the
# exporter dialog. A 0 limit means "cannot move" to Gazebo and to MoveIt, so
# substitute something usable and say so.
FALLBACK_EFFORT = {"prismatic": 200.0, "revolute": 50.0, "continuous": 50.0}
FALLBACK_VELOCITY = {"prismatic": 0.25, "revolute": 1.5, "continuous": 1.5}


# --------------------------------------------------------------------------
# reading the export
# --------------------------------------------------------------------------

def sanitize(name):
    """'joint z1' -> 'joint_z1'. Spaces in joint names are legal URDF but they
    break every ros2 control / ros2 param command line you will ever type."""
    return re.sub(r"[^a-z0-9_]+", "_", name.strip().lower()).strip("_")


def triplet(elem, attr, default=(0.0, 0.0, 0.0)):
    if elem is None or elem.get(attr) is None:
        return list(default)
    return [round(float(v), 9) for v in elem.get(attr).split()]


def parse_export(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links, joints, colors = {}, {}, {}

    for le in root.findall("link"):
        name = sanitize(le.get("name"))
        entry = {}

        mesh = le.find("./visual/geometry/mesh")
        if mesh is not None:
            entry["mesh"] = Path(mesh.get("filename")).name.lower()
            entry["visual_xyz"] = triplet(le.find("./visual/origin"), "xyz")
            entry["visual_rpy"] = triplet(le.find("./visual/origin"), "rpy")

        color = le.find("./visual/material/color")
        if color is not None:
            rgba = tuple(round(float(v), 4) for v in color.get("rgba").split())
            entry["material"] = colors.setdefault(rgba, f"colour_{len(colors) + 1}")

        inertial = le.find("inertial")
        if inertial is not None:
            i = inertial.find("inertia")
            entry["mass"] = round(float(inertial.find("mass").get("value")), 9)
            entry["com_xyz"] = triplet(inertial.find("origin"), "xyz")
            entry["com_rpy"] = triplet(inertial.find("origin"), "rpy")
            entry["inertia"] = {k: round(float(i.get(k)), 12)
                                for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}

        links[name] = entry

    for je in root.findall("joint"):
        jtype = je.get("type")
        entry = {
            "type": jtype,
            "parent": sanitize(je.find("parent").get("link")),
            "child": sanitize(je.find("child").get("link")),
            "xyz": triplet(je.find("origin"), "xyz"),
            "rpy": triplet(je.find("origin"), "rpy"),
        }
        if jtype != "fixed":
            entry["axis"] = triplet(je.find("axis"), "xyz", (0.0, 0.0, 1.0))
            lim = je.find("limit")
            limit = {
                "effort": float(lim.get("effort", 0)) if lim is not None else 0.0,
                "velocity": float(lim.get("velocity", 0)) if lim is not None else 0.0,
            }
            if jtype != "continuous":
                limit["lower"] = float(lim.get("lower", 0)) if lim is not None else 0.0
                limit["upper"] = float(lim.get("upper", 0)) if lim is not None else 0.0
            # The exporter writes a revolute joint with no limits set in its
            # dialog as lower == upper == 0: a joint that cannot move. That is
            # how it spells "continuous".
            if jtype == "revolute" and limit["lower"] == limit["upper"] == 0.0:
                entry["type"] = "continuous"
                del limit["lower"], limit["upper"]
            entry["limit"] = limit
        joints[sanitize(je.get("name"))] = entry

    materials = {v: list(k) for k, v in colors.items()}
    return links, joints, materials


def subtree_mass(joint, links, joints):
    """Mass of everything that moves with `joint`: its child link and every
    link below it. This is what a counterweight on that joint has to carry."""
    todo, total = [joints[joint]["child"]], 0.0
    while todo:
        link = todo.pop()
        total += links.get(link, {}).get("mass", 0.0)
        todo += [J["child"] for J in joints.values() if J["parent"] == link]
    return total


def root_link_of(links, joints):
    children = {j["child"] for j in joints.values()}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        sys.exit(f"expected exactly one root link, found {roots}")
    return roots[0]


# --------------------------------------------------------------------------
# sanity checks on the export
# --------------------------------------------------------------------------

def stl_bbox(path):
    """(lo, hi) of a binary STL, or None if it is ASCII / unreadable."""
    try:
        with open(path, "rb") as fh:
            fh.read(80)
            count = struct.unpack("<I", fh.read(4))[0]
            data = fh.read(count * 50)
        if len(data) < count * 50:
            return None
        lo, hi = [1e18] * 3, [-1e18] * 3
        for t in range(count):
            for v in range(3):
                p = struct.unpack_from("<3f", data, t * 50 + 12 + v * 12)
                for a in range(3):
                    lo[a] = min(lo[a], p[a])
                    hi[a] = max(hi[a], p[a])
        return lo, hi
    except Exception:
        return None


def check_frames(links, mesh_dir):
    """The centre of mass must land inside the mesh's bounding box. When it
    does not, the STL and the link's inertial data were written in different
    frames - usually because the link's reference coordinate system in
    SolidWorks is rotated relative to the assembly origin, which the exporter
    applies to the numbers but not to the base link's STL. The fix is a
    visual_rpy on that link (or a corrected export)."""
    bad = []
    for name, L in links.items():
        if "mesh" not in L or "com_xyz" not in L:
            continue
        box = stl_bbox(mesh_dir / L["mesh"])
        if box is None:
            continue
        lo, hi = box
        pad = [0.05 * (hi[a] - lo[a]) + 1e-6 for a in range(3)]
        if any(not (lo[a] - pad[a] <= L["com_xyz"][a] <= hi[a] + pad[a]) for a in range(3)):
            bad.append((name, L["com_xyz"], lo, hi))
    return bad


# --------------------------------------------------------------------------
# writing the yaml
# --------------------------------------------------------------------------

def num(v):
    if isinstance(v, float) and v == int(v) and abs(v) < 1e6:
        return f"{int(v)}.0"
    return repr(v)


def vec(v):
    return "[" + ", ".join(num(float(x)) for x in v) + "]"


def flow(d):
    return "{" + ", ".join(f"{k}: {num(v)}" for k, v in d.items()) + "}"


def note_lines(entry, indent):
    """A preserved `note` comes back as a YAML block scalar rather than a
    comment, because comments do not survive a re-import and notes have to."""
    text = entry.get("note")
    if not text:
        return []
    return [f"{indent}note: |-"] + [f"{indent}  {line}"
                                    for line in str(text).strip().splitlines()]


def emit(name, root, mount, meshes, materials, links, joints, counterweights, notes):
    out = [
        "# ===========================================================================",
        f"# {name} - every number the URDF is built from",
        "# ===========================================================================",
        "# GENERATED by scripts/import_solidworks_urdf.py from a SolidWorks export.",
        "# Editing it by hand is fine and expected: re-running the importer keeps",
        f"#   links:  {', '.join(PRESERVE['link'])}",
        f"#   joints: {', '.join(PRESERVE['joint'])}",
        "# and the whole mount / meshes / materials sections. Everything else",
        "# (topology, origins, axes, mass, inertia) comes back from the export.",
        "#",
        "# Units are metres, radians, kilograms. Frames follow URDF: a joint's xyz/rpy",
        "# is its origin in the PARENT link's frame, and a link's mesh is drawn in that",
        "# link's own frame.",
        "#",
        "# A `note:` on any entry is free text that survives a re-import - the # comments",
        "# in this file do not, so record anything you want to keep as a note.",
        "",
        "robot:",
        f"  name: {name}",
        f"  root_link: {root}",
        "",
        "# world -> root_link. This is where you place the robot in the scene, and",
        "# where you fix a SolidWorks assembly that was not modelled Z-up.",
        "mount:",
        f"  parent: {mount['parent']}",
        f"  xyz: {vec(mount['xyz'])}",
        f"  rpy: {vec(mount['rpy'])}",
        *note_lines(mount, "  "),
        "",
        "meshes:",
        f"  package: {meshes['package']}",
        f"  visual_dir: {meshes['visual_dir']}",
        f"  collision_dir: {meshes['collision_dir']}",
        f"  scale: {vec(meshes['scale'])}",
        "  # false -> collision reuses the visual mesh (fine for RViz and MoveIt).",
        "  # true  -> collision uses collision_dir, which must hold decimated copies.",
        "  #          Do that before running Gazebo; raw exports are ~25k triangles",
        "  #          a link and the physics step will crawl.",
        f"  use_collision_meshes: {str(meshes['use_collision_meshes']).lower()}",
        "",
        "materials:",
    ]
    for mname, rgba in materials.items():
        out.append(f"  {mname}: {vec(rgba)}")

    out += [
        "",
        "# Prismatic joints whose moving mass is balanced by a counterweight on the",
        "# other side of the gantry. `mass` is the counterweight itself; when it",
        "# equals the mass that joint carries, the axis feels no gravity load.",
        "# The links keep their real mass and inertia either way - only gravity along",
        "# the axis is offset. Gazebo applies it as a constant upward force on the",
        "# joint's child link (see canopyag_bringup/launch/gazebo.launch.py).",
        "counterweights:" + ("" if counterweights else " {}"),
    ]
    for jname, C in counterweights.items():
        out.append(f"  {jname}:")
        out += note_lines(C, "    ")
        out.append(f"    mass: {num(float(C['mass']))}")

    out += ["", "links:"]
    for lname, L in links.items():
        out.append(f"  {lname}:")
        out += note_lines(L, "    ")
        if "mesh" in L:
            out.append(f"    mesh: {L['mesh']}")
            out.append(f"    material: {L.get('material', 'default')}")
            out.append(f"    visual_xyz: {vec(L['visual_xyz'])}")
            out.append(f"    visual_rpy: {vec(L['visual_rpy'])}")
        if "collision_box" in L:
            B = L["collision_box"]
            out.append(f"    collision_box: {{size: {vec(B['size'])}, xyz: {vec(B['xyz'])}}}")
        if "mass" in L:
            out.append(f"    mass: {num(L['mass'])}")
            out.append(f"    com_xyz: {vec(L['com_xyz'])}")
            out.append(f"    com_rpy: {vec(L['com_rpy'])}")
            out.append(f"    inertia: {flow(L['inertia'])}")

    out += ["", "joints:"]
    for jname, J in joints.items():
        out.append(f"  {jname}:")
        out += note_lines(J, "    ")
        out.append(f"    type: {J['type']}")
        out.append(f"    parent: {J['parent']}")
        out.append(f"    child: {J['child']}")
        out.append(f"    xyz: {vec(J['xyz'])}")
        out.append(f"    rpy: {vec(J['rpy'])}")
        if "axis" in J:
            out.append(f"    axis: {vec(J['axis'])}")
            out.append(f"    limit: {flow(J['limit'])}")
            out.append(f"    dynamics: {flow(J.get('dynamics', {'damping': 0.0, 'friction': 0.0}))}")

    if notes:
        out += ["", "# Importer notes from the last run:"] + [f"#   {n}" for n in notes]
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("export", type=Path, help="urdf.urdf produced by sw_urdf_exporter")
    ap.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parents[1],
                    help="canopyag_description root (default: alongside this script)")
    ap.add_argument("--no-preserve", action="store_true",
                    help="overwrite hand-tuned values instead of carrying them over")
    ap.add_argument("--dry-run", action="store_true", help="report, write nothing")
    args = ap.parse_args()

    pkg = args.package_dir
    out_yaml = pkg / "config" / "robot_parameters.yaml"
    links, joints, materials = parse_export(args.export)
    root = root_link_of(links, joints)

    old = {}
    if out_yaml.exists() and not args.no_preserve:
        old = yaml.safe_load(out_yaml.read_text()) or {}

    name = old.get("robot", {}).get("name", pkg.name.replace("_description", ""))
    mount = old.get("mount") or {"parent": "world", "xyz": [0.0] * 3, "rpy": [0.0] * 3}
    meshes = old.get("meshes") or {
        "package": pkg.name,
        "visual_dir": "meshes/visual",
        "collision_dir": "meshes/collision",
        "scale": [1.0, 1.0, 1.0],
        "use_collision_meshes": False,
    }
    # Reuse the names you gave the colours last time instead of inventing
    # colour_1/colour_2 again.
    old_by_rgba = {tuple(v): k for k, v in (old.get("materials") or {}).items()}
    renamed = {old_by_rgba.get(tuple(rgba), mname): rgba
               for mname, rgba in materials.items()}
    for L in links.values():
        if "material" in L:
            L["material"] = old_by_rgba.get(tuple(materials[L["material"]]), L["material"])
    materials = {**renamed, **(old.get("materials") or {})}

    notes, kept = [], []

    # Counterweights are hand-written, not in the export. Keep the ones whose
    # joint still exists and is still prismatic.
    counterweights = {}
    for jname, C in (old.get("counterweights") or {}).items():
        if joints.get(jname, {}).get("type") == "prismatic":
            counterweights[jname] = C
        else:
            notes.append(f"counterweights.{jname} dropped - no prismatic joint "
                         f"of that name in the export")

    # carry hand-tuned values over
    for kind, table, keys in (("link", links, PRESERVE["link"]),
                              ("joint", joints, PRESERVE["joint"])):
        for n, entry in table.items():
            prev = (old.get(kind + "s") or {}).get(n)
            if not prev:
                continue
            for k in keys:
                if k in prev and prev[k] != entry.get(k):
                    entry[k] = prev[k]
                    kept.append(f"{kind} {n}.{k}")

    # An empty limit in the export makes a joint continuous (see above), but a
    # hand-tuned lower/upper that survived the import says it is limited.
    for n, J in joints.items():
        lim = J.get("limit", {})
        if J["type"] == "continuous" and lim.get("lower", 0.0) < lim.get("upper", 0.0):
            J["type"] = "revolute"
            notes.append(f"{n}: export has no limits, kept revolute because of the "
                         f"preserved limit {lim['lower']} .. {lim['upper']}")

    # usable effort/velocity
    for n, J in joints.items():
        if "limit" not in J:
            continue
        for key, table in (("effort", FALLBACK_EFFORT), ("velocity", FALLBACK_VELOCITY)):
            if not J["limit"].get(key):
                J["limit"][key] = table[J["type"]]
                notes.append(f"{n}: {key} was 0 in the export, set to "
                             f"{table[J['type']]} - replace with the real number")

    # copy meshes
    src_dir = args.export.resolve().parent.parent / "meshes"
    dst_dir = pkg / meshes["visual_dir"]
    copied = []
    if src_dir.is_dir() and not args.dry_run:
        dst_dir.mkdir(parents=True, exist_ok=True)
        wanted = {L["mesh"] for L in links.values() if "mesh" in L}
        for f in src_dir.iterdir():
            if f.name.lower() in wanted:
                shutil.copyfile(f, dst_dir / f.name.lower())
                copied.append(f.name.lower())
        for stale in dst_dir.iterdir():
            if stale.name not in wanted:
                notes.append(f"meshes/visual/{stale.name} is no longer referenced")

    missing = [L["mesh"] for L in links.values()
               if "mesh" in L and not (dst_dir / L["mesh"]).exists()]

    text = emit(name, root, mount, meshes, materials, links, joints, counterweights, notes)
    if not args.dry_run:
        out_yaml.parent.mkdir(parents=True, exist_ok=True)
        out_yaml.write_text(text)

    # ---- report ----
    print(f"root link : {root}")
    print(f"links     : {len(links)}  ({', '.join(links)})")
    movable = [n for n, J in joints.items() if J["type"] != "fixed"]
    print(f"joints    : {len(joints)}  ({len(movable)} movable: {', '.join(movable)})")
    print(f"meshes    : {len(copied)} copied -> {meshes['visual_dir']}/")
    if kept:
        print(f"preserved : {', '.join(kept)}")
    print(f"{'would write' if args.dry_run else 'wrote'}: {out_yaml}")

    for n in notes:
        print(f"  note: {n}")
    for jname, C in counterweights.items():
        carried = subtree_mass(jname, links, joints)
        print(f"  counterweight {jname}: {float(C['mass']):.3f} kg against "
              f"{carried:.3f} kg moving -> net {carried - float(C['mass']):+.3f} kg on the axis")
    for m in missing:
        print(f"  WARNING: mesh {m} not found - copy it into {meshes['visual_dir']}/")
    for lname, com, lo, hi in check_frames(links, dst_dir):
        print(f"  WARNING: {lname}: centre of mass {[round(c, 3) for c in com]} is outside "
              f"its mesh bbox {[round(c, 3) for c in lo]}..{[round(c, 3) for c in hi]}.")
        print(f"           The STL and the link frame disagree. Fix the reference "
              f"coordinate system in SolidWorks, or set links.{lname}.visual_rpy.")
    print("\nnext: check controllers.yaml still lists the right joints, then\n"
          "      ros2 launch canopyag_description view_robot.launch.py")


if __name__ == "__main__":
    main()
