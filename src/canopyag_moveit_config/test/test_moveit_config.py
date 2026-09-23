"""Guard rail: the MoveIt config names only links and joints that exist.

A SolidWorks re-import can rename or drop links; the SRDF and the MoveIt
limit/controller files are hand-written and do not follow on their own.

Run standalone: pytest src/canopyag_moveit_config/test
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

PKG = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def urdf():
    xacro = Path(get_package_share_directory("canopyag_description")) / "urdf" / "canopyag.urdf.xacro"
    out = subprocess.run(["xacro", str(xacro), "sim:=false", "mock:=true"],
                         capture_output=True, text=True, check=True).stdout
    return ET.fromstring(out)


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(PKG / "config" / "canopyag.srdf").getroot()


def names(urdf, tag):
    return {e.get("name") for e in urdf.findall(tag)}


def test_srdf_groups_use_real_joints(urdf, srdf):
    joints = names(urdf, "joint")
    for g in srdf.findall("group"):
        for j in g.findall("joint"):
            assert j.get("name") in joints, f"group {g.get('name')}: {j.get('name')}"
    for s in srdf.findall("group_state"):
        for j in s.findall("joint"):
            assert j.get("name") in joints, f"state {s.get('name')}: {j.get('name')}"


def test_srdf_collision_pairs_use_real_links(urdf, srdf):
    links = names(urdf, "link")
    for d in srdf.findall("disable_collisions"):
        assert {d.get("link1"), d.get("link2")} <= links, (d.get("link1"), d.get("link2"))


def test_every_movable_joint_has_planning_limits(urdf):
    limits = yaml.safe_load((PKG / "config" / "joint_limits.yaml").read_text())["joint_limits"]
    movable = {j.get("name") for j in urdf.findall("joint") if j.get("type") != "fixed"}
    assert movable == set(limits), "joint_limits.yaml out of step with the URDF"
    for j, L in limits.items():
        assert L.get("has_acceleration_limits"), f"{j}: Pilz needs an acceleration limit"


def test_moveit_controllers_use_real_joints(urdf):
    cfg = yaml.safe_load((PKG / "config" / "moveit_controllers.yaml").read_text())
    joints = names(urdf, "joint")
    block = cfg["moveit_simple_controller_manager"]
    for c in block["controller_names"]:
        assert set(block[c]["joints"]) <= joints, c
