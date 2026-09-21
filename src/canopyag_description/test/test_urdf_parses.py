"""Guard rail: the xacro must expand and the URDF must be a valid tree.

This catches the two things that actually break when you edit
config/arm_parameters.yaml: a typo in a key name (KeyError inside xacro) and
a joint that references a link that doesn't exist.

Run standalone: pytest src/canopyag_description/test/test_urdf_parses.py
"""

import subprocess
from pathlib import Path

import pytest

URDF_XACRO = Path(__file__).resolve().parents[1] / "urdf" / "canopyag.urdf.xacro"

EXPECTED_ACTUATED_JOINTS = {
    "z_lift_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_roll_joint",
    "tool_pitch_joint",
    "tool_roll_joint",
    "finger_joint",
}


def expand(**args):
    cmd = ["xacro", str(URDF_XACRO)] + [f"{k}:={v}" for k, v in args.items()]
    return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout


@pytest.mark.parametrize("sim", ["true", "false"])
def test_xacro_expands(sim):
    assert "<robot" in expand(sim=sim)


def test_all_actuated_joints_present():
    import xml.etree.ElementTree as ET

    root = ET.fromstring(expand(sim="true"))
    names = {
        j.get("name")
        for j in root.findall("joint")
        if j.get("type") in ("revolute", "prismatic", "continuous")
        and j.find("mimic") is None
    }
    assert names == EXPECTED_ACTUATED_JOINTS


def test_urdf_tree_is_valid():
    import shutil
    import tempfile

    if shutil.which("check_urdf") is None:
        pytest.skip("check_urdf not installed (apt install liburdfdom-tools)")

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as fh:
        fh.write(expand(sim="true"))
        path = fh.name
    result = subprocess.run(["check_urdf", path], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
