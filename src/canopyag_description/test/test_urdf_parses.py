"""Guard rail: the xacro must expand, the tree must be valid, the meshes must
exist and controllers.yaml must name joints that are actually in the URDF.

These are the four things that break when you re-import a SolidWorks export.

Run standalone: pytest src/canopyag_description/test/test_urdf_parses.py
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
URDF_XACRO = PKG / "urdf" / "canopyag.urdf.xacro"
PARAMS = PKG / "config" / "robot_parameters.yaml"
CONTROLLERS = PKG / "config" / "controllers.yaml"


def expand(**args):
    cmd = ["xacro", str(URDF_XACRO)] + [f"{k}:={v}" for k, v in args.items()]
    return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout


@pytest.fixture(scope="module")
def urdf():
    return ET.fromstring(expand(sim="true"))


@pytest.fixture(scope="module")
def cfg():
    return yaml.safe_load(PARAMS.read_text())


@pytest.mark.parametrize("sim", ["true", "false"])
def test_xacro_expands(sim):
    assert "<robot" in expand(sim=sim)


def test_every_joint_in_the_yaml_reaches_the_urdf(urdf, cfg):
    mount = cfg["mount"]["parent"] + "_to_" + cfg["robot"]["root_link"]
    assert {j.get("name") for j in urdf.findall("joint")} == set(cfg["joints"]) | {mount}


def test_meshes_exist(cfg):
    for name, link in cfg["links"].items():
        if "mesh" in link:
            assert (PKG / cfg["meshes"]["visual_dir"] / link["mesh"]).is_file(), name
            if cfg["meshes"]["use_collision_meshes"]:
                assert (PKG / cfg["meshes"]["collision_dir"] / link["mesh"]).is_file(), name


def test_controllers_only_claim_joints_that_exist(urdf):
    movable = {
        j.get("name") for j in urdf.findall("joint")
        if j.get("type") in ("revolute", "prismatic", "continuous")
    }
    controllers = yaml.safe_load(CONTROLLERS.read_text())
    for name, block in controllers.items():
        joints = block.get("ros__parameters", {}).get("joints", [])
        assert set(joints) <= movable, f"{name} names joints not in the URDF"


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
