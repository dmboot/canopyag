"""Guard rail: the xacro must expand, the tree must be valid, the meshes must
exist, controllers.yaml must name joints that are actually in the URDF, and
every counterweight must sit on a prismatic joint.

These are the things that break when you re-import a SolidWorks export.

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


def test_counterweights_sit_on_prismatic_joints(cfg):
    for joint, cw in (cfg.get("counterweights") or {}).items():
        assert cfg["joints"].get(joint, {}).get("type") == "prismatic", joint
        assert float(cw["mass"]) >= 0.0, joint


# --- config/hardware.yaml: only the real plugin may see it -----------------

HARDWARE = PKG / "config" / "hardware.yaml"


@pytest.mark.parametrize("args", [{"sim": "true"}, {"sim": "false", "mock": "true"}])
def test_sim_and_mock_do_not_read_hardware_yaml(args):
    # A missing hardware_file must not matter: sim and mock output stays the
    # same byte for byte, whatever hardware.yaml says.
    assert expand(**args, hardware_file="/nonexistent.yaml") == expand(**args)


def test_hardware_yaml_covers_every_ros2_control_joint(cfg):
    hw = yaml.safe_load(HARDWARE.read_text())
    movable = {n for n, j in cfg["joints"].items() if j["type"] != "fixed"}
    assert set(hw["joints"]) == movable
    can_ids = [j["can_id"] for j in hw["joints"].values() if j["mode"] == "can"]
    assert len(can_ids) == len(set(can_ids)), "duplicate can_id in hardware.yaml"
    for name, j in hw["joints"].items():
        assert j["mode"] in ("can", "virtual"), name
        if j["mode"] == "can":
            assert 0 < j["motor_revs_per_unit"], name
            assert 1 <= j["max_motor_rpm"] <= 3000, name
            assert 0 <= j["acceleration"] <= 255, name


def test_real_plugin_gets_hardware_params():
    root = ET.fromstring(expand(sim="false", hardware_file=str(HARDWARE)))
    rc = root.find("ros2_control")
    assert rc.find("hardware/plugin").text == "canopyag_hardware/CanopyagSystem"
    for joint in rc.findall("joint"):
        params = {p.get("name") for p in joint.findall("param")}
        assert "mode" in params, joint.get("name")
