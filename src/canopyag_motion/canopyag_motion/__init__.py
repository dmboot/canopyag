"""canopyag_motion: move the robot to a world point (or joint values) as fast
as its limits allow, all joints at once, clear of its own collision boxes.

    model.py     kinematics, inverse kinematics, collision boxes
    profiles.py  time-optimal motion of one joint
    planner.py   all joints together: waits, elbow choice, crate gap
"""

import yaml

from .model import ARM, CRATE, JOINTS, WRIST, Model, ModelError  # noqa: F401
from .planner import Plan, PlanError, Planner                  # noqa: F401


def load(robot_parameters, motion_config):
    """(Model, Planner, motion config dict) from the two yaml files."""
    with open(robot_parameters) as f:
        params = yaml.safe_load(f)
    with open(motion_config) as f:
        cfg = yaml.safe_load(f)
    col, crate = cfg["collision"], cfg["crate"]
    model = Model(params, cfg["tool"]["link"], cfg["tool"]["xyz"], col["margin"],
                  col.get("ignore", []))
    limits = {j: (L["velocity"], L["acceleration"]) for j, L in cfg["limits"].items()}
    planner = Planner(model, limits, gap=crate["gap"], crate_trail=crate["trail"],
                      crate_follow=crate["follow"], check_step=col["step"])
    return model, planner, cfg
