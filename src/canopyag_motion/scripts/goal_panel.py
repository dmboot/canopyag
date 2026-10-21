#!/usr/bin/env python3
"""A small window that sends the tool point to an (x, y, z) in the world.

    ros2 run canopyag_motion goal_panel.py
    ros2 launch canopyag_motion demo.launch.py demo:=false panel:=true    # mock + server + this
    ros2 launch canopyag_motion motion.launch.py panel:=true              # real robot

One slider per axis, limited to the box around everything the tool can reach
(worked out from robot_parameters.yaml and motion.yaml, like
motion_server.py), and where the gripper points: auto (motion.yaml's
default, normally the side of the goal), +y or -y. While you pick, the point
is planned from where the robot is now, the same way motion_server.py will:
the window says whether it can get there, with which joints and how fast,
and RViz shows it on goal_preview (green: reachable, with the path; red: not).

  Go      send the point to motion_server.py (the move_to action)
  Stop    cancel: the robot brakes to a stop
  Random  pick a reachable point anywhere in the box (try long paths)
  Rest    the arm to REST
  Send on release   every slider release / typed value is a Go: new goals
                    take over from the running motion

The crate slider is separate from the point: on release (or a typed value)
it sends only the crate there, the arm carries on. It has to stay
motion.yaml crate.gap below the carriage.
"""

import math
import os
import random
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from canopyag_interfaces.action import MoveTo
from geometry_msgs.msg import Point
from python_qt_binding.QtCore import QObject, Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
                                         QGridLayout, QHBoxLayout, QLabel, QPushButton,
                                         QSlider, QVBoxLayout, QWidget)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

import canopyag_motion
from canopyag_motion import CRATE, JOINTS, WRIST, PlanError

REST = [0.05, 0.0, 0.0]    # z_carriage, joint_1, joint_2 (as demo.py)
GREEN, RED, GREY = "#2a9d3a", "#c0392b", "#777777"


class PanelNode(Node):

    def __init__(self):
        super().__init__("goal_panel")
        desc = get_package_share_directory("canopyag_description")
        here = get_package_share_directory("canopyag_motion")
        params_file = self.declare_parameter(
            "robot_parameters", os.path.join(desc, "config", "robot_parameters.yaml")).value
        config_file = self.declare_parameter(
            "motion_config", os.path.join(here, "config", "motion.yaml")).value
        self.frame = self.declare_parameter("frame", "world").value
        self.model, self.planner, _ = canopyag_motion.load(params_file, config_file)
        self.measured = {}
        self.create_subscription(JointState, "joint_states", self.on_joint_states,
                                 qos_profile_sensor_data)
        self.preview_pub = self.create_publisher(MarkerArray, "goal_preview", 10)
        self.client = ActionClient(self, MoveTo, "move_to")

    def on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                self.measured[name] = pos

    def start_state(self):
        """joint -> (position, 0) where the robot is now, or None."""
        m = dict(self.measured)
        if any(j not in m for j in JOINTS):
            return None
        return {j: (m[j], 0.0) for j in JOINTS}

    def reachable(self, point, heading=0):
        """Within reach (joint limits, not collisions or the way there)."""
        psi = self.planner.heading(heading, point[1], self.start_state())
        return bool(self.model.inverse(point, psi))

    def plan(self, point, speed=1.0, heading=0):
        """(Plan or None, message): what motion_server.py would do from here."""
        start = self.start_state()
        if start is None:
            if self.reachable(point, heading):
                return None, "within reach (no /joint_states yet: not planned)"
            return None, "out of reach"
        try:
            return self.planner.to_point(start, point, speed, heading), ""
        except PlanError as e:
            return None, str(e)

    def publish_preview(self, point, plan, reachable):
        stamp = self.get_clock().now().to_msg()
        goal = Marker(type=Marker.SPHERE, ns="preview", id=0, action=Marker.ADD)
        goal.pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.045
        goal.color.r, goal.color.g, goal.color.b = (0.2, 0.85, 0.3) if reachable else (0.9, 0.15, 0.1)
        goal.color.a = 0.6
        path = Marker(type=Marker.LINE_STRIP, ns="preview_path", id=0)
        if plan is not None and plan.duration > 1e-6:
            s = plan.sample(np.linspace(0.0, plan.duration, 60))
            tool = self.model.tool_position({j: s[j][0] for j in JOINTS})
            path.action = Marker.ADD
            path.scale.x = 0.003
            path.color.r, path.color.g, path.color.b, path.color.a = 0.2, 0.85, 0.3, 0.6
            path.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in tool.T]
        else:
            path.action = Marker.DELETE
        for m in (goal, path):
            m.header.frame_id = self.frame
            m.header.stamp = stamp
        self.preview_pub.publish(MarkerArray(markers=[goal, path]))


def moved(a, b):
    """Largest change of any joint between two joint -> position dicts."""
    return max(abs(a[j] - b[j]) for j in JOINTS)


class Bridge(QObject):
    """Moves action results from the ROS thread to the Qt thread."""
    status = Signal(str, str)     # text, colour


class Axis:
    """A slider and a spin box for one coordinate, in step with each other."""

    def __init__(self, grid, row, name, lo, hi, changed, released):
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(round(lo * 1000)), int(round(hi * 1000)))
        self.spin = QDoubleSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setDecimals(3)
        self.spin.setSingleStep(0.01)
        self.spin.setSuffix(" m")
        self.spin.setKeyboardTracking(False)
        grid.addWidget(QLabel(f"<b>{name}</b>"), row, 0)
        grid.addWidget(self.slider, row, 1)
        grid.addWidget(self.spin, row, 2)
        grid.addWidget(QLabel(f"<small>{lo:.2f} … {hi:.2f}</small>"), row, 3)
        self.slider.valueChanged.connect(lambda v: self._set(self.spin, v / 1000.0, changed))
        self.spin.valueChanged.connect(lambda v: self._set(self.slider, int(round(v * 1000)), changed))
        self.slider.sliderReleased.connect(released)
        self.spin.editingFinished.connect(released)

    @staticmethod
    def _set(widget, value, changed):
        widget.blockSignals(True)
        widget.setValue(value)
        widget.blockSignals(False)
        changed()

    def value(self):
        return self.spin.value()

    def set_value(self, v):
        self.spin.setValue(v)     # moves the slider too


class Panel(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.handle = None
        self.touched = False
        self.last_q = None          # robot at the last tick
        self.preview_q = None       # robot when the preview was planned
        self.bridge = Bridge()
        self.bridge.status.connect(self.show_status)
        self.setWindowTitle("canopyag goal")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)

        lo, hi = node.model.reach_box()
        grid = QGridLayout()
        self.axes = [Axis(grid, i, name, lo[i], hi[i], self.on_change, self.on_release)
                     for i, name in enumerate("xyz")]
        self.speed = QSlider(Qt.Horizontal)
        self.speed.setRange(5, 100)
        self.speed.setValue(30)
        speed_label = QLabel()
        self.speed.valueChanged.connect(lambda v: speed_label.setText(f"{v} %"))
        self.speed.valueChanged.connect(self.on_change)
        speed_label.setText(f"{self.speed.value()} %")
        grid.addWidget(QLabel("<b>speed</b>"), 3, 0)
        grid.addWidget(self.speed, 3, 1)
        grid.addWidget(speed_label, 3, 2)
        self.heading = QComboBox()
        for text, value in (("auto (side of the goal)", MoveTo.Goal.HEADING_DEFAULT),
                            ("+y", MoveTo.Goal.HEADING_POS_Y), ("-y", MoveTo.Goal.HEADING_NEG_Y)):
            self.heading.addItem(text, value)
        self.heading.currentIndexChanged.connect(self.on_change)
        grid.addWidget(QLabel("<b>gripper to</b>"), 4, 0)
        grid.addWidget(self.heading, 4, 1)
        # Not part of the point: moves only the crate, as soon as it is let go.
        self.crate = Axis(grid, 5, "crate", *node.model.limits(CRATE), lambda: None,
                          self.send_crate)
        self.crate_synced = False
        self.crate_sent = -math.inf     # time.monotonic() of the last crate goal
        grid.setColumnStretch(1, 1)

        buttons = QHBoxLayout()
        for text, slot in (("Go", self.go), ("Stop", self.stop),
                           ("Random", self.pick_random), ("Rest", self.rest)):
            b = QPushButton(text)
            b.clicked.connect(slot)
            buttons.addWidget(b)
        self.live = QCheckBox("Send on release")
        buttons.addWidget(self.live)

        self.preview_label = QLabel()
        self.status_label = QLabel()
        self.robot_label = QLabel()
        for label in (self.preview_label, self.status_label):
            label.setWordWrap(True)
        self.robot_label.setStyleSheet(f"color: {GREY}")

        box = QVBoxLayout(self)
        box.addLayout(grid)
        box.addLayout(buttons)
        box.addWidget(self.preview_label)
        box.addWidget(self.status_label)
        box.addWidget(self.robot_label)
        self.resize(520, 0)

        self.debounce = QTimer(self, singleShot=True, interval=100, timeout=self.preview)
        self.ticker = QTimer(self, interval=200, timeout=self.tick)
        self.ticker.start()
        for axis, v in zip(self.axes, (lo + hi) / 2):
            axis.set_value(v)
        self.touched = False
        self.show_status("motion_server.py: no goal sent yet", GREY)

    # ------------------------------------------------------------------ inputs

    def point(self):
        return np.array([a.value() for a in self.axes])

    def on_change(self):
        self.touched = True
        self.debounce.start()

    def on_release(self):
        if self.live.isChecked():
            self.go()

    def tick(self):
        start = self.node.start_state()
        if start is None:
            self.robot_label.setText("robot: waiting for /joint_states")
            return
        q = {j: p for j, (p, _) in start.items()}
        tool = self.node.model.tool_position(q)
        self.robot_label.setText(
            f"robot: tool at ({tool[0]:.3f}, {tool[1]:.3f}, {tool[2]:.3f}), joints "
            f"{q['z_carriage']:.3f} m, {math.degrees(q['joint_1']):.1f}°, "
            f"{math.degrees(q['joint_2']):.1f}°, {math.degrees(q[WRIST]):.1f}°, "
            f"crate {q[CRATE]:.3f} m")
        if not self.touched:
            # Start where the robot is.
            for axis, v in zip(self.axes, tool):
                axis.set_value(float(v))
            self.touched = True
        # The robot came to rest somewhere else: the preview starts from there now.
        at_rest = self.last_q is not None and moved(self.last_q, q) < 1e-4
        # The crate slider shows where the crate is, once it stands still
        # (an arm move may have pushed it down, a goal been refused), unless
        # it is being moved or a crate goal has only just gone out.
        crate_busy = (self.crate.slider.isSliderDown() or self.crate.spin.hasFocus() or
                      time.monotonic() - self.crate_sent < 0.5)
        if not self.crate_synced or (at_rest and not crate_busy and
                                     abs(self.crate.value() - q[CRATE]) > 1e-3):
            self.crate.set_value(q[CRATE])
            self.crate_synced = True
        if at_rest and self.preview_q is not None and moved(self.preview_q, q) > 1e-3:
            self.preview()
        self.last_q = q

    def preview(self):
        p = self.point()
        start = self.node.start_state()
        self.preview_q = None if start is None else {j: v for j, (v, _) in start.items()}
        plan, why = self.node.plan(p, self.speed.value() / 100.0, self.heading.currentData())
        reachable = plan is not None or why.startswith("within reach")
        self.node.publish_preview(p, plan, reachable)
        if plan is None:
            colour = GREEN if reachable else RED
            self.preview_label.setText(f"<span style='color:{colour}'>{why}</span>")
            return
        notes = f" ({', '.join(plan.notes)})" if plan.notes else ""
        self.preview_label.setText(
            f"<span style='color:{GREEN}'>reachable</span>: joints {plan.goal[0]:.3f} m, "
            f"{math.degrees(plan.goal[1]):.1f}°, {math.degrees(plan.goal[2]):.1f}°, "
            f"{math.degrees(plan.goal[3]):.1f}°; "
            f"{plan.duration:.2f} s from where the robot is now{notes}")

    # ------------------------------------------------------------------ actions

    def go(self):
        p = self.point()
        self.send(MoveTo.Goal(point=Point(x=p[0], y=p[1], z=p[2])),
                  f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")

    def rest(self):
        self.send(MoveTo.Goal(joints=[float(v) for v in REST]), "rest")

    def send_crate(self):
        v = self.crate.value()
        self.crate_sent = time.monotonic()
        self.send(MoveTo.Goal(crate=[v], crate_only=True), f"crate to {v:.3f} m")

    def send(self, goal, what):
        if not self.node.client.server_is_ready():
            self.show_status("no move_to action server: is motion_server.py running? "
                             "(ros2 launch canopyag_motion motion.launch.py)", RED)
            return
        goal.speed = self.speed.value() / 100.0
        goal.heading = self.heading.currentData()
        self.show_status(f"{what}: sent", GREY)
        future = self.node.client.send_goal_async(
            goal, feedback_callback=lambda fb: self.bridge.status.emit(
                f"{what}: moving, {fb.feedback.time_left:.1f} s to go", GREY))
        future.add_done_callback(lambda f: self.on_accepted(f, what))

    def on_accepted(self, future, what):        # ROS thread
        handle = future.result()
        if not handle.accepted:
            self.bridge.status.emit(f"{what}: rejected", RED)
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, handle, what))

    def on_result(self, future, handle, what):  # ROS thread
        if handle is not self.handle:
            return      # an older goal, replaced by a newer one
        r = future.result().result
        if r.success:
            self.bridge.status.emit(f"{what}: {r.message} ({r.duration:.2f} s planned)", GREEN)
        else:
            self.bridge.status.emit(f"{what}: {r.message}", RED)

    def stop(self):
        if self.handle is not None:
            self.handle.cancel_goal_async()
            self.show_status("stopping", GREY)

    def pick_random(self):
        """A point anywhere in the box that motion_server.py can plan to."""
        lo = np.array([a.spin.minimum() for a in self.axes])
        hi = np.array([a.spin.maximum() for a in self.axes])
        for _ in range(500):
            p = lo + (hi - lo) * np.array([random.random() for _ in range(3)])
            # Cheap check first: within reach. Planning is the slow part.
            heading = self.heading.currentData()
            if not self.node.reachable(p, heading):
                continue
            plan, _ = self.node.plan(p, 1.0, heading)
            if plan is not None or self.node.start_state() is None:
                for axis, v in zip(self.axes, p):
                    axis.set_value(round(float(v), 3))
                self.preview()
                if self.live.isChecked():
                    self.go()
                return
        self.show_status("Random: no reachable point found", RED)

    def show_status(self, text, colour):
        self.status_label.setText(f"<span style='color:{colour}'>{text}</span>")


def main():
    rclpy.init()
    node = PanelNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    app = QApplication(sys.argv[:1])
    panel = Panel(node)
    panel.show()
    # Qt never returns to Python on its own: wake it so Ctrl+C gets through.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    wake = QTimer(interval=200, timeout=lambda: None)
    wake.start()
    code = app.exec_()
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
