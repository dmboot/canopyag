"""hardware.launch.py + canopyag_hardware against tools/mks_sim.py on vcan0.

One launch per case: mks_sim.py drives CAN IDs 1 (z_carriage) and 3
(joint_1), with at most one fault flag. The healthy cases must bring the
controllers up as on the real robot and follow forward_position_controller
commands; the fault cases must end with the hardware finalized, a FAULT
message naming the joint, and an F7 seen on the bus for every motor.

Needs vcan0 (README "Testing on vcan0"); without it every case is skipped.

    colcon test --packages-select canopyag_bringup --event-handlers console_direct+
"""

import os
import socket
import struct
import threading
import time
import unittest
from pathlib import Path

import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SwitchController,
)
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

IFACE = "vcan0"
HERE = Path(__file__).resolve().parent
FORWARD_JOINTS = ["z_carriage", "joint_1", "joint_2"]
TARGET = [0.05, 0.4, 0.3]     # z_carriage m, joint_1..2 rad
MOTOR_IDS = (1, 3)

# case -> (mks_sim.py flags, expected outcome, text the FAULT/FATAL log must contain)
CASES = {
    "nominal":         ([], "track", None),
    "latency_20ms":    (["--latency", "20"], "track", None),
    "no_done":         (["--no-done"], "track", None),
    "retarget_ignore": (["--retarget", "ignore"], "track", None),
    "retarget_reject": (["--retarget", "reject"], "fault", "REJECTED"),
    "drop":            (["--drop", "3", "--fault-after", "6"], "fault", "'joint_1' (CAN ID 3) has not answered"),
    "stall":           (["--stall", "1", "--fault-after", "6"], "fault", "'z_carriage' (CAN ID 1): STALL"),
    "duplicate_id":    (["--duplicate-id", "3"], "no_configure", "answered 2 times"),
}


def vcan_up():
    return os.path.exists(f"/sys/class/net/{IFACE}")


class Sniffer:
    """Records every frame on the bus, to check which motors got F7."""

    FRAME = struct.Struct("=IB3x8s")

    def __init__(self):
        self.frames, self.lock = [], threading.Lock()
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((IFACE,))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                can_id, dlc, data = self.FRAME.unpack(self.sock.recv(16))
            except socket.timeout:
                continue
            except OSError:
                return
            with self.lock:
                self.frames.append((can_id & 0x7FF, bytes(data[:dlc])))

    def ids_that_got(self, cmd):
        with self.lock:
            return {i for i, d in self.frames if d and d[0] == cmd}


@launch_testing.parametrize("case", list(CASES))
def generate_test_description(case):
    if not vcan_up():
        return LaunchDescription([launch_testing.actions.ReadyToTest()]), {"sniffer": None}

    sniffer = Sniffer()
    sim = ExecuteProcess(
        cmd=["python3",
             os.path.join(get_package_prefix("canopyag_hardware"), "lib", "canopyag_hardware",
                          "mks_sim.py"),
             "--iface", IFACE, "--ids", *map(str, MOTOR_IDS), *CASES[case][0]],
        output="screen",
    )
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("canopyag_bringup"), "launch", "hardware.launch.py")),
        launch_arguments={"can_interface": IFACE,
                          "hardware_file": str(HERE / "hardware_vcan.yaml")}.items(),
    )
    return LaunchDescription([
        sim,
        TimerAction(period=1.0, actions=[hardware]),
        launch_testing.actions.ReadyToTest(),
    ]), {"sniffer": sniffer}


class TestVcan(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_vcan")
        cls.joint_state = None
        cls.node.create_subscription(JointState, "/joint_states",
                                     lambda m: setattr(cls, "joint_state", m), 10)
        cls.pub = cls.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---- helpers ----
    def call(self, srv_type, name, request, timeout=10.0):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), f"{name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), f"{name} timed out")
        return future.result()

    def controllers(self):
        res = self.call(ListControllers, "/controller_manager/list_controllers",
                        ListControllers.Request())
        return {c.name: c.state for c in res.controller}

    def hardware_state(self):
        res = self.call(ListHardwareComponents, "/controller_manager/list_hardware_components",
                        ListHardwareComponents.Request())
        return res.component[0].state.label

    def wait_for(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            ok, last = predicate()
            if ok:
                return last
        self.fail(f"timed out waiting for {what}; last: {last}")

    def wait_controllers_up(self):
        want = {"joint_state_broadcaster": "active", "arm_controller": "inactive",
                "crate_controller": "active", "forward_position_controller": "inactive"}

        def check():
            try:
                have = self.controllers()
            except AssertionError:
                return False, None
            return have == want, have
        self.wait_for(check, 40.0, "controllers active/inactive as on the real robot")

    def activate_forward(self):
        req = SwitchController.Request()
        req.activate_controllers = ["forward_position_controller"]
        req.strictness = SwitchController.Request.STRICT
        self.assertTrue(self.call(SwitchController, "/controller_manager/switch_controller", req).ok)

    def command_ramp(self, steps=30, dt=0.05):
        """Stream a ramp to TARGET: every new F5 arrives while the last one is running."""
        for k in range(1, steps + 1):
            self.pub.publish(Float64MultiArray(data=[t * k / steps for t in TARGET]))
            end = time.monotonic() + dt
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.01)

    def assert_fault(self, proc_output, sniffer, text):
        proc_output.assertWaitFor(text, timeout=30)
        self.wait_for(lambda: ((s := self.hardware_state()) == "finalized", s), 15.0,
                      "hardware component to be finalized")
        time.sleep(0.5)
        self.assertEqual(sniffer.ids_that_got(0xF7), set(MOTOR_IDS), "F7 not sent to every motor")

    # ---- the test ----
    def test_case(self, proc_output, case, sniffer):
        if sniffer is None:
            self.skipTest(f"{IFACE} missing: sudo modprobe vcan && sudo ip link add dev vcan0 "
                          "type vcan && sudo ip link set vcan0 up")
        _, expect, text = CASES[case]

        if expect == "no_configure":
            # Humble's ros2_control_node exits when the startup configure fails.
            proc_output.assertWaitFor(text, timeout=30)
            proc_output.assertWaitFor("Failed to 'configure' hardware 'canopyag'", timeout=10)
            time.sleep(0.5)
            self.assertEqual(sniffer.ids_that_got(0xF5), set(), "a motor was moved")
            self.assertEqual(sniffer.ids_that_got(0xF7), set(MOTOR_IDS), "F7 not sent to every motor")
            return

        self.wait_controllers_up()
        self.assertEqual(self.hardware_state(), "active")
        self.activate_forward()

        if expect == "fault" and case != "retarget_reject":
            # drop / stall: the injected fault fires on its own timer
            self.assert_fault(proc_output, sniffer, text)
            return

        self.command_ramp()
        if expect == "fault":
            self.assert_fault(proc_output, sniffer, text)
            return

        def arrived():
            self.pub.publish(Float64MultiArray(data=TARGET))
            js = self.joint_state
            if js is None:
                return False, None
            pos = dict(zip(js.name, js.position))
            err = {n: abs(pos[n] - t) for n, t in zip(FORWARD_JOINTS, TARGET)}
            return max(err.values()) < 2e-3, err
        self.wait_for(arrived, 15.0, "/joint_states to follow the command")
        self.assertEqual(self.hardware_state(), "active")
        self.assertEqual(sniffer.ids_that_got(0xF7), set(), "unexpected emergency stop")
        # joint_1 is `reversed` in hardware_vcan.yaml: +0.4 rad must be negative motor counts
        f5 = [d for i, d in sniffer.frames if i == 3 and d[0] == 0xF5 and len(d) == 8]
        last = int.from_bytes(f5[-1][4:7], "big", signed=True)
        self.assertLess(last, 12345, "reversed joint moved the motor the wrong way")
