#!/usr/bin/env python3
"""
mks_sim.py  --  simulated MKS SERVO57D drivers on a SocketCAN interface
=======================================================================

Stand-in for the real bus, so canopyag_hardware can be tested with no motor
attached. Python standard library only.

    python3 mks_sim.py --ids 1 3                       # two healthy drivers on vcan0
    python3 mks_sim.py --ids 1 3 --drop 3 --fault-after 5
    python3 mks_sim.py --ids 1 3 --retarget reject     # driver refuses F5 mid-move

Every command the plugin uses is answered with a correct CRC (see the table
in the README): F3 F4 F5 F6 F7 31 32 3A 3D 3E 92. Motion follows the MKS
acceleration rule (1 rpm every (256 - acc) * 50 us, acc 0 = instant) with
the commanded speed as the limit, and F4/F5 reply status 1 on start and 2 on
arrival.

Fault injection (times are counted from that driver's first enable, F3 1):
  --drop ID          driver goes silent after --fault-after seconds
  --stall ID         stall protection trips after --fault-after seconds
                     (0x3E reads 1, moves are refused until 0x3D)
  --duplicate-id ID  two drivers answer on ID (positions differ by a few counts)
  --no-done          F4/F5 never send status 2
  --latency MS       every reply is delayed by MS milliseconds
  --retarget MODE    what a new F5 does while a move is running - the open
                     hardware question (README, "F5 on the real drivers"):
                       smooth  switch to the new target on the fly (default)
                       ignore  reply 1 but keep going to the old target
                       reject  reply status 0
"""

import argparse
import heapq
import math
import select
import socket
import struct
import sys
import time

COUNTS_PER_REV = 16384
FRAME = struct.Struct("=IB3x8s")


def crc(can_id, payload):
    return (can_id + sum(payload)) & 0xFF


def i24(b):
    v = int.from_bytes(bytes(b), "big")
    return v - (1 << 24) if v & 0x800000 else v


class Driver:
    def __init__(self, can_id, start_counts, args, tag=""):
        self.id = can_id
        self.tag = tag
        self.args = args
        self.pos = float(start_counts)     # encoder counts
        self.vel = 0.0                     # counts/s
        self.mode = "idle"                 # idle | pos | speed
        self.target = self.pos
        self.rpm = 0
        self.acc = 0
        self.speed_target = 0.0            # counts/s, speed mode
        self.move_cmd = None               # F4/F5 of the running move
        self.enabled = False
        self.stalled = False
        self.silent = False
        self.enabled_at = None
        self.axis_zero = 0.0               # 0x92 shifts reported and F5 coordinates

    # ---- motion ----
    def accel(self):
        """counts/s^2 for the current acc, or None for 'instant'."""
        if self.acc == 0:
            return None
        rpm_per_s = 1.0 / ((256 - self.acc) * 50e-6)
        return rpm_per_s * COUNTS_PER_REV / 60.0

    def _approach(self, want, dt):
        a = self.accel()
        if a is None:
            self.vel = want
        else:
            step = a * dt
            self.vel += max(-step, min(step, want - self.vel))

    def step(self, dt, now, out):
        if self.enabled_at is not None and not (self.silent or self.stalled):
            if now - self.enabled_at >= self.args.fault_after:
                if self.id in self.args.drop:
                    self.silent = True
                    log(f"id {self.id}{self.tag}: DROPPED OFF THE BUS (injected)")
                if self.id in self.args.stall:
                    self.stalled = True
                    self.mode, self.vel, self.move_cmd = "idle", 0.0, None
                    log(f"id {self.id}{self.tag}: STALL PROTECTION TRIPPED (injected)")

        if self.mode == "pos":
            vmax = self.rpm * COUNTS_PER_REV / 60.0
            err = self.target - self.pos
            a = self.accel()
            reach = vmax if a is None else min(vmax, math.sqrt(2.0 * a * abs(err)))
            self._approach(math.copysign(reach, err), dt)
            nxt = self.pos + self.vel * dt
            if (self.target - nxt) * err <= 0 or abs(err) < 0.5:   # arrived / crossed
                self.pos, self.vel, self.mode = self.target, 0.0, "idle"
                if self.move_cmd is not None and not self.args.no_done:
                    self.reply(out, now, [self.move_cmd, 2])
                self.move_cmd = None
            else:
                self.pos = nxt
        elif self.mode == "speed":
            self._approach(self.speed_target, dt)
            self.pos += self.vel * dt
            if self.speed_target == 0.0 and self.vel == 0.0:
                self.mode = "idle"

    # ---- protocol ----
    def reply(self, out, now, body):
        if self.silent:
            return
        body = bytes(body)
        out.push(now + self.args.latency / 1000.0, self.id, body + bytes([crc(self.id, body)]))

    def handle(self, data, now, out):
        cmd, p = data[0], data[1:-1]
        rep = lambda *b: self.reply(out, now, [cmd, *b])   # noqa: E731
        reported = int(round(self.pos - self.axis_zero))

        if cmd == 0xF3:
            self.enabled = bool(p[0]) if p else False
            if self.enabled and self.enabled_at is None:
                self.enabled_at = now
            if not self.enabled:
                self.mode, self.vel = "idle", 0.0
            rep(1)
        elif cmd in (0xF4, 0xF5):
            rpm, acc, counts = (p[0] << 8) | p[1], p[2], i24(p[3:6])
            if not self.enabled or self.stalled or rpm > 3000:
                rep(0)
                return
            moving = self.mode == "pos"
            if moving and self.args.retarget == "reject":
                rep(0)
                return
            if moving and self.args.retarget == "ignore":
                rep(1)
                return
            target = (self.pos + counts) if cmd == 0xF4 else (counts + self.axis_zero)
            self.target, self.rpm, self.acc = float(target), rpm, acc
            self.move_cmd = cmd
            self.mode = "pos"
            rep(1)
        elif cmd == 0xF6:
            rpm = ((p[0] & 0x0F) << 8) | p[1]
            if not self.enabled or self.stalled:
                rep(0)
                return
            self.acc = p[2]
            self.speed_target = (-1 if p[0] & 0x80 else 1) * rpm * COUNTS_PER_REV / 60.0
            self.mode, self.move_cmd = "speed", None
            rep(1)
        elif cmd == 0xF7:
            self.mode, self.vel, self.move_cmd = "idle", 0.0, None
            log(f"id {self.id}{self.tag}: EMERGENCY STOP (F7) at {reported} counts")
            rep(1)
        elif cmd == 0x31:
            rep(*(reported & 0xFFFFFFFFFFFF).to_bytes(6, "big"))
        elif cmd == 0x32:
            rpm = int(round(self.vel * 60.0 / COUNTS_PER_REV))
            rep(*(rpm & 0xFFFF).to_bytes(2, "big"))
        elif cmd == 0x3A:
            rep(1 if self.enabled else 0)
        elif cmd == 0x3E:
            rep(1 if self.stalled else 0)
        elif cmd == 0x3D:
            self.stalled = False
            rep(1)
        elif cmd == 0x92:
            self.axis_zero = self.pos
            rep(1)


class Outbox:
    """Replies waiting for their (latency-delayed) send time."""

    def __init__(self):
        self.q, self.n = [], 0

    def push(self, when, can_id, data):
        self.n += 1
        heapq.heappush(self.q, (when, self.n, can_id, data))

    def flush(self, sock, now):
        while self.q and self.q[0][0] <= now:
            _, _, can_id, data = heapq.heappop(self.q)
            try:
                sock.send(FRAME.pack(can_id, len(data), data.ljust(8, b"\x00")))
            except OSError as e:          # vcan never fills, but be safe
                log(f"send failed: {e}")


def log(msg):
    print(f"[mks_sim {time.monotonic():.3f}] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 3])
    ap.add_argument("--start-counts", type=int, default=12345,
                    help="encoder value at 'power-up' (non-zero, to exercise the zero offset)")
    ap.add_argument("--drop", type=int, action="append", default=[])
    ap.add_argument("--stall", type=int, action="append", default=[])
    ap.add_argument("--duplicate-id", type=int, action="append", default=[])
    ap.add_argument("--fault-after", type=float, default=3.0)
    ap.add_argument("--no-done", action="store_true")
    ap.add_argument("--latency", type=float, default=0.0, help="ms")
    ap.add_argument("--retarget", choices=["smooth", "ignore", "reject"], default="smooth")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every frame")
    args = ap.parse_args()

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.iface,))
    except OSError as e:
        sys.exit(f"cannot open {args.iface}: {e}\n"
                 "  sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && "
                 "sudo ip link set vcan0 up")
    sock.setblocking(False)

    drivers = {}
    for i in args.ids:
        drivers.setdefault(i, []).append(Driver(i, args.start_counts, args))
    for i in args.duplicate_id:
        drivers.setdefault(i, []).append(Driver(i, args.start_counts + 3, args, tag="(dup)"))

    log(f"{args.iface}: drivers {sorted(drivers)} retarget={args.retarget} "
        f"drop={args.drop} stall={args.stall} dup={args.duplicate_id} "
        f"no_done={args.no_done} latency={args.latency}ms")

    out = Outbox()
    last = time.monotonic()
    try:
        while True:
            select.select([sock], [], [], 0.001)
            now = time.monotonic()
            while True:
                try:
                    raw = sock.recv(16)
                except BlockingIOError:
                    break
                can_id, dlc, data = FRAME.unpack(raw)
                if can_id & (socket.CAN_EFF_FLAG | socket.CAN_ERR_FLAG | socket.CAN_RTR_FLAG):
                    continue
                data = data[:dlc]
                if args.verbose:
                    log(f"RX {can_id:03X}#{data.hex(' ')}")
                if dlc < 2 or crc(can_id, data[:-1]) != data[-1]:
                    continue
                for d in drivers.get(can_id, []):
                    d.handle(data, now, out)
            dt, last = now - last, now
            for ds in drivers.values():
                for d in ds:
                    d.step(dt, now, out)
            out.flush(sock, now)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
