"""Manual driving and example waypoint autonomy for the M01 Webots prototype."""
import json
import math
import sys
from pathlib import Path

from controller import Keyboard, Supervisor

WHEEL_RADIUS = 0.032
TRACK_WIDTH = 0.188
AXLE_X = 0.014
LIFT_MIN = 0.026
LIFT_MAX = 0.272
JAW_TRAVEL = 0.034
SPEED_LIMIT = 0.52
HOOK_SEAT = 0.181
HOOK_TRAVEL = 0.060
HOOK_CLEARANCE = 0.211
TRIDENT_REACH = 0.202
# Hanging cubes: Webots DEF name -> (offset from the field centre on each axis, centre height, long hanger).
HOOK_CUBES = {
    "BLUE_2": (-0.153, 0.149, True), "BLUE_3": (-0.086, 0.167, False),
    "RED_2": (0.153, 0.149, True), "RED_3": (0.086, 0.167, False),
}


def limit(value, lower, upper):
    return max(lower, min(upper, value))


def angle_error(target, current):
    return (target - current + math.pi) % (2 * math.pi) - math.pi


class Driver:
    def __init__(self):
        self.robot = Supervisor()
        self.step_ms = int(self.robot.getBasicTimeStep())
        self.wheels = [self.robot.getDevice(f"{side} wheel") for side in ("left", "right")]
        self.encoders = [self.sensor(f"{side} encoder") for side in ("left", "right")]
        for motor in self.wheels:
            motor.setPosition(float("inf"))
            motor.setVelocity(0)
        self.lift = self.robot.getDevice("lift")
        self.lift.setVelocity(0.21)
        self.lift_encoder = self.sensor("lift encoder")
        self.jaws = [self.robot.getDevice(f"{side} jaw") for side in ("left", "right")]
        self.jaw_encoders = [self.sensor(f"{side} jaw encoder") for side in ("left", "right")]
        for jaw in self.jaws:
            jaw.setVelocity(0.10)
        self.gps = self.sensor("gps")
        self.imu = self.sensor("imu")
        self.gyro = self.sensor("gyro")
        self.camera = self.robot.getDevice("front camera")
        self.hook = self.robot.getDevice("rail hook")
        self.hook.enablePresence(self.step_ms)
        self.hook_state = None
        self.hook_started = 0.0
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.step_ms)
        self.closed = False
        self.move_action = None
        self.move_origin = (0.0, 0.0)
        self.arrived = None
        self.height = LIFT_MIN
        self.set_height(self.height)
        self.grip(False)

    def sensor(self, name):
        device = self.robot.getDevice(name)
        device.enable(self.step_ms)
        return device

    def pose(self):
        x, y, z = self.gps.getValues()
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        return x, y, z, roll, pitch, yaw

    def drive(self, linear, angular):
        speeds = [
            (linear - angular * TRACK_WIDTH / 2) / WHEEL_RADIUS,
            (linear + angular * TRACK_WIDTH / 2) / WHEEL_RADIUS,
        ]
        scale = max(1, max(map(abs, speeds)) / (SPEED_LIMIT / WHEEL_RADIUS))
        for motor, speed in zip(self.wheels, speeds):
            motor.setVelocity(speed / scale)

    def set_height(self, height):
        self.height = limit(height, LIFT_MIN, LIFT_MAX)
        self.lift.setPosition(round(self.height - LIFT_MIN, 6))

    def grip(self, closed):
        self.closed = closed
        for jaw in self.jaws:
            jaw.setPosition(-JAW_TRAVEL if closed else 0)

    def hook_height(self):
        return self.lift_encoder.getValue() + LIFT_MIN

    def toggle_hook(self):
        if self.hook_state == "seat":
            self.hook_state = None
            self.set_height(HOOK_CLEARANCE)
        elif self.hook_state:
            self.hook_state = "release"
            self.set_height(HOOK_SEAT)
        elif self.closed or self.hook.getPresence() != 1:
            return False
        else:
            self.drive(0, 0)
            self.hook_started = self.robot.getTime()
            self.hook_state = "seat"
            self.set_height(HOOK_SEAT)
        return True

    def update_hook(self):
        if self.hook_state == "seat":
            if self.hook.getPresence() == 1 and abs(self.hook_height() - HOOK_SEAT) < .006:
                self.hook.lock()
                self.hook_state = "pull"
                self.set_height(HOOK_SEAT - HOOK_TRAVEL)
            elif self.robot.getTime() - self.hook_started > 2:
                self.hook_state = None
                self.set_height(HOOK_CLEARANCE)
                print("Hook alignment timed out.")
        elif self.hook_state == "release" and abs(self.hook_height() - HOOK_SEAT) < .006:
            self.hook.unlock()
            self.hook_state = None
            self.set_height(HOOK_CLEARANCE)

    def navigate(self, target_x, target_y, heading=None, stop_radius=0.014):
        """Return True once the position and optional final yaw are reached."""
        x, y, _, _, _, yaw = self.pose()
        # Steer by the axle centre: it stays put while the robot turns on the spot, unlike the GPS point.
        x += AXLE_X * math.cos(yaw)
        y += AXLE_X * math.sin(yaw)
        if heading is not None:
            target_x += AXLE_X * math.cos(heading)
            target_y += AXLE_X * math.sin(heading)
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        goal = (target_x, target_y, heading)
        # Turning on the spot shifts the GPS point a little, so the arrival is latched until the yaw settles.
        if distance < stop_radius or (self.arrived == goal and distance < stop_radius + 0.02):
            error = angle_error(heading, yaw) if heading is not None else 0
            if abs(error) < 0.045:
                self.drive(0, 0)
                self.arrived = None
                return True
            self.arrived = goal
            self.drive(0, limit(4 * error, -2.5, 2.5))
            return False
        self.arrived = None
        forward = math.atan2(dy, dx)
        error = angle_error(forward, yaw)
        direction = 1
        if distance < 0.25 and abs(error) > 2.0:
            # A nearby point behind the robot is reached in reverse instead of spinning round.
            error = angle_error(forward + math.pi, yaw)
            direction = -1
        linear = limit(2.6 * distance, 0.046, 0.35) if abs(error) < 0.48 else 0
        self.drive(direction * linear, limit(4.2 * error, -3.0, 3.0))
        return False

    def step(self):
        return self.robot.step(self.step_ms) != -1

    def advance(self, action, elapsed):
        op, *args = action
        if op == "pose":
            return self.navigate(*args)
        if op == "lift":
            self.set_height(args[0])
            return abs(self.lift_encoder.getValue() + LIFT_MIN - self.height) < 0.006
        if op == "move":
            # Straight creep over a signed distance measured by GPS from where the action began.
            x, y = self.gps.getValues()[:2]
            if self.move_action is not action:
                self.move_action, self.move_origin = action, (x, y)
            done = math.hypot(x - self.move_origin[0], y - self.move_origin[1]) >= abs(args[0])
            self.drive(0 if done else math.copysign(0.06, args[0]), 0)
            if done:
                self.move_action = None
            return done
        if op == "wait":
            self.drive(0, 0)
            return elapsed >= args[0]
        if op == "reverse":
            done = elapsed >= args[0]
            self.drive(0 if done else -0.17, 0)
            return done
        self.grip(args[0])
        return elapsed > 0.42

    def tick_for(self, seconds, linear=0, angular=0):
        self.drive(linear, angular)
        steps = round(seconds * 1000 / self.step_ms)
        return all(self.step() for _ in range(steps))

    def status(self):
        x, y, z, roll, pitch, yaw = self.pose()
        return {
            "position_m": [round(v, 3) for v in (x, y, z)],
            "speed_m_s": round(self.gps.getSpeed(), 3),
            "attitude_deg": [round(math.degrees(v), 1) for v in (roll, pitch, yaw)],
            "rotation_deg_s": [round(math.degrees(v), 1) for v in self.gyro.getValues()],
            "encoders_rad": [round(sensor.getValue(), 2) for sensor in self.encoders],
            "lift_m": round(self.lift_encoder.getValue() + LIFT_MIN, 3),
            "jaws_m": [round(sensor.getValue(), 3) for sensor in self.jaw_encoders],
            "hook_present": self.hook.getPresence() == 1,
            "hook_locked": self.hook.isLocked(),
        }


def blue_base_drop():
    return [
        ("pose", -0.34, -0.76, math.pi), ("pose", -0.495, -0.76, math.pi),
        ("grip", False), ("reverse", 0.66), ("lift", LIFT_MIN),
    ]


def demo_steps(lab):
    if lab:
        return [
            ("pose", -0.454, -0.37, 0.0), ("grip", True), ("lift", 0.144),
            ("pose", -0.185, -0.37, 0.0), ("grip", False), ("lift", LIFT_MIN),
        ]
    return [
        ("pose", -0.645, -0.286, 0.0), ("pose", -0.432, -0.286, 0.0),
        ("grip", True), ("lift", 0.150),
        ("pose", -0.42, -0.50), *blue_base_drop(),
    ]


def core_steps():
    return [
        ("pose", -0.56, -0.12), ("pose", -0.56, 0.27),
        ("pose", -0.322, 0.322, -math.pi / 4), ("lift", 0.250),
        ("pose", -0.124, 0.124, -math.pi / 4, 0.006), ("grip", True),
        ("lift", LIFT_MAX), ("reverse", 0.90),
        ("pose", -0.526, 0.22), ("pose", -0.526, -0.44), *blue_base_drop(),
    ]


def rail_point(along, across):
    """Field point from trident coordinates: along the rail and across it (positive on the y > x side)."""
    return (along - across) / math.sqrt(2), (along + across) / math.sqrt(2)


def hanging_cubes(driver):
    """Cubes that still hang on the trident, as (name, offset, height, long)."""
    found = []
    for name, (offset, height, long_hanger) in HOOK_CUBES.items():
        node = driver.robot.getFromDef(name)
        if node is None:
            continue
        x, y, z = node.getPosition()
        if math.hypot(x - offset, y - offset) < 0.02 and abs(z - height) < 0.012:
            found.append((name, offset, height, long_hanger))
    return found


def hook_cube_steps(driver, name=None):
    """Route that takes a hanging cube off the trident.

    The hanger lip only slides off the rail towards the y > x side of the field, and the hang hook of M01
    cannot pass under the rail while the jaws hold a cube. So the cube is always gripped from the open
    side and pulled straight back; from the other side the robot first drives round the end of the trident.
    """
    cubes = hanging_cubes(driver)
    if name:
        cubes = [cube for cube in cubes if cube[0] == name]
    if not cubes:
        return []
    x, y = driver.pose()[:2]
    name, offset, height, long_hanger = min(cubes, key=lambda cube: math.hypot(x - cube[1], y - cube[1]))
    along = offset * math.sqrt(2)
    across = (y - x) / math.sqrt(2)
    steps = [("grip", False)]
    if abs(across) < 0.30:
        steps.append(("move", abs(across) - 0.30))
    if across < 0:
        end = math.copysign(0.62, offset)
        steps += [("lift", LIFT_MIN), ("pose", *rail_point(along, -0.36)), ("pose", *rail_point(end, -0.30)),
                  ("pose", *rail_point(end, 0.30))]
    facing = -math.pi / 4
    # Keep the mast guides clear of the crossbeam next to a short hanger.
    bias = 0 if long_hanger else -math.copysign(0.008, offset)
    return steps + [
        ("pose", *rail_point(along + bias, 0.36), facing), ("lift", height),
        ("pose", *rail_point(along + bias, TRIDENT_REACH), facing, 0.006),
        ("grip", True), ("lift", height + 0.005), ("wait", 0.25), ("move", -0.12), ("lift", 0.10),
    ]


def manual(driver, lab):
    pending = []
    last_keys = set()
    started = 0.0
    camera_enabled = False
    print("M01: WASD/arrows move, Shift precise motion, Q/E lift, Space grip, T cube demo, C camera, I telemetry.")
    if not lab:
        print("Y core demo from reset; U take the nearest hanging cube off the trident;")
        print("H hook/unhook near a blue arrow with jaws open and lift at 211 mm.")
    while driver.step():
        keys = set()
        precise = False
        while (key := driver.keyboard.getKey()) != -1:
            precise = precise or bool(key & Keyboard.SHIFT)
            keys.add(key & 0xFFFF)
        taps = keys - last_keys
        last_keys = keys
        if ord("I") in taps:
            print(json.dumps(driver.status()))
        if ord("C") in taps:
            camera_enabled = not camera_enabled
            if camera_enabled:
                driver.camera.enable(driver.step_ms)
            else:
                driver.camera.disable()
        if ord("T") in taps:
            pending = [] if pending else demo_steps(lab)
            started = driver.robot.getTime()
        if not lab and ord("Y") in taps:
            pending = [] if pending else core_steps()
            started = driver.robot.getTime()
        if not lab and ord("U") in taps:
            pending = [] if pending else hook_cube_steps(driver)
            started = driver.robot.getTime()
            if not pending:
                print("No cube is hanging on the trident.")
        if not lab and ord("H") in taps:
            pending.clear()
            if not driver.toggle_hook():
                print("Open the gripper, align with a blue arrow and raise the lift to about 211 mm.")
        if driver.hook_state:
            pending.clear()
            driver.drive(0, 0)
            driver.update_hook()
            continue
        forward = int(bool(keys & {Keyboard.UP, ord("W")})) - int(bool(keys & {Keyboard.DOWN, ord("S")}))
        left = int(bool(keys & {Keyboard.LEFT, ord("A")})) - int(bool(keys & {Keyboard.RIGHT, ord("D")}))
        if forward or left or ord(" ") in taps or keys & {ord("Q"), ord("E")}:
            pending.clear()
        if pending:
            elapsed = driver.robot.getTime() - started
            if elapsed > 12:
                driver.drive(0, 0)
                pending.clear()
                print("Autonomous action timed out.")
            elif driver.advance(pending[0], elapsed):
                pending.pop(0)
                started = driver.robot.getTime()
            continue
        scale = 0.35 if precise else 1
        driver.drive(forward * SPEED_LIMIT * scale, left * 3.35 * scale)
        if ord(" ") in taps:
            driver.grip(not driver.closed)
        lift_direction = int(ord("E") in keys) - int(ord("Q") in keys)
        if lift_direction:
            driver.set_height(driver.height + lift_direction * 0.19 * scale * driver.step_ms / 1000)


def smoke(driver):
    """Exercise Webots contacts and sensors in the physics lab."""
    driver.tick_for(0.3)
    cube = driver.robot.getFromDef("LAB_CUBE")
    outcomes = {}
    results = {}
    start_x = driver.pose()[0]
    for operation, *args in demo_steps(True)[:1]:
        assert operation == "pose"
        for _ in range(round(5 * 1000 / driver.step_ms)):
            if driver.navigate(*args):
                break
            if not driver.step():
                return
    driver.tick_for(0.15)
    approach_x = driver.pose()[0]
    outcomes["drive"] = approach_x > start_x + 0.22 and abs(approach_x + 0.454) < 0.032
    results["drive_m"] = round(approach_x - start_x, 3)
    results["approach"] = driver.status()
    driver.grip(True)
    driver.tick_for(0.5)
    results["closed"] = driver.status()
    driver.set_height(0.144)
    driver.tick_for(1)
    raised = cube.getPosition()
    driver.tick_for(1.2, 0.20)
    driver.tick_for(0.2)
    carried = cube.getPosition()
    outcomes["grip"] = raised[2] > 0.095 and carried[2] > 0.095 and carried[0] > raised[0] + 0.10
    results["raised_m"] = round(raised[2], 3)
    results["carry_m"] = round(carried[0] - raised[0], 3)
    driver.grip(False)
    driver.tick_for(0.9)
    outcomes["release"] = abs(cube.getPosition()[2] - 0.026) < 0.022
    results["released_m"] = round(cube.getPosition()[2], 3)
    driver.set_height(0.144)
    yaw = driver.pose()[-1]
    driver.tick_for(0.75, 0, 2.5)
    driver.tick_for(0.20)
    turn = abs(angle_error(driver.pose()[-1], yaw))
    outcomes["turn"] = turn > 0.95
    results["turn_deg"] = round(math.degrees(turn), 1)

    own = driver.robot.getSelf()
    own.getField("translation").setSFVec3f([-0.63, 0.36, 0.001])
    own.getField("rotation").setSFRotation([0, 0, 1, 0])
    own.resetPhysics()
    driver.tick_for(0.5)
    max_pitch = 0
    max_height = 0
    for _ in range(round(5 * 1000 / driver.step_ms)):
        x, _, z, _, pitch, _ = driver.pose()
        max_pitch = max(max_pitch, abs(pitch))
        max_height = max(max_height, z)
        if x > 0.255:
            break
        driver.drive(0.29, 0)
        driver.step()
    ramp_x = driver.pose()[0]
    outcomes["ramp"] = ramp_x > 0.20 and max_height > 0.065
    results["ramp_x_m"] = round(ramp_x, 3)
    results["ramp_height_m"] = round(max_height, 3)
    results["ramp_pitch_deg"] = round(math.degrees(max_pitch), 1)
    results["ramp_pose"] = driver.status()

    own.getField("translation").setSFVec3f([0.23, -0.37, 0.001])
    own.getField("rotation").setSFRotation([0, 0, 1, 0])
    own.resetPhysics()
    driver.tick_for(0.25)
    driver.tick_for(2.1, 0.30)
    driver.tick_for(0.17)
    outcomes["threshold"] = driver.pose()[0] > 0.57
    results["threshold_x_m"] = round(driver.pose()[0], 3)
    report = json.dumps({"checks": outcomes, "measurements": results}, indent=2)
    Path("smoke-results.json").write_text(report + "\n", encoding="utf-8")
    print(report, flush=True)
    driver.tick_for(0.1)
    driver.robot.simulationQuit(0 if all(outcomes.values()) else 1)


def hook_trials(driver):
    """Repeat the hanging-cube routes from scattered start poses and record whether the cube stays gripped."""
    own = driver.robot.getSelf()
    homes = {}
    for name in HOOK_CUBES:
        node = driver.robot.getFromDef(name)
        homes[name] = (node, node.getField("translation").getSFVec3f(), node.getField("rotation").getSFRotation())
    naive = "--naive" in sys.argv
    offsets = [(0.0, 0.0, 0.0), (0.03, 0.02, 0.2), (-0.03, -0.02, -0.2), (0.05, -0.04, 0.5), (-0.05, 0.05, -0.5),
               (0.08, 0.10, -0.9), (-0.08, -0.12, 0.9), (0.0, -0.15, 1.6)]
    rows = []
    for name, (offset, height, long_hanger) in HOOK_CUBES.items():
        for side in (1, -1):
            for shift, back, twist in offsets:
                for node, at, turn in homes.values():
                    node.getField("translation").setSFVec3f(at)
                    node.getField("rotation").setSFRotation(turn)
                    node.resetPhysics()
                driver.grip(False)
                driver.set_height(LIFT_MIN)
                driver.tick_for(1.3)
                x, y = rail_point(offset * math.sqrt(2) + shift, side * (0.40 + back))
                own.getField("translation").setSFVec3f([x, y, 0.001])
                own.getField("rotation").setSFRotation([0, 0, 1, (-math.pi / 4 if side > 0 else 3 * math.pi / 4) + twist])
                own.resetPhysics()
                driver.tick_for(0.4)
                steps = hook_cube_steps(driver, name)
                if naive and side < 0:
                    facing = 3 * math.pi / 4
                    along = offset * math.sqrt(2)
                    steps = [("pose", *rail_point(along, -0.36), facing), ("lift", height),
                             ("pose", *rail_point(along, -TRIDENT_REACH), facing, 0.006),
                             ("grip", True), ("lift", height + 0.012), ("move", -0.12)]
                began = started = driver.robot.getTime()
                stuck = None
                while steps:
                    now = driver.robot.getTime()
                    if now - started > 14 or not driver.step():
                        stuck = steps[0][0]
                        break
                    if driver.advance(steps[0], now - started):
                        steps.pop(0)
                        started = driver.robot.getTime()
                spent = driver.robot.getTime() - began
                driver.tick_for(0.3)
                driver.tick_for(1.0, -0.15)
                driver.tick_for(0.4)
                cx, cy, cz = homes[name][0].getPosition()
                rx, ry, _, _, _, yaw = driver.pose()
                ahead = (cx - rx) * math.cos(yaw) + (cy - ry) * math.sin(yaw)
                aside = (cy - ry) * math.cos(yaw) - (cx - rx) * math.sin(yaw)
                held = 0.14 < ahead < 0.24 and abs(aside) < 0.03 and cz > 0.05
                rows.append({"cube": name, "side": "open" if side > 0 else "far", "start": [shift, back, twist],
                             "held": held, "stuck_on": stuck, "seconds": round(spent, 1),
                             "cube_m": [round(v, 3) for v in (cx, cy, cz)]})
                print(json.dumps(rows[-1]), flush=True)
    Path("hook-results.json").write_text(json.dumps(rows, indent=1) + "\n", encoding="utf-8")
    driver.robot.simulationQuit(0 if all(row["held"] for row in rows) else 1)


if __name__ == "__main__":
    control = Driver()
    if "--hook-trials" in sys.argv:
        hook_trials(control)
    elif "--smoke" in sys.argv:
        smoke(control)
    else:
        manual(control, lab="--lab" in sys.argv)
