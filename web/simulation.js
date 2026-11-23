"use strict";

const LOM = (() => {
  const model = window.LOM_MODEL;
  const { field, robot: design, lab, round, layout } = model;
  const half = field.size / 2;
  const inner = half - field.baseInside;
  const radius = 0.120;
  const dt = 1 / 60;
  const clamp = (n, min, max) => Math.max(min, Math.min(max, n));
  const mix = (a, b, t) => a + (b - a) * t;
  const norm = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle));
  const signOf = (side) => side === "blue" ? -1 : 1;
  const opposite = (side) => side === "blue" ? "red" : "blue";
  const distance = (x, y) => Math.hypot(x, y);
  const ease = (current, target, amount) => current + clamp(target - current, -amount, amount);

  function bodyPoint(robot, forward, side = 0) {
    const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
    return [robot.x + c * forward - s * side, robot.y + s * forward + c * side];
  }

  function inBase(prop, side, margin = 0) {
    const sign = signOf(side);
    return prop.x * sign >= inner + margin && prop.y * sign >= inner + margin &&
      Math.abs(prop.x) <= half - margin && Math.abs(prop.y) <= half - margin;
  }

  function inStart(prop, side, margin = 0) {
    return layout.starts.some((start) => start.side === side &&
      Math.abs(prop.x - start.x) <= start.w / 2 - margin && Math.abs(prop.y - start.y) <= start.d / 2 - margin);
  }

  function robotState(start, index) {
    return {
      ...start, id: index, name: ["M01", "B02", "R01", "R02"][index], slot: index % 2,
      x: start.spawnX ?? start.x, y: start.spawnY ?? start.y,
      speed: 0, omega: 0, left: 0, right: 0, encL: 0, encR: 0, z: 0, roll: 0, pitch: 0,
      lift: design.liftMin, liftTarget: design.liftMin, jaws: design.jawOpen, closed: false,
      held: null, grasp: null, gripMotion: null, out: false, reset: false, mark: 0, tipped: false,
      hook: null, hookPhase: null,
      blocked: false, captures: 0, path: 0, peak: 0, maxTilt: 0, margin: 1, plan: null,
    };
  }

  const railSize = field.baseInside + field.baseWidth;
  const hardObstacles = [
    { x: 0, y: 0, w: 0.09, d: 0.09 },
    ...layout.pillars.map((p) => ({ ...p, w: 0.071, d: 0.071 })),
    ...layout.bases.flatMap((base) => {
      const rail = signOf(base.side) * (inner - field.baseWidth / 2);
      return [
        { x: rail, y: base.y, w: field.baseWidth, d: railSize },
        { x: base.x, y: rail, w: railSize, d: field.baseWidth },
      ];
    }),
  ];

  function overlap(x, y, box) {
    const a = x - box.x, b = y - box.y;
    const nx = a - clamp(a, -box.w / 2, box.w / 2);
    const ny = b - clamp(b, -box.d / 2, box.d / 2);
    const gap = Math.hypot(nx, ny);
    if (gap >= radius) return null;
    if (gap > 0.00001) return [nx * (radius / gap - 1), ny * (radius / gap - 1)];
    const sx = box.w / 2 - Math.abs(a), sy = box.d / 2 - Math.abs(b);
    return sx < sy ? [(a < 0 ? -1 : 1) * (sx + radius), 0] : [0, (b < 0 ? -1 : 1) * (sy + radius)];
  }

  function propHalf(prop) {
    return (prop.kind === "core" ? field.coreSize[2] : field.cubeSide) / 2;
  }

  function propWidth(prop) {
    return (prop.kind === "core" ? field.coreSize[0] : field.cubeSide) / 2;
  }

  function gripReach(prop) {
    if (prop.kind === "core") return design.coreReach;
    return prop.home && prop.origin === "hook" ? design.tridentReach : design.gripReach;
  }

  const jawHalfWidth = design.jawSize[1] / 2;
  const jawTip = design.jawX + design.jawSize[0] / 2;
  const backplate = design.carriage[0].at[0] + design.carriage[0].size[0] / 2;
  const mastGuides = design.body.filter((part) => part.shape === "upright");
  const crossbeam = field.tridentCrossAt * Math.SQRT2;
  const pickupAssistDistance = 0.62;
  const stopped = { speed: 0, turn: 0, brake: true };

  function railApproach(robot, prop) {
    if (!prop.home || (prop.origin !== "hook" && prop.kind !== "core")) return null;
    return robot.y < robot.x ? 3 * Math.PI / 4 : -Math.PI / 4;
  }

  function clearsCrossbeam(robot, c, s) {
    const along = Math.abs(c + s) * Math.SQRT1_2;
    const across = Math.abs(c - s) * Math.SQRT1_2;
    return mastGuides.every((guide) => {
      const [x, y] = bodyPoint(robot, guide.at[0], guide.at[1]);
      const longitudinal = Math.abs((x + y) * Math.SQRT1_2);
      const lateral = Math.abs((y - x) * Math.SQRT1_2);
      const [depth, width] = guide.size;
      return Math.abs(longitudinal - crossbeam) > (field.tridentBeamWidth + along * depth + across * width) / 2 ||
        lateral > (field.tridentCrossSpan + across * depth + along * width) / 2;
    });
  }

  function underRobot(x, y, z) {
    if (Math.abs(y - lab.rampY) < lab.rampWidth / 2) {
      const u = x - lab.rampStart;
      const height = lab.rampLength * Math.tan(lab.rampAngle * Math.PI / 180);
      if (u >= 0 && u < lab.rampLength) return u * height / lab.rampLength;
      if (u >= lab.rampLength && u < lab.rampLength + lab.platformLength && z > height * 0.40) return height;
    }
    if (Math.abs(y - lab.stepY) < 0.15 && Math.abs(x - lab.stepX) <= 0.0525) return lab.stepHeight;
    return 0;
  }

  class Simulation {
    constructor(options = {}) {
      this.settings = { speed: design.maxWheelSpeed, grip: design.friction, mass: design.mass, cg: design.centerOfGravity };
      this.reset(options);
    }

    reset({ mode = "match", division = "elementary", tag = 1 } = {}) {
      this.mode = mode;
      this.division = division;
      this.tag = tag;
      this.time = 0;
      this.frame = 0;
      this.running = false;
      this.finished = false;
      this.auto = false;
      this.scouting = { blue: 0, red: 0 };
      this.adjustments = { blue: 0, red: 0 };
      this.score = { blue: 0, red: 0 };
      this.pursuit = { blue: false, red: false };
      this.samples = [];
      this.events = [];
      const starts = mode === "lab"
        ? [{ side: "blue", x: lab.start[0], y: lab.start[1], yaw: 0 }]
        : mode === "practice" ? layout.starts.slice(0, 1) : layout.starts;
      this.robots = starts.map(robotState);
      const props = mode === "lab" ? [{
        id: "lab", kind: "cube", origin: "floor", x: lab.cube[0], y: lab.cube[1], z: field.cubeSide / 2,
      }] : layout.props;
      this.props = props.map((p) => ({ ...p, home: p.origin === "hook" || p.origin === "pillar" || p.origin === "core", vz: 0, heldBy: null, removedBy: null }));
      this.player = this.robots[0];
      if (mode === "practice") {
        const cube = this.props.find((p) => p.origin === "floor" && p.side === this.player.side);
        [this.player.x, this.player.y] = bodyPoint({ ...cube, yaw: this.player.yaw }, -design.gripReach - 0.10);
      }
    }

    get phase() {
      if (this.mode !== "match") return this.mode;
      if (this.finished) return "over";
      if (this.time < round.exploration) return "auto";
      return this.time < round.occupation ? "driver" : "endgame";
    }

    get manual() {
      return this.running && !this.finished && !this.auto &&
        (this.phase !== "auto" || this.division === "club") && !this.player.out && !this.player.tipped;
    }

    get canStartPractice() {
      return this.mode === "practice" && !this.running && this.frame === 0;
    }

    get hangAllowed() {
      return this.phase === "practice" || this.phase === "endgame" ||
        (this.phase === "driver" && !this.props.some((p) => p.home && (p.origin === "hook" || p.origin === "core")));
    }

    // With a human pilot in the match, the bots leave one hanging cube per alliance on the trident
    // during the autonomous stage, and the pilot's cube stays untouched until the pilot takes it.
    reservedForPilot(prop) {
      if (this.mode !== "match" || this.auto || !prop.home || prop.origin !== "hook") return false;
      const spare = this.division === "junior" && this.tag === 2 ? 3 : 2;
      return prop.tag === spare && (this.phase === "auto" || prop.side === this.player.side);
    }

    log(text) {
      this.events.unshift({ text, time: this.time });
      this.events.length = Math.min(this.events.length, 15);
    }

    start() {
      if (!this.finished) this.running = true;
    }

    pickupInfo(robot, target = null) {
      const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
      const canAssist = this.manual && robot === this.player && target === null && !robot.closed;
      let best = null;
      for (const prop of this.props) {
        if (target !== null && prop.id !== target) continue;
        if (prop.heldBy !== null || inBase(prop, opposite(robot.side), 0.006)) continue;
        if (this.phase === "auto" && (prop.kind === "core" || prop.origin === "pillar")) continue;
        const dx = prop.x - robot.x, dy = prop.y - robot.y;
        const forward = c * dx + s * dy, side = c * dy - s * dx;
        const inFront = forward >= 0.02 && forward <= 0.39 && Math.abs(side) <= 0.14;
        const withinTurn = this.manual && robot === this.player && forward >= -0.075 && distance(dx, dy) <= 0.33;
        if (!inFront && !withinTurn) continue;
        const halfWidth = propWidth(prop);
        const approachYaw = railApproach(robot, prop);
        const railAccess = approachYaw === null ||
          (Math.abs(norm(robot.yaw - approachYaw)) < 0.20 && clearsCrossbeam(robot, c, s));
        const hanging = prop.home && prop.origin === "hook";
        const reverseHook = hanging && approachYaw > 0;
        const shortHookFromBack = reverseHook && prop.tag === 3;
        const min = shortHookFromBack ? 0.204 : hanging ? design.tridentReach - 0.008 : backplate + halfWidth - 0.008;
        const max = jawTip + halfWidth - (shortHookFromBack ? 0.008 : 0.012);
        const jawContact = halfWidth + jawHalfWidth + Math.abs(side);
        const centered = jawContact <= design.jawOpen + 0.001;
        const heightOffset = prop.z - robot.z - robot.lift;
        const height = prop.z - robot.z - (shortHookFromBack ? 0.010 : 0);
        const verticalTolerance = prop.home ? 0.010 : propHalf(prop) + design.jawSize[2] / 2 - 0.007;
        const level = Math.abs(height - robot.lift) <= verticalTolerance;
        const aligned = forward >= min && forward <= max && centered && level && railAccess;
        const fitsJaws = robot.jaws >= Math.min(jawContact, design.jawOpen) - 0.002;
        const ready = aligned && fitsJaws && !robot.gripMotion?.openFirst;
        const candidate = { prop, forward, side, height, heightOffset, min, max, centered, level, approachYaw,
          reverseHook, shortHookFromBack, railAccess, aligned, fitsJaws, ready, jawContact };
        const goal = canAssist ? this.pickupAssistGoal(robot, candidate) : null;
        const positionError = Math.hypot(forward - gripReach(prop), side, heightOffset * 0.6);
        const score = aligned ? positionError : goal ?
          0.25 + distance(goal.x - robot.x, goal.y - robot.y) + 0.04 * Math.abs(norm(goal.yaw - robot.yaw)) :
          0.70 + positionError + (inFront ? 0 : 0.10);
        if (!best || score < best.score) best = { ...candidate, score };
      }
      return best;
    }

    pickupAssistGoal(robot, pickup) {
      if (!pickup || pickup.aligned) return null;
      const { prop, approachYaw } = pickup;
      let goal;
      if (approachYaw === null) {
        const reach = gripReach(prop);
        const span = distance(prop.x - robot.x, prop.y - robot.y) || 1;
        const x = prop.x - (prop.x - robot.x) * reach / span, y = prop.y - (prop.y - robot.y) * reach / span;
        const blocked = Math.abs(x) > 0.85 || Math.abs(y) > 0.85 ||
          (this.mode !== "lab" && hardObstacles.some((box) => overlap(x, y, box)));
        const detour = blocked ? this.approach(robot, prop) : null;
        goal = blocked ? detour && { ...detour, line: true } : { x, y, yaw: Math.atan2(prop.y - y, prop.x - x) };
      } else {
        const reach = pickup.shortHookFromBack ? (pickup.min + pickup.max) / 2 : gripReach(prop);
        const [x, y] = bodyPoint({ ...prop, yaw: approachYaw }, -reach);
        goal = { x, y, yaw: approachYaw, rail: true };
      }
      if (!goal || distance(goal.x - robot.x, goal.y - robot.y) > pickupAssistDistance) return null;
      return goal;
    }

    cancelPickup(robot) {
      if (!robot.plan?.assist) return false;
      robot.plan = null;
      robot.liftTarget = robot.grasp?.clearance ?? robot.lift;
      if (robot.gripMotion) this.grip(robot, false);
      return true;
    }

    manualGrip(robot) {
      if (!this.manual || robot.hook) return false;
      if (this.cancelPickup(robot)) {
        this.log("Подвод захвата отменён");
        return true;
      }
      if (robot.closed || robot.held) return this.grip(robot);
      const pickup = this.pickupInfo(robot);
      const goal = this.pickupAssistGoal(robot, pickup);
      if (!goal) return (pickup?.aligned || this.mode === "lab") && this.grip(robot);

      robot.plan = {
        id: pickup.prop.id, assist: true, actions: [], time: 0, age: 0,
        line: goal.rail || goal.line ? goal.yaw : null,
        reach: pickup.shortHookFromBack ? (pickup.min + pickup.max) / 2 : gripReach(pickup.prop),
        lower: pickup.shortHookFromBack ? 0.010 : 0,
      };
      return true;
    }

    // Steers the player's robot onto the highlighted prop without stop-and-turn waypoints.
    // The lift moves while the robot drives, and the jaws close as soon as the prop is in the grip zone.
    assistCommand(robot, plan) {
      const prop = this.props.find((p) => p.id === plan.id);
      if (prop.heldBy !== null && prop.heldBy !== robot.id) {
        this.cancelPickup(robot);
        this.log("Предмет уже захвачен");
        return stopped;
      }
      plan.age += dt;
      if (plan.gripping) {
        if (robot.gripMotion) return stopped;
        robot.plan = null;
        if (robot.held?.id !== plan.id) {
          this.grip(robot, false);
          this.log("Не удалось сомкнуть губки на предмете");
        }
        return stopped;
      }
      if (plan.age > 9) {
        this.cancelPickup(robot);
        const crowded = this.robots.some((other) => other !== robot && !other.out &&
          distance(other.x - robot.x, other.y - robot.y) < radius * 2 + 0.03);
        this.log(crowded ? "Подводу мешает другой робот. Подожди или зайди с другой стороны" :
          "Не удалось подвести захват, подъедь ближе");
        return stopped;
      }
      robot.liftTarget = clamp(prop.z - robot.z - plan.lower, design.liftMin, design.liftMax);
      const dx = prop.x - robot.x, dy = prop.y - robot.y;
      const aligned = this.pickupInfo(robot, plan.id)?.aligned;
      const close = () => {
        this.grip(robot, true, plan.id);
        plan.gripping = true;
        return stopped;
      };
      const pace = (gap) => Math.abs(gap) < 0.003 ? 0 : Math.sign(gap) * clamp(Math.abs(gap) * 5, 0.09, 0.7);

      const bearing = norm(Math.atan2(dy, dx) - robot.yaw);
      const gap = distance(dx, dy) - plan.reach;
      const direct = () => ({
        speed: Math.abs(bearing) < 0.7 ? pace(gap) * Math.cos(bearing) : gap < -0.01 ? -0.3 : 0,
        turn: clamp(bearing * 3.2, -0.87, 0.87),
      });
      if (plan.line === null) {
        // Free prop: turn the jaws towards it while closing the distance.
        if (aligned && Math.abs(bearing) < 0.10 && Math.abs(gap) < 0.03) return close();
        return direct();
      }

      // Prop on the trident (or behind an obstacle): it must be met close to the approach line.
      const c = Math.cos(plan.line), s = Math.sin(plan.line);
      const left = dx * s - dy * c;                 // robot's offset to the left of the approach line
      const ahead = dx * c + dy * s - plan.reach;   // distance still to drive along the line
      const facing = norm(plan.line - robot.yaw);
      // Near the line the jaws have enough slack to simply turn onto the prop and drive in.
      if (Math.abs(left) < (plan.near ? 0.030 : 0.022)) plan.near = true;
      else plan.near = false;
      if (plan.near && !plan.strict) {
        const settled = Math.abs(bearing) < 0.05 && Math.abs(gap) < 0.005;
        if (aligned && settled) return close();
        if (settled && Math.abs(robot.lift - robot.liftTarget) < 0.002) plan.strict = true;
        else return direct();
      }
      // Otherwise steer back onto the line, backing up only for the room needed to straighten.
      if (aligned && Math.abs(ahead) < 0.004 && Math.abs(left) < 0.005 && Math.abs(facing) < 0.03) return close();
      const room = clamp((Math.abs(left) - (plan.strict ? 0 : 0.018)) * 5, 0, 0.16);
      if (Math.abs(left) < 0.004) plan.back = false;
      else if (ahead < room) plan.back = true;
      else if (ahead > room + 0.025) plan.back = false;
      const steer = Math.abs(left) < 0.004 ? 0 : Math.atan(left / 0.07) * (plan.back ? 1 : -1);
      const error = norm(plan.line + clamp(steer, -0.75, 0.75) - robot.yaw);
      const turn = clamp(error * 3.2, -0.87, 0.87);
      if (Math.abs(error) > 0.8) return { speed: 0, turn };
      if (plan.back) return { speed: -clamp(0.25 + (room - ahead) * 4, 0.25, 0.6) * Math.cos(error), turn };
      return { speed: pace(ahead) * Math.cos(error), turn };
    }

    grip(robot, close = !robot.closed, target = null) {
      if (robot.out || robot.tipped || robot.hook || robot.closed === close) return false;
      robot.closed = close;
      if (close) {
        const current = target === null ? this.pickupInfo(robot) : null;
        robot.gripMotion = {
          target: target ?? (current?.aligned ? current.prop.id : null),
          openFirst: robot.jaws < design.jawOpen - 0.002,
          announce: target === null,
        };
      } else {
        robot.gripMotion = null;
      }
      if (!close && robot.held) {
        const prop = robot.held;
        [prop.x, prop.y] = bodyPoint(robot, prop.kind === "core" ? design.coreReach : design.releaseReach);
        prop.z = robot.z + robot.lift + robot.grasp.height;
        prop.vz = 0;
        prop.heldBy = null;
        robot.held = null;
        robot.grasp = null;
        if (robot.id === 0) this.log(prop.kind === "core" ? "Ядро отпущено" : "Куб отпущен");
      }
      return true;
    }

    updateJaws(robot) {
      if (robot.hook || robot.out || robot.tipped) return;
      if (robot.gripMotion?.openFirst) {
        robot.jaws = ease(robot.jaws, design.jawOpen, design.jawSpeed * dt);
        if (robot.jaws === design.jawOpen) robot.gripMotion.openFirst = false;
        return;
      }
      const closed = robot.held ? propWidth(robot.held) + jawHalfWidth : design.jawClosed;
      const next = ease(robot.jaws, robot.closed ? closed : design.jawOpen, design.jawSpeed * dt);
      if (!robot.gripMotion) {
        robot.jaws = next;
        return;
      }
      const hit = this.pickupInfo(robot, robot.gripMotion.target);
      const touch = hit ? Math.min(hit.jawContact, design.jawOpen) : 0;
      if (hit?.ready && next <= touch) {
        const { prop } = hit;
        if (prop.home && prop.origin === "hook" && this.phase === "auto") prop.removedBy = robot.side;
        prop.home = false;
        prop.heldBy = robot.id;
        robot.held = prop;
        robot.grasp = { forward: hit.forward, side: hit.side, height: hit.heightOffset };
        if (hit.reverseHook) {
          const clearance = field.tridentHeight - propHalf(prop) + 0.018 - robot.z - hit.heightOffset;
          robot.grasp.clearance = clamp(clearance, design.liftMin, design.liftMax);
          robot.liftTarget = Math.max(robot.liftTarget, robot.grasp.clearance);
        }
        robot.gripMotion = null;
        if (robot.plan?.assist) {
          if (this.mode === "practice" && prop.z <= propHalf(prop) + 0.005) {
            robot.liftTarget = Math.max(robot.liftTarget, design.liftMin + 0.05);
          }
          robot.plan = null;
        }
        robot.jaws = Math.max(next, propWidth(prop) + jawHalfWidth);
        robot.captures++;
        if (prop.kind === "core") this.log(robot.name + " захватил ядро");
        else if (robot.id === 0) this.log("Куб в захвате");
        return;
      }
      robot.jaws = next;
      if (next === design.jawClosed) {
        if (robot.id === 0 && robot.gripMotion.announce) this.log("Губки закрылись без предмета");
        robot.gripMotion = null;
      }
    }

    freeSlots(robot) {
      return layout.hangSlots.filter((slot) => slot.side === robot.side &&
        !this.robots.some((other) => other !== robot && other.hook?.id === slot.id));
    }

    nearbyHook(robot, preferred = null) {
      return this.freeSlots(robot).find((slot) => (!preferred || slot.id === preferred) &&
        distance(slot.x - robot.x, slot.y - robot.y) < 0.064 &&
        Math.abs(norm(slot.yaw - robot.yaw)) < 0.42);
    }

    toggleHang(robot, preferred = null) {
      if (!this.running || robot.out || robot.tipped || this.mode === "lab") return false;
      if (robot.hook) {
        if (robot.hookPhase === "align") {
          robot.hook = null;
          robot.hookPhase = null;
        } else {
          robot.hookPhase = "release";
          robot.liftTarget = robot.hook.barZ - design.hang.heightAboveLift;
        }
        return true;
      }
      const slot = this.nearbyHook(robot, preferred);
      if (!this.hangAllowed || robot.held || !slot) {
        if (!robot.id && !preferred) {
          this.log(!this.hangAllowed ? "Подвес доступен после очистки трезубца или в финальные 30 с" :
            robot.held ? "Освободи захват перед подвесом" : "Подъедь к своей перекладине и направь крюк к метке");
        }
        return false;
      }
      Object.assign(robot, {
        x: slot.x, y: slot.y, yaw: slot.yaw, hook: slot, hookPhase: "align",
        speed: 0, omega: 0, left: 0, right: 0, plan: null,
        gripMotion: null,
        liftTarget: slot.barZ - design.hang.heightAboveLift,
      });
      return true;
    }

    resetRobot(robot) {
      if (!robot.out || robot.reset || this.phase !== "driver") return false;
      const starts = layout.starts.filter((slot) => slot.side === robot.side);
      const free = starts.find((slot) => this.robots.every((other) => other === robot || other.out ||
        distance(other.x - slot.x, other.y - slot.y) > radius * 2));
      if (!free) return false;
      Object.assign(robot, {
        x: free.spawnX, y: free.spawnY, yaw: free.yaw, out: false, reset: true,
        speed: 0, omega: 0, left: 0, right: 0, plan: null, closed: false, gripMotion: null,
      });
      this.adjustments[opposite(robot.side)] += 50;
      if (robot.id === 0) this.log("Робот возвращён на старт");
      return true;
    }

    setLab(course) {
      this.reset({ mode: "lab" });
      if (course === "ramp") {
        this.player.x = -0.67;
        this.player.y = lab.rampY;
      } else if (course === "step") {
        this.player.x = 0.19;
        this.player.y = lab.stepY;
      }
    }

    approach(robot, prop) {
      if (prop.home && (prop.kind === "core" || prop.origin === "hook")) {
        const yaw = prop.kind === "core" && robot.side === "red" ? 3 * Math.PI / 4 : -Math.PI / 4;
        const [x, y] = bodyPoint({ ...prop, yaw }, -gripReach(prop));
        return { x, y, yaw, rail: true };
      }
      const from = Math.atan2(prop.y - robot.y, prop.x - robot.x);
      const variants = [0, 0.45, -0.45, 0.85, -0.85, 1.4, -1.4, Math.PI];
      for (const delta of variants) {
        const yaw = from + delta;
        const x = prop.x - gripReach(prop) * Math.cos(yaw);
        const y = prop.y - gripReach(prop) * Math.sin(yaw);
        if (Math.abs(x) > 0.85 || Math.abs(y) > 0.85) continue;
        if (this.mode !== "lab" && hardObstacles.some((box) => overlap(x, y, box))) continue;
        if (this.phase === "auto" && (x + y) * signOf(robot.side) < 0.20) continue;
        return { x, y, yaw };
      }
      return null;
    }

    makePlan(robot) {
      if (this.mode === "lab") return null;
      const sign = signOf(robot.side), side = robot.side;
      if (!robot.held && this.phase === "endgame") {
        const slots = this.freeSlots(robot).filter((slot) =>
          !this.robots.some((other) => other !== robot && other.plan?.hook === slot.id));
        const slot = slots.find((entry) => entry.slot === robot.slot) ?? slots[0];
        if (!slot) return null;
        const staging = bodyPoint(slot, -0.18);
        return { id: null, hook: slot.id, time: 0, age: 0, actions: [
          { kind: "drive", x: staging[0], y: staging[1], yaw: slot.yaw },
          { kind: "drive", x: slot.x, y: slot.y, yaw: slot.yaw },
        ] };
      }
      const wantsTag = this.phase === "auto" && this.division === "junior" && robot.slot === 0;
      const tagId = wantsTag ? side + "_" + this.tag : "";
      const deliver = (prop) => {
        const targetTag = prop.id === tagId;
        const lane = targetTag ? field.startCenter[1] : 0.76;
        const aim = targetTag ? 0.482 : prop.kind === "core" ? 0.490 : 0.474;
        const yaw = robot.slot === 0 ? (sign < 0 ? Math.PI : 0) : sign * Math.PI / 2;
        const place = robot.slot === 0
          ? { x: sign * aim, y: sign * lane, yaw }
          : { x: sign * lane, y: sign * aim, yaw };
        const neutral = robot.slot === 0
          ? { x: 0.38 * sign, y: mix(robot.y, place.y, 0.7) }
          : { x: mix(robot.x, place.x, 0.7), y: 0.38 * sign };
        return [
          { kind: "lift", z: targetTag ? 0.06 : 0.145 },
          { kind: "drive", ...neutral },
          { kind: "drive", ...place },
          { kind: "grip", close: false },
        ];
      };

      if (robot.held) return { id: robot.held.id, actions: deliver(robot.held), time: 0, age: 0 };
      const choices = this.props.filter((prop) => {
        if (prop.heldBy !== null || inBase(prop, "blue", 0.006) || inBase(prop, "red", 0.006)) return false;
        if (this.reservedForPilot(prop)) return false;
        if (this.phase === "auto" && ((prop.x + prop.y) * sign < 0.08 || prop.origin === "pillar" ||
            (this.division === "junior" && prop.id === side + "_" + this.tag &&
             (inStart(prop, side, 0.02) || robot.slot !== 0)))) return false;
        return !this.robots.some((ally) => ally !== robot && ally.side === side && ally.plan?.id === prop.id);
      }).map((prop) => ({ prop, approach: this.approach(robot, prop) }))
        .filter(({ approach }) => approach)
        .sort((a, b) => {
          const weight = ({ prop, approach }) => distance(approach.x - robot.x, approach.y - robot.y) +
            (prop.id === tagId ? -10 : 0) + (prop.kind === "core" ? -4 : 0) +
            (robot.slot === 1 && prop.origin === "hook" ? -0.10 : 0);
          return weight(a) - weight(b);
        });
      const pick = choices[0];
      if (!pick) return null;
      const actions = [
        { kind: "grip", close: false },
        { kind: "lift", z: pick.prop.z },
      ];
      if (pick.approach.rail) {
        const [x, y] = bodyPoint(pick.approach, -0.18);
        actions.push({ kind: "drive", x, y, yaw: pick.approach.yaw });
        if (pick.prop.kind === "core" && !this.pursuit[side]) {
          this.pursuit[side] = true;
          this.log(side === "blue" ? "Синие идут за ядром" : "Красные идут за ядром");
        }
      }
      actions.push({ kind: "drive", ...pick.approach, stop: pick.approach.rail ? 0.005 : 0.011 },
        { kind: "grip", close: true }, ...deliver(pick.prop));
      return { id: pick.prop.id, actions, time: 0, age: 0 };
    }

    autoCommand(robot) {
      const idle = { speed: 0, turn: 0 };
      if (robot.hook || robot.tipped) return idle;
      if (robot.out) {
        if (this.phase === "driver" && this.time > robot.outAt + 2) this.resetRobot(robot);
        return idle;
      }
      if (!robot.plan) robot.plan = this.makePlan(robot);
      const plan = robot.plan;
      if (!plan) return idle;
      if (plan.assist) return this.assistCommand(robot, plan);
      if (plan.id === "core" && !robot.held) {
        const core = this.props.find((p) => p.id === plan.id);
        if (core.heldBy !== null || inBase(core, "blue", propWidth(core)) || inBase(core, "red", propWidth(core))) {
          robot.plan = null;
          return idle;
        }
      }
      plan.time += dt;
      plan.age += dt;
      if (plan.time > 8 || plan.age > 22) {
        robot.plan = null;
        return idle;
      }
      const move = plan.actions[0];
      if (move.kind === "drive") {
        const dx = move.x - robot.x, dy = move.y - robot.y;
        const dist = distance(dx, dy);
        const stop = move.stop ?? 0.025;
        let aim = dist > stop ? Math.atan2(dy, dx) : (move.yaw ?? robot.yaw);
        const backward = move.reverse && dist > stop && Math.abs(norm(aim - robot.yaw)) > Math.PI / 2;
        if (backward) aim += Math.PI;
        const error = norm(aim - robot.yaw);
        if (dist > stop || Math.abs(error) > (move.rail ? 0.025 : 0.075)) {
          const speed = dist > stop && Math.abs(error) < 0.9
            ? clamp(dist * 3.8, move.stop ? 0.07 : 0.12, move.speedLimit ?? 0.86) *
              Math.cos(error) * (backward ? -1 : 1) : 0;
          return { speed, turn: clamp(error * 2.1, -0.87, 0.87) };
        }
      } else if (move.kind === "lift") {
        robot.liftTarget = clamp(move.z, design.liftMin, design.liftMax);
        if (Math.abs(robot.lift - robot.liftTarget) > 0.008) return idle;
      } else if (move.close) {
        if (!move.started) {
          this.grip(robot, true, plan.id);
          move.started = true;
          return idle;
        }
        if (robot.gripMotion) return idle;
        if (robot.held?.id !== plan.id) {
          robot.plan = null;
          return idle;
        }
      } else {
        this.grip(robot, false);
        if (robot.jaws < design.jawOpen - 0.003) return idle;
      }

      plan.actions.shift();
      plan.time = 0;
      if (!plan.actions.length) {
        robot.plan = null;
        if (plan.hook) this.toggleHang(robot, plan.hook);
      }
      return idle;
    }

    moveHook(robot) {
      const engagement = robot.hook.barZ - design.hang.heightAboveLift;
      robot.lift = ease(robot.lift, robot.liftTarget, design.liftSpeed * dt);
      if (robot.hookPhase !== "align") robot.z = Math.max(0, engagement - robot.lift);
      if (Math.abs(robot.lift - robot.liftTarget) > 0.0001) return;
      if (robot.hookPhase === "align") {
        robot.hookPhase = "pull";
        robot.liftTarget = engagement - design.hang.rise;
      } else if (robot.hookPhase === "pull") {
        robot.hookPhase = "hanging";
        this.log(robot.name + " повис на перекладине");
      } else if (robot.hookPhase === "release") {
        robot.hook = null;
        robot.hookPhase = null;
        robot.z = 0;
        if (!robot.id) this.log("M01 опущен на поле");
      }
    }

    move(robot, input) {
      if (robot.out || robot.tipped) return;
      if (robot.hook) {
        this.moveHook(robot);
        return;
      }
      const top = this.settings.speed;
      const motorForce = input.brake ? 2 * design.wheelTorque / design.wheelRadius : 4.6;
      const accel = Math.min(this.settings.grip * 9.81, motorForce / this.settings.mass);
      const nominal = top * input.speed, turn = top * 0.71 * input.turn;
      const scale = Math.max(1, (Math.abs(nominal) + Math.abs(turn)) / top);
      robot.left = ease(robot.left, (nominal - turn) / scale, accel * dt);
      robot.right = ease(robot.right, (nominal + turn) / scale, accel * dt);
      robot.encL += robot.left * dt / design.wheelRadius;
      robot.encR += robot.right * dt / design.wheelRadius;
      let speed = (robot.left + robot.right) / 2;
      robot.omega = (robot.right - robot.left) / design.trackWidth;
      robot.yaw = norm(robot.yaw + robot.omega * dt);
      robot.lift = ease(robot.lift, robot.liftTarget, design.liftSpeed * dt);

      if (this.mode === "lab") {
        const [frontX, frontY] = bodyPoint(robot, design.axleX);
        const [backX, backY] = bodyPoint(robot, design.casterX);
        const front = underRobot(frontX, frontY, robot.z), back = underRobot(backX, backY, robot.z);
        const [leftX, leftY] = bodyPoint(robot, design.axleX, design.trackWidth * 0.5);
        const [rightX, rightY] = bodyPoint(robot, design.axleX, -design.trackWidth * 0.5);
        const lateral = underRobot(leftX, leftY, robot.z) - underRobot(rightX, rightY, robot.z);
        robot.z = ease(robot.z, Math.max(0, mix(front, back, 0.34)), dt * 0.75);
        robot.pitch = Math.atan2(front - back, design.axleX - design.casterX);
        robot.roll = Math.atan2(lateral, design.trackWidth);
        const climb = speed * robot.pitch > 0;
        if (climb && Math.tan(Math.abs(robot.pitch)) > this.settings.grip) speed *= 0.17;
        const lever = Math.sqrt(2 * design.wheelRadius * lab.stepHeight - lab.stepHeight ** 2);
        const stallTorque = .36 * this.settings.mass * 9.81 * lever;
        const travelX = speed * dt * Math.cos(robot.yaw);
        const side = Math.sign(travelX);
        const edge = lab.stepX - side * (.0525 + lever);
        const clearance = (edge - frontX) * side;
        robot.blocked = false;
        if (side && Math.abs(frontY - lab.stepY) < .15 && clearance >= -0.0001 && clearance < Math.abs(travelX) &&
            (stallTorque > design.wheelTorque || this.settings.grip < .46)) {
          speed *= Math.max(0, clearance) / Math.abs(travelX);
          robot.blocked = true;
        }
      }

      const loadHeight = robot.held ? robot.lift * 0.16 : 0;
      const limitAngle = Math.atan2(design.trackWidth * 0.5, this.settings.cg + loadHeight);
      const tilt = Math.abs(robot.roll + Math.atan2(speed * robot.omega, 9.81)) + Math.abs(robot.pitch);
      robot.margin = clamp(1 - tilt / limitAngle, 0, 1);
      robot.maxTilt = Math.max(robot.maxTilt, tilt);
      if (tilt > limitAngle) {
        this.grip(robot, false);
        robot.tipped = true;
        if (!robot.id) this.log("Потеря устойчивости");
      }
      const oldX = robot.x, oldY = robot.y;
      robot.x += Math.cos(robot.yaw) * speed * dt;
      robot.y += Math.sin(robot.yaw) * speed * dt;
      if (this.mode !== "lab") {
        for (const obstacle of hardObstacles) {
          const push = overlap(robot.x, robot.y, obstacle);
          if (!push) continue;
          robot.x += push[0];
          robot.y += push[1];
        }
      }
      const traveled = distance(robot.x - oldX, robot.y - oldY);
      robot.path += traveled;
      robot.speed = traveled / dt * Math.sign(speed);
      robot.peak = Math.max(robot.peak, Math.abs(robot.speed));
      if (Math.max(Math.abs(robot.x), Math.abs(robot.y)) >= half + 0.020) {
        this.grip(robot, false);
        robot.out = true;
        robot.outAt = this.time;
        robot.speed = 0;
        if (!robot.id) this.log("Робот сошёл с поля");
      }
    }

    resolveRobots() {
      for (let i = 0; i < this.robots.length; i++) {
        const a = this.robots[i];
        if (a.out) continue;
        for (let j = 0; j < i; j++) {
          const b = this.robots[j];
          if (b.out) continue;
          const dx = a.x - b.x, dy = a.y - b.y;
          const gap = distance(dx, dy);
          if (gap > radius * 2 || gap < 0.00001) continue;
          const overlap = (radius * 2 - gap) / gap;
          const aShare = a.hook ? 0 : b.hook ? 1 : 0.5;
          const bShare = b.hook ? 0 : a.hook ? 1 : 0.5;
          a.x += dx * overlap * aShare;
          a.y += dy * overlap * aShare;
          b.x -= dx * overlap * bShare;
          b.y -= dy * overlap * bShare;
          if (a.side !== b.side && this.phase === "auto") {
            for (const contact of [a, b]) {
              if (contact.mark === 1) {
                this.adjustments[contact.side] -= 200;
                contact.mark = 2;
              }
            }
          }
        }
      }
    }

    propsStep() {
      for (const prop of this.props) {
        if (prop.home) continue;
        if (prop.heldBy !== null) {
          const robot = this.robots[prop.heldBy];
          const grasp = robot.grasp;
          if (grasp.clearance && robot.lift >= grasp.clearance - 0.003) grasp.clearance = null;
          const sideRoom = Math.max(0, robot.jaws - jawHalfWidth - propWidth(prop));
          grasp.side = clamp(grasp.side, -sideRoom, sideRoom);
          [prop.x, prop.y] = bodyPoint(robot, grasp.forward, grasp.side);
          const floor = this.mode === "lab" ? underRobot(prop.x, prop.y, prop.z) : 0;
          prop.z = Math.max(floor + propHalf(prop), robot.z + robot.lift + grasp.height);
          continue;
        }
        const floor = this.mode === "lab" ? underRobot(prop.x, prop.y, prop.z) : 0;
        prop.vz -= 9.81 * dt;
        prop.z += prop.vz * dt;
        if (prop.z <= floor + propHalf(prop)) {
          prop.vz = 0;
          prop.z = floor + propHalf(prop);
        }
        if (Math.max(Math.abs(prop.x), Math.abs(prop.y)) > half) {
          const edge = clamp((prop.x - prop.y) / 2, -0.88, 0.88);
          const where = (edge < 0 ? -1 : 1) * Math.max(0.67, Math.abs(edge));
          prop.x = where;
          prop.y = -where;
        }
      }
    }

    countScouting() {
      for (const side of ["blue", "red"]) {
        let points = this.props.filter((prop) => prop.removedBy === side).length * 100;
        for (const prop of this.props) {
          if (prop.heldBy !== null || prop.home || prop.kind !== "cube" || prop.z > propHalf(prop) + 0.005) continue;
          if (prop.id === side + "_" + this.tag && this.division === "junior") continue;
          if (inBase(prop, side, 0.008)) points += 300;
        }
        if (this.division === "junior") {
          const target = this.props.find((prop) => prop.id === side + "_" + this.tag);
          const done = target.heldBy === null && !target.home && target.z <= propHalf(target) + 0.005 &&
            inStart(target, side, 0.010);
          points += done ? 400 : -200;
        }
        this.scouting[side] = points;
      }
    }

    updateScore() {
      if (this.time < round.exploration && this.mode === "match") this.countScouting();
      for (const side of ["blue", "red"]) {
        let total = this.scouting[side] + this.adjustments[side];
        if (this.time >= round.exploration || this.mode === "practice") {
          for (const prop of this.props) {
            if (prop.heldBy !== null || prop.home || prop.z > propHalf(prop) + 0.007) continue;
            if (prop.kind === "core" && !this.finished && this.mode !== "practice") continue;
            if (!inBase(prop, side, prop.kind === "core" ? propWidth(prop) : 0.008)) continue;
            total += prop.kind === "core" ? 300 : 100;
          }
        }
        if (this.finished) {
          const suspended = this.robots.filter((r) => r.side === side && r.hook?.side === side && r.z > 0.012);
          total += 400 * suspended.length + (suspended.length === 2 ? 200 : 0);
          total += this.robots.filter((r) => r.out && r.side !== side).length * 50;
          total += this.robots.filter((r) => r.side === side && !r.out && !suspended.includes(r) &&
            Math.abs(r.x - r.y) < 0.139 && (r.x + r.y) * signOf(side) > 0.184 &&
            (r.x + r.y) * signOf(side) < 0.68).length * 100;
        }
        this.score[side] = total;
      }
    }

    tick(input = { speed: 0, turn: 0, lift: 0 }) {
      if (!this.running || this.finished) return;
      const previousPhase = this.phase;
      const allowHand = this.manual || (this.mode === "lab" && this.auto);
      for (const robot of this.robots) {
        let command;
        if (this.manual && robot === this.player && robot.plan?.assist && (input.speed || input.turn || input.lift)) {
          this.cancelPickup(robot);
        }
        if (robot.id || (this.auto && this.mode !== "lab") || (previousPhase === "auto" && this.division !== "club") ||
            (this.manual && robot === this.player && robot.plan?.assist)) {
          command = this.autoCommand(robot);
        } else {
          const braking = Boolean(robot.grasp?.clearance) || (this.manual && robot.gripMotion &&
            this.pickupInfo(robot, robot.gripMotion.target)?.aligned);
          command = allowHand && !braking ? input : braking ? stopped : { speed: 0, turn: 0 };
          if (allowHand && !braking && input.lift && !robot.hook) {
            robot.liftTarget = clamp(robot.liftTarget + input.lift * design.liftSpeed * dt, design.liftMin, design.liftMax);
          }
        }
        this.move(robot, command);
        if (previousPhase === "auto" && (robot.x + robot.y) * signOf(robot.side) < 0.166 && !robot.mark) {
          robot.mark = 1;
          this.adjustments[robot.side] -= 100;
        }
      }
      this.resolveRobots();
      for (const robot of this.robots) this.updateJaws(robot);
      this.propsStep();
      this.frame++;
      this.time = this.frame * dt;
      if (this.mode === "match" && this.frame === round.exploration * 60) {
        this.countScouting();
        for (const robot of this.robots) if (!robot.plan?.assist) robot.plan = null;
        this.log(this.auto || this.division === "club" ? "Начался этап борьбы за ядро" : "Управление передано пилоту");
      }
      if (this.mode === "match" && this.frame === round.occupation * 60) {
        for (const robot of this.robots) if (!robot.held && !robot.plan?.assist) robot.plan = null;
        this.log("Финальные 30 секунд · Подвес открыт");
      }
      if (this.mode === "match" && this.frame === round.duration * 60) {
        this.finished = true;
        this.running = false;
        this.log("Матч завершён");
      }
      if (this.mode !== "lab") this.updateScore();
      if (this.frame % 6 === 0) this.record();
    }

    record() {
      const r = this.player;
      this.samples.push([this.time, r.x, r.y, r.z, r.yaw, r.pitch, r.roll, r.speed, r.encL, r.encR,
        r.lift, r.margin, r.held ? r.held.id : "", this.score.blue, this.score.red, r.hook?.id ?? ""]);
      if (this.samples.length > 9000) this.samples.shift();
    }
  }

  return {
    model, Simulation, dt, bodyPoint, propHalf, propWidth, inBase, clamp, signOf,
    csvHead: "time_s,x_m,y_m,z_m,yaw_rad,pitch_rad,roll_rad,speed_m_s,left_rad,right_rad,lift_m,stability,held,blue,red,hook",
  };
})();
