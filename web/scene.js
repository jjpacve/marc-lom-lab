"use strict";

class ArenaView {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.azimuth = Math.PI / 4;
    this.elevation = 0.66;
    this.zoom = 1;
    this.top = false;
    this.dpr = 1;
    this.width = 0;
    this.height = 0;
    this.faces = [];
    this.colors = {
      blue: "#59a5ff", red: "#ff6a70", tape: "#dce4de", alloy: "#8dabae",
      metal: "#748e9c", dark: "#24323b", tire: "#161f27", orange: "#d69d47",
      accent: "#b8ed6b", core: "#66edc6", ramp: "#46665e",
    };
  }

  project(x, y, z) {
    const side = x * this.c + y * this.s;
    const far = -x * this.s + y * this.c;
    return [this.cx + this.scale * side, this.cy - this.scale * (far * this.alt + z * this.up),
      far * this.up - z * this.alt];
  }

  polygon(points, fill, stroke, width = 1) {
    const ctx = this.ctx;
    ctx.beginPath();
    ctx.moveTo(...points[0].slice(0, 2));
    for (const p of points.slice(1)) ctx.lineTo(...p.slice(0, 2));
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
    if (stroke) {
      ctx.strokeStyle = stroke;
      ctx.lineWidth = width;
      ctx.stroke();
    }
  }

  plane(corners, color, stroke, width) {
    this.polygon(corners.map(([x, y, z = 0]) => this.project(x, y, z)), color, stroke, width);
  }

  rect(x, y, w, d, color, stroke, z = 0) {
    this.plane([[x - w / 2, y - d / 2, z], [x + w / 2, y - d / 2, z],
      [x + w / 2, y + d / 2, z], [x - w / 2, y + d / 2, z]], color, stroke);
  }

  tape(a, b, width, color, z = 0.001) {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const d = Math.hypot(dx, dy);
    const ox = -dy * width / (2 * d), oy = dx * width / (2 * d);
    this.plane([[a[0] + ox, a[1] + oy, z], [b[0] + ox, b[1] + oy, z],
      [b[0] - ox, b[1] - oy, z], [a[0] - ox, a[1] - oy, z]], color);
  }

  addFace(verts, color, light, edge) {
    const points = verts.map(([x, y, z]) => this.project(x, y, z));
    const [a, b, c] = points;
    if ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) >= -0.01) return;
    const depth = points.reduce((sum, point) => sum + point[2], 0) / points.length;
    const base = parseInt(color.slice(1), 16);
    const rgb = [base >> 16, (base >> 8) & 255, base & 255].map((n) => Math.min(255, Math.round(n * light)));
    this.faces.push({ points, depth, color: `rgb(${rgb.join(",")})`, edge });
  }

  box(at, size, color, yaw = 0, transform = null, edge = "rgba(10,23,29,.47)") {
    const [x, y, z] = at;
    const [w, d, h] = size.map((n) => n / 2);
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const verts = [
      [-w, -d, -h], [w, -d, -h], [w, d, -h], [-w, d, -h],
      [-w, -d, h], [w, -d, h], [w, d, h], [-w, d, h],
    ].map(([u, v, t]) => {
      const point = [x + c * u - s * v, y + s * u + c * v, z + t];
      return transform ? transform(...point) : point;
    });
    const surfaces = [
      [[4, 5, 6, 7], 1.20], [[1, 0, 3, 2], 0.65],
      [[0, 1, 5, 4], 0.84], [[1, 2, 6, 5], 0.97],
      [[2, 3, 7, 6], 0.69], [[3, 0, 4, 7], 0.72],
    ];
    for (const [indices, light] of surfaces) this.addFace(indices.map((i) => verts[i]), color, light, edge);
  }

  tire(axle, center, height, radius, width, transform, accent) {
    const slices = 20;
    const ring = (side) => Array.from({ length: slices }, (_, i) => {
      const angle = i * 2 * Math.PI / slices;
      return transform(axle + radius * Math.cos(angle), center + side * width / 2, height + radius * Math.sin(angle));
    });
    const left = ring(-1), right = ring(1);
    const color = this.colors.tire;
    this.addFace(left, color, 1.45);
    this.addFace([...right].reverse(), color, 1.32);
    const edge = "rgba(112,142,145,.20)";
    for (let i = 0; i < slices; i++) {
      const j = (i + 1) % slices;
      this.addFace([right[i], right[j], left[j], left[i]], color, 1.18 + 0.35 * Math.sin(i * Math.PI / 10), edge);
    }
    for (const side of [-1, 1]) {
      const cap = Array.from({ length: slices }, (_, i) => {
        const a = -side * i * 2 * Math.PI / slices;
        return transform(axle + radius * 0.56 * Math.cos(a), center + side * (width / 2 + 0.0001),
          height + radius * 0.56 * Math.sin(a));
      });
      this.addFace(cap, accent, 0.75);
    }
  }

  shadow(x, y, width, height, opacity = .48) {
    const p = this.project(x, y, 0.001);
    const ctx = this.ctx;
    ctx.save();
    ctx.translate(p[0], p[1]);
    ctx.scale(width * this.scale, height * this.scale * this.alt);
    const gradient = ctx.createRadialGradient(0, 0, .2, 0, 0, 1);
    gradient.addColorStop(0, `rgba(2,7,10,${opacity})`);
    gradient.addColorStop(1, "rgba(2,7,10,0)");
    ctx.fillStyle = gradient;
    ctx.fillRect(-1, -1, 2, 2);
    ctx.restore();
  }

  robot(robot, mine = false) {
    const { robot: model } = LOM.model;
    const color = mine ? this.colors.accent : this.colors[robot.side];
    const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
    const pitch = robot.tipped ? .8 : (robot.pitch || 0);
    const transform = (x, y, z) => {
      const forward = x * Math.cos(pitch) - z * Math.sin(pitch);
      return [robot.x + c * forward - s * y, robot.y + s * forward + c * y, (robot.z || 0) +
        x * Math.sin(pitch) + z * Math.cos(pitch) + y * Math.sin(robot.roll || 0)];
    };
    if (robot.out) return;
    this.shadow(robot.x, robot.y, .18, .15);
    for (const item of model.body) {
      const material = item.material === "accent" ? color : this.colors[item.material];
      this.box(item.at, item.size, material, 0, transform);
      if (item.shape === "upright") {
        for (let row = -12; row <= 12; row++) {
          const hole = [item.at[0] + 0.00505, item.at[1], item.at[2] + row * 0.009];
          this.box(hole, [.0004, .005, .004], "#304851", 0, transform, null);
        }
      }
    }
    for (const side of [-1, 1]) {
      this.tire(model.axleX, side * model.trackWidth / 2, model.wheelRadius, model.wheelRadius,
        model.wheelWidth, transform, color);
      this.box([model.jawX, side * robot.jaws, robot.lift], model.jawSize, color, 0, transform);
    }
    for (const item of model.carriage) {
      this.box([item.at[0], item.at[1], item.at[2] + robot.lift], item.size,
        item.material === "accent" ? color : this.colors[item.material], 0, transform);
    }
    this.box([model.casterX, 0, model.casterRadius], [0.024, 0.024, 0.024], this.colors.dark, 0, transform);
    const [a, b] = [LOM.bodyPoint(robot, .199), LOM.bodyPoint(robot, .238)];
    if (mine && this.alt > .25) this.tape(a, b, .009, "#b8ed6b", robot.z + .003);
  }

  drawField(sim) {
    const { field, lab, layout } = LOM.model;
    const h = field.size / 2;
    this.shadow(0, 0, 1.51, 1.48, .65);
    const square = [[-h, -h], [h, -h], [h, h], [-h, h]];
    for (let i = 0; i < 4; i++) {
      const a = square[i], b = square[(i + 1) % 4];
      const [p, q] = [this.project(...a, 0), this.project(...b, 0)];
      if (p[0] < q[0]) continue;
      this.plane([[...a, 0], [...b, 0], [...b, -field.matHeight], [...a, -field.matHeight]],
        "#172a34", "#36505a");
    }
    this.plane(square, "#20313b", "#628495");
    for (let y = 0; y < 6; y++) {
      for (let x = 0; x < 6; x++) {
        const tile = field.tile;
        const a = (x - 2.5) * tile, b = (y - 2.5) * tile;
        this.rect(a, b, tile - .003, tile - .003, ((x * 3 + y) % 5 < 2) ? "#2e424c" : "#293e48");
        for (let notch = -1; notch <= 1; notch++) {
          this.rect(a + notch * .083, b + tile / 2, .032, .006, "#233740");
          this.rect(a + tile / 2, b + notch * .083, .006, .032, "#233740");
        }
      }
    }
    if (sim.mode === "lab") {
      const angle = lab.rampAngle * Math.PI / 180;
      const height = lab.rampLength * Math.tan(angle);
      const rampX = lab.rampStart + lab.rampLength / 2;
      const transform = (x, y, z) => [rampX + (x - rampX) * Math.cos(angle) - (z - height / 2) *
        Math.sin(angle), y, height / 2 + (x - rampX) * Math.sin(angle) + (z - height / 2) * Math.cos(angle)];
      this.box([rampX, lab.rampY, height / 2 - .010], [lab.rampLength / Math.cos(angle), lab.rampWidth, .02],
        this.colors.ramp, 0, transform);
      this.box([lab.rampStart + lab.rampLength + lab.platformLength / 2, lab.rampY, height / 2],
        [lab.platformLength, lab.rampWidth, height], "#46665e");
      this.box([lab.stepX, lab.stepY, lab.stepHeight / 2], [.105, .30, lab.stepHeight], this.colors.alloy);
      this.tape([-.84, lab.rampY], [-.34, lab.rampY], .012, "#8bd5a6");
      this.tape([-.84, lab.stepY], [.76, lab.stepY], .012, "#8bd5a6");
      return;
    }
    for (const line of layout.lines) this.tape(line.a, line.b, field.lineWidth,
      line.color === "tape" ? "#b7c9c5" : this.colors[line.color]);
    if (sim.hangAllowed || sim.finished) {
      for (const slot of layout.hangSlots) {
        const color = this.colors[slot.side];
        const ring = Array.from({ length: 20 }, (_, i) => {
          const angle = i * Math.PI / 10;
          return [slot.x + .049 * Math.cos(angle), slot.y + .049 * Math.sin(angle), .002];
        });
        this.plane(ring, color + "24", color, 1.1);
        const tip = LOM.bodyPoint(slot, .035);
        this.tape(LOM.bodyPoint(slot, -.025), tip, .008, color, .003);
        this.tape(LOM.bodyPoint(slot, .016, -.015), tip, .007, color, .003);
        this.tape(LOM.bodyPoint(slot, .016, .015), tip, .007, color, .003);
        this.box([slot.barX, slot.barY, slot.barZ + .001], [.016, .016, .002], "#e2eeed", slot.yaw);
      }
    }
    for (const start of layout.starts) {
      const alongX = start.w > start.d;
      const sign = LOM.signOf(start.side);
      const color = this.colors[start.side];
      this.rect(start.x, start.y, start.w, start.d, color + "22", "#bccac7", 0.001);
      if (alongX) this.rect(start.x + sign * (start.w / 2 - .037), start.y, .018, start.d, color, null, .002);
      else this.rect(start.x, start.y + sign * (start.d / 2 - .037), start.w, .018, color, null, .002);
    }
    for (const base of layout.bases) {
      const sign = LOM.signOf(base.side), color = this.colors[base.side];
      const rail = sign * (base.inner - field.baseWidth / 2);
      const length = field.baseInside + field.baseWidth;
      this.rect(base.x, base.y, field.baseInside, field.baseInside, color + "30", color + "65");
      for (const [x, y, w, d] of [[rail, base.y, field.baseWidth, length], [base.x, rail, length, field.baseWidth]]) {
        this.box([x, y, field.baseHeight - .013], [w, d, .026], color);
      }
      for (const at of [base.inner + .008, h - .024]) {
        for (const [x, y] of [[rail, sign * at], [sign * at, rail]]) {
          this.box([x, y, .029], [.014, .014, .058], this.colors.metal);
        }
      }
      if (sim.division === "junior") this.marker(base.x, base.y, sim.tag);
    }
    for (const p of layout.pillars) {
      this.shadow(p.x, p.y, .085, .065);
      this.box([p.x, p.y, .047], [.022, .022, .094], this.colors.metal);
      for (const [i, z] of field.pillarLevels.entries()) {
        const offset = (i * 2 - 1) * field.pillarOffset;
        this.box([p.x + offset, p.y - offset, z - .003], [field.pillarPlate, field.pillarPlate, .006], this.colors.alloy);
      }
    }
    this.shadow(0, 0, .27, .23);
    const z = field.tridentHeight;
    this.box([0, 0, .012], [.107, .074, .024], this.colors.metal, Math.PI / 4);
    this.box([0, 0, z - .002], [.066, .066, .014], this.colors.alloy, Math.PI / 4);
    for (const s of [-1, 1]) {
      this.box([s * .024, s * .024, (z - .028) / 2], [.016, .035, z - .028], this.colors.alloy, Math.PI / 4);
      const x = s * field.tridentCrossAt;
      this.box([x, x, z + .008], [field.tridentCrossSpan, field.tridentBeamWidth, .018],
        this.colors[s < 0 ? "blue" : "red"], -Math.PI / 4);
    }
    this.box([0, 0, z - .009], [field.tridentSpan, field.tridentBeamWidth, .018], this.colors.alloy, Math.PI / 4);
  }

  marker(x, y, id) {
    this.rect(x, y, .065, .065, "#182632", "#adbec4", .003);
    const at = this.project(x, y, .005);
    this.ctx.font = "600 12px ui-monospace, monospace";
    this.ctx.fillStyle = "#ecf4eb";
    this.ctx.textAlign = "center";
    this.ctx.textBaseline = "middle";
    this.ctx.fillText(id, at[0], at[1]);
  }

  drawProp(prop) {
    const f = LOM.model.field;
    const size = prop.kind === "core" ? f.coreSize : [f.cubeSide, f.cubeSide, f.cubeSide];
    const color = this.colors[prop.kind === "core" ? "core" : prop.side || "metal"];
    this.shadow(prop.x, prop.y, size[0], size[1], .43);
    if (prop.kind === "core") {
      for (const angle of [Math.PI / 4, -Math.PI / 4]) {
        this.box([prop.x, prop.y, prop.z], [size[0], .020, size[2]], color, angle);
      }
    } else {
      this.box([prop.x, prop.y, prop.z], size, color);
      this.box([prop.x, prop.y, prop.z + size[2] / 2 + .0005], [size[0] * .51, size[1] * .51, .001],
        prop.side ? color : "#a5c5ce");
    }
    if (prop.origin === "hook" && prop.home) {
      const height = f.tridentHeight - prop.z - size[2] / 2;
      this.box([prop.x, prop.y, f.tridentHeight - height / 2], [.012, .012, height], this.colors.metal);
    }
  }

  drawPickup(sim) {
    const robot = sim.player;
    if ((!sim.manual && !sim.canStartPractice) || robot.held || robot.hook || (robot.closed && !robot.gripMotion)) return;
    const pickup = sim.pickupInfo(robot, robot.gripMotion?.target ?? (robot.plan?.assist ? robot.plan.id : null));
    if (!pickup) return;
    const drop = pickup.prop;
    const [x, y] = this.project(drop.x, drop.y, drop.z);
    const size = Math.max(9, this.scale * LOM.propWidth(drop) * 1.65);
    const ctx = this.ctx;
    ctx.strokeStyle = pickup.ready ? this.colors.accent : "#efc187";
    ctx.lineWidth = 1.6;
    if (pickup.approachYaw !== null && !pickup.railAccess) {
      const rail = { ...drop, yaw: pickup.approachYaw };
      const arrow = this.project(...LOM.bodyPoint(rail, -.205), .002);
      const stem = this.project(...LOM.bodyPoint(rail, -.298), .002);
      ctx.beginPath();
      ctx.moveTo(stem[0], stem[1]);
      ctx.lineTo(arrow[0], arrow[1]);
      for (const side of [-1, 1]) {
        const wing = this.project(...LOM.bodyPoint(rail, -.235, side * .020), .002);
        ctx.moveTo(wing[0], wing[1]);
        ctx.lineTo(arrow[0], arrow[1]);
      }
      ctx.stroke();
    }
    ctx.beginPath();
    for (const sign of [-1, 1]) {
      const edge = x + sign * size;
      ctx.moveTo(edge - sign * 5, y - size);
      ctx.lineTo(edge, y - size);
      ctx.lineTo(edge, y + size);
      ctx.lineTo(edge - sign * 5, y + size);
    }
    ctx.stroke();
  }

  render(sim, inspect = false) {
    const bounds = this.canvas.getBoundingClientRect();
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    if (this.width !== bounds.width || this.height !== bounds.height || this.dpr !== dpr) {
      this.width = bounds.width;
      this.height = bounds.height;
      this.dpr = dpr;
      this.canvas.width = Math.round(bounds.width * dpr);
      this.canvas.height = Math.round(bounds.height * dpr);
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    const { ctx, width, height } = this;
    if (!width || !height) return;
    ctx.clearRect(0, 0, width, height);
    const angle = this.top ? Math.PI / 2 - .0001 : this.elevation;
    const azimuth = this.top ? 0 : this.azimuth;
    this.c = Math.cos(azimuth);
    this.s = Math.sin(azimuth);
    this.alt = Math.sin(angle);
    this.up = Math.cos(angle);
    this.cx = width / 2;
    this.cy = height * (inspect ? .68 : .54);
    const extent = Math.abs(this.c) + Math.abs(this.s);
    this.scale = this.zoom * (inspect ? Math.min(width * 1.24, height * 1.73) :
      Math.min(width * 0.88 / (1.88 * extent), height * 0.75 / (1.88 * extent * this.alt)));
    if (this.top && !inspect) this.cy = height * .50;
    this.faces = [];
    if (inspect) {
      for (let i = -10; i <= 10; i++) {
        const xo = i * .03;
        const grid = i === 0 ? "#547079" : "#273e49";
        this.tape([xo, -.30], [xo, .30], .001, grid);
        this.tape([-.30, xo], [.30, xo], .001, grid);
      }
      this.robot({ x: 0, y: 0, z: 0, yaw: .08, jaws: LOM.model.robot.jawOpen, lift: .098 }, true);
    } else {
      this.drawField(sim);
      for (const prop of sim.props) this.drawProp(prop);
      for (const robot of sim.robots) this.robot(robot, robot.id === 0);
    }
    this.faces.sort((a, b) => b.depth - a.depth);
    for (const face of this.faces) this.polygon(face.points, face.color, face.edge, .58);
    if (!inspect) {
      this.drawPickup(sim);
      for (const robot of sim.robots) {
        if (robot.out) continue;
        const [x, y] = this.project(robot.x, robot.y, robot.z + (this.top ? 0 : .337));
        const color = robot.id ? this.colors[robot.side] : this.colors.accent;
        ctx.font = "700 10px ui-monospace, monospace";
        ctx.textBaseline = "middle";
        ctx.textAlign = "center";
        const suspended = robot.hook && robot.z > .012;
        const tagWidth = suspended ? 47 : 36;
        ctx.fillStyle = "#12212c";
        ctx.strokeStyle = color;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect(x - tagWidth / 2, y - 8, tagWidth, 16, 4);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = color;
        ctx.fillText(suspended ? "↑ " + robot.name : robot.name, x, y + .5);
      }
    }
  }
}
