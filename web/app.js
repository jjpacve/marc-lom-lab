"use strict";

(() => {
  const item = (id) => document.getElementById(id);
  const text = (id, value) => {
    const node = item(id);
    const label = String(value);
    if (node.textContent !== label) node.textContent = label;
  };
  const fieldSession = new LOM.Simulation({ mode: item("mode").value, division: item("division").value });
  const labSession = new LOM.Simulation({ mode: "lab" });
  labSession.settings = fieldSession.settings;
  let sim = fieldSession;
  let tab = "field";
  let drag = null;
  let lastTime = 0;
  let carriedTime = 0;
  let lastHud = 0;
  let eventKey = "";
  let labStage = "";
  let precision = false;
  let dirty = true;
  const keys = new Set();
  const stage = new ArenaView(item("scene"));
  const int = new Intl.NumberFormat("ru-RU", { maximumFractionDigits: 0 });
  const num = (n, decimals = 2) => n.toFixed(decimals).replace(".", ",");
  const minute = (seconds) => [Math.floor(seconds / 60), seconds % 60].map((n) => String(n).padStart(2, "0")).join(":");
  const eventTime = (seconds) => minute(Math.floor(seconds));
  const title = { field: "Тренировочная арена", robot: "Прототип M01", lab: "Испытательный полигон" };
  const phases = { auto: "Автономный этап", driver: "Ручное управление", endgame: "Финальные 30 с", over: "Финиш" };
  const pauseKeys = new Set(["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "KeyW", "KeyA", "KeyS",
    "KeyD", "KeyQ", "KeyE", "KeyF", "KeyH", "Space"]);

  function resetCamera() {
    stage.azimuth = Math.PI / 4;
    stage.elevation = .66;
    stage.zoom = 1;
    stage.top = false;
    setCamera(false);
  }

  function setCamera(top) {
    stage.top = top;
    item("top").setAttribute("aria-pressed", top);
    item("iso").setAttribute("aria-pressed", !top);
    dirty = true;
  }

  function selectTab(target) {
    if (target === tab) return;
    keys.clear();
    sim.running = false;
    tab = target;
    sim = tab === "lab" ? labSession : fieldSession;
    for (const nav of document.querySelectorAll("[data-tab]")) {
      if (nav.dataset.tab === tab) nav.setAttribute("aria-current", "page");
      else nav.removeAttribute("aria-current");
    }
    item("arena-controls").hidden = tab !== "field";
    item("lab-controls").hidden = tab !== "lab";
    item("scoreboard").hidden = tab !== "field";
    item("drive-bar").hidden = tab === "robot";
    item("pilot-panel").hidden = tab === "robot";
    item("robot-panel").hidden = tab !== "robot";
    item("trial-stats").hidden = tab !== "lab";
    item("field-missions").hidden = tab !== "field";
    item("hang-controls").hidden = tab !== "field";
    text("page-title", title[tab]);
    text("scene-label", tab === "robot" ? "Сборка M01" : tab === "lab" ? "Полигон M01" : "Undersea Core Contest");
    text("scene-size", tab === "robot" ? "312 × 214 × 289 мм" : "1880 × 1880 мм");
    item("scene").setAttribute("aria-label", tab === "robot" ? "Трёхмерная модель M01" : "Трёхмерная модель поля и роботов");
    resetCamera();
    updateHud();
  }

  function resetRun() {
    keys.clear();
    carriedTime = 0;
    if (tab === "lab") {
      labSession.setLab(item("course").value);
      labStage = "";
    } else {
      fieldSession.reset({
        mode: item("mode").value,
        division: item("division").value,
        tag: 1 + Math.floor(Math.random() * 3),
      });
      if (fieldSession.mode === "practice") manualPace();
    }
    dirty = true;
    updateHud();
  }

  function toggleRun() {
    if (tab === "robot") return;
    keys.clear();
    if (sim.finished) resetRun();
    if (sim.running) sim.running = false;
    else {
      sim.start();
      if (!sim.time && sim.manual) manualPace();
      item("scene").focus({ preventScroll: true });
    }
    dirty = true;
    updateHud();
  }

  function toggleAuto() {
    if (sim.finished) return;
    keys.clear();
    if (tab === "lab" && !sim.auto) {
      resetRun();
      labStage = item("course").value === "grip" ? "approach" : "drive";
    }
    sim.cancelPickup(sim.player);
    sim.auto = !sim.auto;
    sim.player.plan = null;
    if (sim.auto) sim.start();
    else if (sim.manual) manualPace();
    updateHud();
  }

  function manualPace() {
    item("rate").value = "1";
    carriedTime = 0;
  }

  function toggleGrip() {
    if (sim.canStartPractice) toggleRun();
    if (!sim.manual) return;
    const robot = sim.player;
    if (sim.manualGrip(robot) && (robot.plan?.assist || robot.gripMotion)) keys.clear();
    dirty = true;
    updateHud();
  }

  function setPickupHeight() {
    const robot = sim.player;
    if (!sim.manual || robot.hook || robot.held || robot.gripMotion || robot.plan?.assist) return;
    const pickup = sim.pickupInfo(robot);
    if (!pickup) return;
    const design = LOM.model.robot;
    robot.liftTarget = LOM.clamp(pickup.height, design.liftMin, design.liftMax);
    updateHud();
  }

  function toggleHang() {
    if (!sim.manual || tab !== "field") return;
    keys.clear();
    sim.cancelPickup(sim.player);
    sim.toggleHang(sim.player);
    dirty = true;
    updateHud();
  }

  function held(...codes) {
    return Number(codes.some((key) => keys.has(key)));
  }

  function manualInput() {
    const scale = precision || held("ShiftLeft", "ShiftRight") ? 0.35 : 1;
    return {
      speed: scale * (held("KeyW", "ArrowUp") - held("KeyS", "ArrowDown")),
      turn: scale * (held("KeyA", "ArrowLeft") - held("KeyD", "ArrowRight")),
      lift: scale * (held("KeyE") - held("KeyQ")),
    };
  }

  function stopLab(message) {
    sim.auto = false;
    sim.running = false;
    sim.log(message);
  }

  function labInput() {
    const robot = sim.player;
    const idle = { speed: 0, turn: 0, lift: 0 };
    if (robot.out || robot.tipped || sim.time > 12) {
      stopLab("Заезд остановлен");
      return idle;
    }
    if (labStage === "drive") {
      const ramp = item("course").value === "ramp";
      if (!ramp && robot.blocked) {
        stopLab("Робот остановился у порога");
        return idle;
      }
      const target = ramp ? .26 : .63;
      if (robot.x < target) return { ...idle, speed: 0.63 };
      stopLab(robot.margin > 0 ? "Препятствие пройдено" : "Потеря устойчивости");
    } else if (labStage === "approach") {
      const x = LOM.model.lab.cube[0] - LOM.model.robot.gripReach;
      if (robot.x < x - 0.009) return { ...idle, speed: LOM.clamp(5 * (x - robot.x), .13, .64) };
      sim.grip(robot, true);
      labStage = "close";
    } else if (labStage === "close") {
      if (robot.gripMotion) return idle;
      if (!robot.held) stopLab("Подъезд к кубу не удался");
      else {
        robot.liftTarget = 0.145;
        labStage = "carry";
      }
    } else if (labStage === "carry") {
      if (robot.lift < .137) return idle;
      if (robot.x < -.14) return { ...idle, speed: 0.51 };
      sim.grip(robot, false);
      labStage = "land";
    } else if (labStage === "land" && sim.props[0].z < .027) {
      stopLab("Куб поднят, перемещён и отпущен");
    }
    return idle;
  }

  function updateEvents() {
    const key = sim.events.map((event) => event.time + event.text).join("|");
    if (eventKey === key) return;
    eventKey = key;
    const list = item("event-list");
    list.replaceChildren();
    if (!sim.events.length) {
      const row = document.createElement("li");
      row.className = "empty";
      row.textContent = "События появятся после старта.";
      list.append(row);
      return;
    }
    for (const event of sim.events.slice(0, 4)) {
      const row = document.createElement("li");
      const stamp = document.createElement("span");
      stamp.className = "event-time";
      stamp.textContent = eventTime(event.time);
      row.append(stamp, event.text);
      list.append(row);
    }
  }

  function updatePickup(robot, handoff) {
    const assisting = Boolean(robot.plan?.assist);
    const grip = item("grip");
    const status = robot.grasp?.clearance ? "Снимает с подвеса" :
      robot.held ? (robot.held.kind === "core" ? "Ядро" : "Куб") :
      robot.gripMotion?.openFirst ? "Открывается" : robot.gripMotion ? "Закрывается" :
      assisting ? "Подводится" : robot.closed ? "Закрыт · пусто" :
      robot.jaws < LOM.model.robot.jawOpen - 0.002 ? "Открывается" : "Открыт";
    text("grip-status", status);
    const hint = item("pickup-hint");
    const firstPickup = sim.canStartPractice;
    const inControl = sim.manual && !robot.hook;
    const pickup = inControl && !robot.held ?
      sim.pickupInfo(robot, robot.gripMotion?.target ?? (assisting ? robot.plan.id : null)) : null;
    const canAssist = Boolean(sim.pickupAssistGoal(robot, pickup));
    const canGrip = robot.held || robot.closed || assisting || pickup?.aligned || canAssist || sim.mode === "lab";
    const showPickup = inControl || firstPickup || handoff;
    item("target-height").disabled = !pickup || assisting || Boolean(robot.gripMotion);
    hint.hidden = !showPickup;
    grip.hidden = !showPickup || handoff;
    grip.disabled = !firstPickup && !(inControl && canGrip);
    text("grip", assisting ? "Отменить · пробел" : robot.held ? "Отпустить · пробел" :
      robot.closed ? "Открыть · пробел" : pickup?.prop.kind === "core" ? "Взять ядро · пробел" : "Взять куб · пробел");
    hint.classList.toggle("ready", firstPickup || canAssist || Boolean(pickup?.ready && !robot.closed));
    if (!showPickup) return;
    text("pickup-hint", firstPickup ? "Первый куб уже перед M01." :
      handoff ? (robot.out ? "Верни M01 на старт, затем продолжи вручную." : "Матч на паузе до продолжения.") :
      robot.grasp?.clearance ? "M01 снимает куб с подвеса." :
      robot.held ? (robot.held.kind === "core" ? "Ядро в захвате." : "Куб в захвате.") :
      assisting ? "M01 подводит захват." :
      robot.gripMotion?.openFirst ? "Губки раскрываются." : robot.gripMotion ? "Губки смыкаются." :
      robot.closed ? "Захват пуст. Открой губки для новой попытки." :
      !pickup ? "Подъедь к кубу, чтобы активировать захват." :
      canAssist ? "M01 сам подведёт захват к подсвеченному предмету." :
      pickup.aligned ? "Предмет в зоне захвата." :
      !pickup.railAccess ? "Объедь трезубец и подъедь по стрелке." :
      !pickup.centered ? "Направь захват к предмету." :
      pickup.forward > pickup.max ? "Подъедь ближе." : pickup.forward < pickup.min ? "Немного сдай назад." :
      "F выставит высоту " + int.format(pickup.height * 1000) + " мм.");
  }

  function updateHud() {
    if (tab === "robot") return;
    const robot = sim.player;
    const phase = sim.phase;
    const match = sim.mode === "match";
    const practice = sim.mode === "practice";
    const active = sim.running;
    const paused = !active && sim.time > 0 && !sim.finished;
    const { duration, exploration } = LOM.model.round;
    const handoff = paused && match && !sim.auto && sim.division !== "club" && sim.time === exploration;
    text("blue-score", sim.score.blue);
    text("red-score", sim.score.red);
    text("clock", match ? minute(Math.max(0, Math.ceil(duration - sim.time))) : "—");
    text("clock-caption", sim.finished ? phases.over : handoff ? "Твой ход" : paused ? "Пауза" :
      practice ? "Без таймера" : !active ? "До старта" : match ? phases[phase] : "Полигон");
    item("phase-bar").hidden = !match;
    item("clock-marker").style.left = (match ? Math.min(100, sim.time * 100 / duration) : 0) + "%";
    item("live-dot").classList.toggle("live", active);
    text("speed", num(Math.abs(robot.speed)));
    text("path", num(robot.path));
    text("captures", int.format(robot.captures));
    text("lift-height", int.format(robot.lift * 1000) + " мм");
    text("peak", num(robot.peak) + " м/с");
    text("tilt", num(robot.maxTilt * 180 / Math.PI, 1) + "°");
    text("pilot-status", robot.out ? "Вне поля" : robot.tipped ? "Опрокинулся" : robot.hookPhase === "hanging" ?
      "На перекладине" : robot.hook ? "Крюк задействован" : active ? "На поле" : paused ? "Пауза" : "Ожидание");
    updatePickup(robot, handoff);
    const earlyAuto = phase === "auto" && sim.division !== "club";
    text("control-mode", sim.finished ? "Финиш" :
      handoff ? "Твой ход" : !active ? paused ? "Пауза" : "Старт" : earlyAuto || sim.auto ? "Автономно" : "Ручное");
    const tag = sim.division === "junior" ? " Цель AprilTag № " + sim.tag + "." : "";
    text("control-hint", tab === "lab" ? "Выполните испытание вручную или запустите контрольный заезд." :
      earlyAuto ? "Первые 30 секунд робот едет автономно." + (sim.auto ? "" : " Затем матч встанет на паузу. Один подвесной куб боты оставят тебе.") + tag :
      match && phase === "auto" ? "Дивизион Club допускает управление с начала матча. Один подвесной куб боты оставят тебе." :
      robot.hook ? "Нажмите H, чтобы опустить робота." :
      sim.hangAllowed ? "Подъедьте к метке своей перекладины, направьте крюк к ней и нажмите H." :
      "Ядро берётся с высоты около 250 мм. Подвес открыт после снятия предметов с трезубца или в финальные 30 с.");
    item("control-hint").hidden = sim.finished || handoff || (practice && !robot.captures);
    if (tab === "field") {
      const core = sim.props.find((prop) => prop.kind === "core");
      const carrier = core.heldBy !== null ? sim.robots[core.heldBy] : null;
      const grounded = core.z <= LOM.propHalf(core) + .007;
      text("core-status", core.home ? "На трезубце" : carrier ? "Несёт " + carrier.name :
        grounded && LOM.inBase(core, "blue", LOM.propWidth(core)) ? "На синей базе" :
        grounded && LOM.inBase(core, "red", LOM.propWidth(core)) ? "На красной базе" : "В поле");
      const hanging = (side) => sim.robots.filter((r) => r.hook?.side === side && r.z > .012).length;
      const blue = hanging("blue"), red = hanging("red");
      text("hang-status", blue || red || sim.finished ? `Синие ${blue} · Красные ${red}` :
        sim.hangAllowed ? (sim.nearbyHook(robot) && !robot.held ? "Готов к зацепу" : "Доступен") : "Пока закрыт");
    }
    item("precision").disabled = !sim.manual || Boolean(robot.hook);
    item("precision").setAttribute("aria-pressed", precision || Boolean(held("ShiftLeft", "ShiftRight")));
    item("hang").disabled = !sim.manual;
    const auto = item("auto");
    auto.disabled = sim.finished;
    auto.setAttribute("aria-pressed", sim.auto);
    auto.textContent = tab === "lab" ? (sim.auto ? "Остановить заезд" : "Контрольный заезд") : "Автопилот";
    item("reset-card").hidden = !(robot.out && !robot.reset && phase === "driver");
    item("result").hidden = !sim.finished;
    text("winner", sim.score.blue === sim.score.red ? "Ничья" :
      sim.score.blue > sim.score.red ? "Победа синих" : "Победа красных");
    text("play", active ? "Ⅱ  Пауза" : sim.finished ? "▶  Новый матч" : handoff ? "▶  Продолжить вручную" :
      paused ? "▶  Продолжить" : match ? "▶  Начать матч" : practice ? "▶  Начать тренировку" : "▶  Начать заезд");
    item("export").disabled = sim.samples.length === 0;
    item("division-label").hidden = !match;
    updateEvents();
  }

  function draw(timestamp) {
    const elapsed = lastTime ? Math.min(0.13, (timestamp - lastTime) / 1000) : 0;
    lastTime = timestamp;
    if (sim.running && tab !== "robot") {
      carriedTime += elapsed * Number(item("rate").value);
      while (carriedTime >= LOM.dt && sim.running) {
        const exploration = sim.phase === "auto" && sim.division !== "club" && !sim.auto;
        sim.tick(tab === "lab" && sim.auto ? labInput() : manualInput());
        carriedTime -= LOM.dt;
        if (exploration && sim.phase === "driver") {
          sim.running = false;
          keys.clear();
          manualPace();
          break;
        }
      }
      dirty = true;
    }
    if (timestamp - lastHud > 110) {
      updateHud();
      lastHud = timestamp;
    }
    if (dirty) {
      stage.render(sim, tab === "robot");
      dirty = false;
    }
    requestAnimationFrame(draw);
  }

  for (const nav of document.querySelectorAll("[data-tab]")) nav.addEventListener("click", () => selectTab(nav.dataset.tab));
  for (const id of ["mode", "division", "course"]) item(id).addEventListener("change", resetRun);
  item("play").addEventListener("click", toggleRun);
  item("reset").addEventListener("click", resetRun);
  item("again").addEventListener("click", resetRun);
  item("auto").addEventListener("click", toggleAuto);
  item("grip").addEventListener("click", toggleGrip);
  item("target-height").addEventListener("click", setPickupHeight);
  item("precision").addEventListener("click", () => {
    precision = !precision;
    updateHud();
  });
  item("hang").addEventListener("click", toggleHang);
  item("reset-card").addEventListener("click", () => {
    sim.resetRobot(sim.player);
    dirty = true;
    updateHud();
  });
  item("iso").addEventListener("click", () => setCamera(false));
  item("top").addEventListener("click", () => setCamera(true));
  item("home-view").addEventListener("click", resetCamera);
  item("export").addEventListener("click", () => {
    const csv = [LOM.csvHead, ...sim.samples.map((row) => row.map((n) =>
      typeof n === "number" ? n.toFixed(4) : n).join(","))].join("\n");
    const url = URL.createObjectURL(new Blob([csv], { type: "text/csv;charset=utf-8" }));
    const a = document.createElement("a");
    a.href = url;
    a.download = "marc-lom-" + sim.mode + ".csv";
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });

  for (const [id, key, format] of [
    ["speed-limit", "speed", (v) => num(v) + " м/с"], ["friction", "grip", num],
    ["mass", "mass", (v) => num(v) + " кг"], ["cg", "cg", (v) => int.format(v * 1000) + " мм"],
  ]) {
    item(id).addEventListener("input", () => {
      const value = Number(item(id).value);
      fieldSession.settings[key] = value;
      text(id + "-value", format(value));
    });
  }
  for (const button of document.querySelectorAll("[data-hold]")) {
    const set = (pressed) => {
      if (pressed) keys.add(button.dataset.hold);
      else keys.delete(button.dataset.hold);
      button.classList.toggle("pressed", pressed);
    };
    button.addEventListener("pointerdown", (event) => {
      if (!sim.manual) return;
      button.setPointerCapture(event.pointerId);
      set(true);
    });
    for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) {
      button.addEventListener(event, () => set(false));
    }
  }
  window.addEventListener("keydown", (event) => {
    if (tab === "robot" || event.altKey || event.ctrlKey || event.metaKey ||
        ["SELECT", "INPUT"].includes(document.activeElement.tagName)) return;
    if (event.code === "Escape" && sim.running) {
      toggleRun();
      return;
    }
    if (event.code === "ShiftLeft" || event.code === "ShiftRight") {
      if (sim.manual) keys.add(event.code);
      return;
    }
    if (!pauseKeys.has(event.code)) return;
    event.preventDefault();
    if (event.code === "Space" && !event.repeat) toggleGrip();
    else if (event.code === "KeyF" && !event.repeat) setPickupHeight();
    else if (event.code === "KeyH" && !event.repeat) toggleHang();
    else if (sim.manual && !event.repeat) keys.add(event.code);
  });
  window.addEventListener("keyup", (event) => keys.delete(event.code));
  window.addEventListener("blur", () => {
    keys.clear();
    for (const button of document.querySelectorAll(".keys .pressed")) button.classList.remove("pressed");
  });
  const canvas = item("scene");
  canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0 || stage.top) return;
    canvas.setPointerCapture(event.pointerId);
    drag = { x: event.clientX, y: event.clientY };
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!drag) return;
    stage.azimuth -= (event.clientX - drag.x) * .008;
    stage.elevation = LOM.clamp(stage.elevation + (event.clientY - drag.y) * .006, .30, 1.40);
    drag = { x: event.clientX, y: event.clientY };
    dirty = true;
  });
  canvas.addEventListener("lostpointercapture", () => { drag = null; });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    stage.zoom = LOM.clamp(stage.zoom * Math.exp(-event.deltaY * .0014), .48, 2.5);
    dirty = true;
  }, { passive: false });
  new ResizeObserver(() => { dirty = true; }).observe(canvas);
  updateHud();
  requestAnimationFrame(draw);
})();
