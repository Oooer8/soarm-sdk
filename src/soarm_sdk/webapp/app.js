const ui = {
  pages: document.querySelectorAll("[data-page]"),
  configPath: document.querySelector("#configPath"),
  mockToggle: document.querySelector("#mockToggle"),
  armLabel: document.querySelector("#armLabel"),
  portLabel: document.querySelector("#portLabel"),
  baudLabel: document.querySelector("#baudLabel"),
  resultLabel: document.querySelector("#resultLabel"),
  armName: document.querySelector("#armName"),
  portSelect: document.querySelector("#portSelect"),
  manualPort: document.querySelector("#manualPort"),
  scanPorts: document.querySelector("#scanPorts"),
  saveConfig: document.querySelector("#saveConfig"),
  frequencyForm: document.querySelector("#frequencyForm"),
  frequencySummary: document.querySelector("#frequencySummary"),
  trajectoryInputHz: document.querySelector("#trajectoryInputHz"),
  controlHz: document.querySelector("#controlHz"),
  serialWriteHz: document.querySelector("#serialWriteHz"),
  feedbackHz: document.querySelector("#feedbackHz"),
  webMotionHz: document.querySelector("#webMotionHz"),
  webStateHz: document.querySelector("#webStateHz"),
  webRenderHz: document.querySelector("#webRenderHz"),
  saveFrequencies: document.querySelector("#saveFrequencies"),
  identityForm: document.querySelector("#identityForm"),
  checkStatus: document.querySelector("#checkStatus"),
  messageBox: document.querySelector("#messageBox"),
  jointGrid: document.querySelector("#jointGrid"),
  reportMode: document.querySelector("#reportMode"),
  reportText: document.querySelector("#reportText"),
  stepStatus: document.querySelector("#stepStatus"),
  stepCalibration: document.querySelector("#stepCalibration"),
  stepControl: document.querySelector("#stepControl"),
  modelLock: document.querySelector("#modelLock"),
  robotStage: document.querySelector("#robotStage"),
  urdfViewport: document.querySelector("#urdfViewport"),
  modelMode: document.querySelector("#modelMode"),
  modelPose: document.querySelector("#modelPose"),
  startCalibration: document.querySelector("#startCalibration"),
  resetCalibration: document.querySelector("#resetCalibration"),
  captureZero: document.querySelector("#captureZero"),
  startSweep: document.querySelector("#startSweep"),
  stopSweep: document.querySelector("#stopSweep"),
  saveCalibration: document.querySelector("#saveCalibration"),
  calibrationState: document.querySelector("#calibrationState"),
  calibrationGate: document.querySelector("#calibrationGate"),
  calibrationInstruction: document.querySelector("#calibrationInstruction"),
  calibrationRows: document.querySelector("#calibrationRows"),
  calibrationReport: document.querySelector("#calibrationReport"),
  jointPanel: document.querySelector("#jointPanel"),
  jointSliders: document.querySelector("#jointSliders"),
  readState: document.querySelector("#readState"),
  torqueToggle: document.querySelector("#torqueToggle"),
  syncState: document.querySelector("#syncState"),
  resetPose: document.querySelector("#resetPose"),
  motionPanel: document.querySelector("#motionPanel"),
  solveIk: document.querySelector("#solveIk"),
  ikX: document.querySelector("#ikX"),
  ikY: document.querySelector("#ikY"),
  ikZ: document.querySelector("#ikZ"),
  fkReport: document.querySelector("#fkReport"),
  robot: {
    turntable: document.querySelector("#turntable"),
    linkUpper: document.querySelector("#linkUpper"),
    linkForearm: document.querySelector("#linkForearm"),
    linkWrist: document.querySelector("#linkWrist"),
    toolPalm: document.querySelector("#toolPalm"),
    fingerTop: document.querySelector("#fingerTop"),
    fingerBottom: document.querySelector("#fingerBottom"),
    jointBase: document.querySelector("#jointBase"),
    jointShoulder: document.querySelector("#jointShoulder"),
    jointElbow: document.querySelector("#jointElbow"),
    jointWrist: document.querySelector("#jointWrist"),
  },
};

let config = null;
let workflow = {
  phase: "status",
  status_passed: false,
  calibration_ready: false,
  calibrated: false,
  control_ready: false,
};
let busy = false;
let robotModel = null;
let jointPositions = {};
let modelJointPositions = {};
let hardwareJointPositions = {};
let hardwareModelJointPositions = {};
let jointTorqueEnabled = false;
let urdfViewer = null;
let urdfViewerModulePromise = null;
let liveStateTimer = null;
let liveStateInFlight = false;
let liveStateErrorShown = false;
let motionCommandTimer = null;
let motionCommandInFlight = false;
let pendingMotionCommand = null;

const DEFAULT_FREQUENCIES = {
  trajectory_input_hz: 60,
  control_hz: 200,
  serial_write_hz: 200,
  feedback_hz: 100,
  web_motion_hz: 30,
  web_state_hz: 30,
  web_render_hz: 60,
};

let runtimeFrequencies = { ...DEFAULT_FREQUENCIES };
let liveStateIntervalMs = intervalFromHz(runtimeFrequencies.web_state_hz);
let motionCommandIntervalMs = intervalFromHz(runtimeFrequencies.web_motion_hz);

const calibration = {
  started: false,        // torque disabled, session open
  zeroTicks: null,       // {name: tick} after captureZero
  sweeping: false,       // sweep is running in background
  sweepResults: null,    // [{name, zero_tick, inferred_direction, range_ticks, ...}] after stopSweep
};

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.error || `Request failed: ${response.status}`);
  }
  return payload;
}

function setBusy(isBusy) {
  busy = isBusy;
  updateWorkflowUI();
  updateCalibrationUI();
  updateJointControls();
}

function setMessage(text, isError = false) {
  ui.messageBox.textContent = text;
  ui.messageBox.classList.toggle("error", isError);
}

function normalizeView(view) {
  if (view === "control" || view === "motion") return "visualizer";
  if (view === "calibration" || view === "visualizer") return view;
  return "status";
}

function canNavigateTo(view) {
  const target = normalizeView(view);
  if (target === "status") return true;
  if (target === "calibration") return Boolean(workflow.calibration_ready);
  if (target === "visualizer") return Boolean(workflow.control_ready);
  return false;
}

function updateStepElement(element, target, stateClass) {
  const available = canNavigateTo(target);
  element.className = [stateClass, available ? "available" : "locked", busy ? "busy" : ""]
    .filter(Boolean)
    .join(" ");
  element.dataset.stepTarget = target;
  element.setAttribute("aria-disabled", String(!available || busy));
  element.tabIndex = available ? 0 : -1;
}

function viewFromWorkflow() {
  if (workflow.control_ready) return "visualizer";
  if (!workflow.calibration_ready) return "status";
  return "calibration";
}

function navigateTo(view) {
  const nextView = normalizeView(view);
  if (!canNavigateTo(nextView)) {
    updateWorkflowUI();
    return false;
  }
  document.body.dataset.view = nextView;
  ui.pages.forEach((page) => {
    page.hidden = page.dataset.page !== nextView;
  });
  updateWorkflowUI();
  return true;
}

async function goToStep(view) {
  const target = normalizeView(view);
  if (busy || !canNavigateTo(target)) {
    updateWorkflowUI();
    return;
  }
  if (target === "visualizer" && !robotModel) {
    setBusy(true);
    setMessage("Loading SOARM model...");
    try {
      await loadRobotModel();
    } catch (error) {
      setMessage(error.message, true);
      return;
    } finally {
      setBusy(false);
    }
  }
  navigateTo(target);
}

function portFromInputs() {
  return ui.manualPort.value.trim() || ui.portSelect.value.trim();
}

function allCalibrationComplete() {
  return Array.isArray(calibration.sweepResults) && calibration.sweepResults.length > 0;
}

function initialPositions() {
  const home = config?.poses?.home || {};
  return Object.fromEntries((config?.joints || []).map((joint) => [joint.name, Number(home[joint.name] || 0)]));
}

function updateHeader() {
  if (!config) return;
  ui.configPath.textContent = config.config_path;
  ui.armLabel.textContent = config.arm.name || "-";
  ui.portLabel.textContent = config.arm.port || "Not set";
  ui.baudLabel.textContent = String(config.arm.baudrate || "-");
  ui.armName.value = config.arm.name || "";
  ui.manualPort.value = config.arm.port || "";
}

function updateWorkflowUI() {
  let view = normalizeView(document.body.dataset.view || viewFromWorkflow());
  if (!canNavigateTo(view)) view = viewFromWorkflow();
  document.body.dataset.view = view;
  document.body.dataset.calibrated = String(Boolean(workflow.calibrated));
  document.body.dataset.controlReady = String(Boolean(workflow.control_ready));
  document.body.dataset.statusPassed = String(Boolean(workflow.status_passed));
  document.body.dataset.calibrationReady = String(Boolean(workflow.calibration_ready));
  ui.pages.forEach((page) => {
    page.hidden = page.dataset.page !== view;
  });

  updateStepElement(
    ui.stepStatus,
    "status",
    view === "status" ? "active" : workflow.calibration_ready ? "done" : "",
  );
  updateStepElement(
    ui.stepCalibration,
    "calibration",
    view === "calibration" ? "active" : workflow.calibrated ? "done" : "",
  );
  updateStepElement(
    ui.stepControl,
    "visualizer",
    view === "visualizer" ? "active" : workflow.control_ready ? "done" : "",
  );

  const controlDisabled = busy;
  ui.scanPorts.disabled = controlDisabled;
  ui.saveConfig.disabled = controlDisabled;
  ui.saveFrequencies.disabled = controlDisabled;
  ui.checkStatus.disabled = controlDisabled;

  const calibrationLocked = !workflow.calibration_ready;
  ui.calibrationGate.hidden = !calibrationLocked;
  ui.startCalibration.disabled = busy || calibrationLocked;

  const controlLocked = !workflow.control_ready;
  if (controlLocked) {
    jointTorqueEnabled = false;
    cancelPendingMotionCommand();
  }
  ui.jointPanel.classList.toggle("locked", controlLocked);
  ui.motionPanel.classList.toggle("locked", controlLocked);
  ui.readState.disabled = busy || controlLocked;
  ui.torqueToggle.disabled = busy || controlLocked;
  ui.torqueToggle.checked = Boolean(jointTorqueEnabled && !controlLocked);
  ui.solveIk.disabled = busy || controlLocked;
  ui.modelLock.classList.toggle("hidden", !controlLocked);
  ui.robotStage.classList.toggle("loaded", !controlLocked);
  updateLiveStateLoop();
}

function isVisualizerVisible() {
  return normalizeView(document.body.dataset.view || viewFromWorkflow()) === "visualizer";
}

function updateLiveStateLoop() {
  const shouldRun = Boolean(workflow.control_ready && isVisualizerVisible());
  if (shouldRun && !liveStateTimer) {
    liveStateTimer = setInterval(pollLiveState, liveStateIntervalMs);
    pollLiveState({ force: true });
  } else if (!shouldRun && liveStateTimer) {
    clearInterval(liveStateTimer);
    liveStateTimer = null;
  }
}

function intervalFromHz(value) {
  const hz = Math.max(1, Number(value) || 1);
  return Math.max(1, Math.round(1000 / hz));
}

function normalizeFrequencies(values = {}) {
  const merged = { ...DEFAULT_FREQUENCIES, ...(values || {}) };
  const controlHz = Math.max(1, Number(merged.control_hz) || DEFAULT_FREQUENCIES.control_hz);
  return {
    trajectory_input_hz: Math.max(1, Number(merged.trajectory_input_hz) || DEFAULT_FREQUENCIES.trajectory_input_hz),
    control_hz: controlHz,
    serial_write_hz: controlHz,
    feedback_hz: Math.max(1, Number(merged.feedback_hz) || DEFAULT_FREQUENCIES.feedback_hz),
    web_motion_hz: Math.max(1, Number(merged.web_motion_hz) || DEFAULT_FREQUENCIES.web_motion_hz),
    web_state_hz: Math.max(1, Number(merged.web_state_hz) || DEFAULT_FREQUENCIES.web_state_hz),
    web_render_hz: Math.max(1, Number(merged.web_render_hz) || DEFAULT_FREQUENCIES.web_render_hz),
  };
}

function applyFrequencySettings(values = {}) {
  const wasPolling = Boolean(liveStateTimer);
  if (wasPolling) {
    clearInterval(liveStateTimer);
    liveStateTimer = null;
  }

  runtimeFrequencies = normalizeFrequencies(values);
  motionCommandIntervalMs = intervalFromHz(runtimeFrequencies.web_motion_hz);
  liveStateIntervalMs = intervalFromHz(runtimeFrequencies.web_state_hz);
  if (urdfViewer?.setRenderHz) urdfViewer.setRenderHz(runtimeFrequencies.web_render_hz);
  renderFrequencySettings();
  if (wasPolling) updateLiveStateLoop();
}

function renderFrequencySettings() {
  ui.trajectoryInputHz.value = String(runtimeFrequencies.trajectory_input_hz);
  ui.controlHz.value = String(runtimeFrequencies.control_hz);
  ui.serialWriteHz.value = String(runtimeFrequencies.serial_write_hz);
  ui.feedbackHz.value = String(runtimeFrequencies.feedback_hz);
  ui.webMotionHz.value = String(runtimeFrequencies.web_motion_hz);
  ui.webStateHz.value = String(runtimeFrequencies.web_state_hz);
  ui.webRenderHz.value = String(runtimeFrequencies.web_render_hz);
  ui.frequencySummary.textContent =
    `轨迹 ${runtimeFrequencies.trajectory_input_hz} Hz · ` +
    `控制/下发 ${runtimeFrequencies.control_hz} Hz · ` +
    `反馈 ${runtimeFrequencies.feedback_hz} Hz · ` +
    `Web ${runtimeFrequencies.web_motion_hz}/${runtimeFrequencies.web_state_hz} Hz · ` +
    `渲染 ${runtimeFrequencies.web_render_hz} Hz`;
}

function frequencyPayloadFromInputs() {
  return normalizeFrequencies({
    trajectory_input_hz: ui.trajectoryInputHz.value,
    control_hz: ui.controlHz.value,
    feedback_hz: ui.feedbackHz.value,
    web_motion_hz: ui.webMotionHz.value,
    web_state_hz: ui.webStateHz.value,
    web_render_hz: ui.webRenderHz.value,
  });
}

function renderPortOptions(ports) {
  ui.portSelect.innerHTML = "";
  if (!ports.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "No candidate ports found";
    ui.portSelect.append(option);
    return;
  }
  for (const port of ports) {
    const option = document.createElement("option");
    option.value = port;
    option.textContent = port;
    ui.portSelect.append(option);
  }
  if (ports.length === 1) {
    ui.portSelect.value = ports[0];
    ui.manualPort.value = ports[0];
  } else if (config?.arm?.port && ports.includes(config.arm.port)) {
    ui.portSelect.value = config.arm.port;
  }
}

function emptyJointState(joint) {
  return {
    name: joint.name,
    id: joint.id,
    communication: "idle",
    position: "idle",
    soft_limit: "idle",
    voltage: "idle",
  };
}

function parseStatus(lines) {
  const byName = new Map((config?.joints || []).map((joint) => [joint.name, emptyJointState(joint)]));
  let current = null;
  for (const line of lines) {
    const jointMatch = line.match(/^- ([A-Za-z0-9_]+) \(id=(\d+)\)/);
    if (jointMatch) {
      current = byName.get(jointMatch[1]);
      continue;
    }
    if (!current) continue;
    for (const key of ["communication", "position", "soft_limit", "voltage"]) {
      if (line.trim().startsWith(`${key}:`)) {
        current[key] = line.includes("[PASS]") ? "pass" : line.includes("[FAIL]") ? "fail" : "idle";
      }
    }
  }
  return Array.from(byName.values());
}

function renderJoints(joints) {
  ui.jointGrid.innerHTML = "";
  for (const joint of joints) {
    const card = document.createElement("article");
    card.className = "joint-card";

    const title = document.createElement("h3");
    title.textContent = joint.name;
    const servo = document.createElement("span");
    servo.className = "servo-id";
    servo.textContent = `id ${joint.id}`;

    const checks = document.createElement("div");
    checks.className = "check-list";
    for (const [label, key] of [
      ["Comm", "communication"],
      ["Pos", "position"],
      ["Limit", "soft_limit"],
      ["Volt", "voltage"],
    ]) {
      const pill = document.createElement("span");
      const state = joint[key] || "idle";
      pill.className = `check-pill ${state === "pass" || state === "fail" ? state : ""}`;
      pill.innerHTML = `<span>${label}</span><strong>${state.toUpperCase()}</strong>`;
      checks.append(pill);
    }
    card.append(title, servo, checks);
    ui.jointGrid.append(card);
  }
}

function tickText(value) {
  if (value === undefined || value === null) return "未采集";
  return String(value);
}

function renderCalibrationRows() {
  ui.calibrationRows.innerHTML = "";
  for (const joint of config?.joints || []) {
    const result = (calibration.sweepResults || []).find((r) => r.name === joint.name);
    const row = document.createElement("tr");
    const done = Boolean(result);
    const warn = result && !result.well_excited;
    row.innerHTML = `
      <td>${joint.name}<span>id ${joint.id}</span></td>
      <td>${result ? result.zero_tick : calibration.zeroTicks ? calibration.zeroTicks[joint.name] ?? "—" : "—"}</td>
      <td>${result ? (result.inferred_direction > 0 ? "+1" : "−1") : "—"}</td>
      <td>${result ? `${result.raw_min_tick}..${result.raw_max_tick} (Δ${result.range_ticks})${warn ? " ⚠" : ""}` : "—"}</td>
      <td>${result ? `${result.safe_min_tick}..${result.safe_max_tick}` : "—"}</td>
      <td>${result ? "[" + result.safe_min_rad.toFixed(4) + ", " + result.safe_max_rad.toFixed(4) + "]" : "—"}</td>
      <td><mark class="${done ? (warn ? "warn" : "done") : calibration.sweeping ? "current" : ""}"
        >${done ? (warn ? "⚠ 欠激励" : "Ready") : calibration.sweeping ? "录制中…" : "Pending"}</mark></td>
    `;
    ui.calibrationRows.append(row);
  }
}

function nextCalibrationInstruction() {
  if (!workflow.calibration_ready) return "先完成串口保存和状态检查。";
  if (!calibration.started) return "点击【开始】禁用扭矩后，把机械臂移动到物理零位。";
  if (!calibration.zeroTicks) return "① 把所有关节移到物理零位，点击【采集零点】。";
  if (calibration.sweeping) return "② 正在录制中……自由扭动全臂覆盖完整行程，完成后点击【停止录制】。";
  if (!calibration.sweepResults) return "② 点击【开始录制】，自由摆动机械臂，系统自动记录各关节范围。";
  return allCalibrationComplete() ? "③ 检查下方结果，点击【保存标定并进入可视化】。" : "无数据，请重新开始录制。";
}

function updateCalibrationUI() {
  const gateLocked = !workflow.calibration_ready;
  const zeroCaptured = Boolean(calibration.zeroTicks);
  const started = calibration.started;
  const sweeping = calibration.sweeping;
  const hasSweepResults = allCalibrationComplete();

  ui.calibrationState.textContent = workflow.calibrated
    ? "已标定"
    : hasSweepResults
      ? "待保存"
      : sweeping
        ? "录制中"
        : zeroCaptured
          ? "零点已采"
          : started
            ? "零位"
            : "Idle";
  ui.calibrationInstruction.textContent = nextCalibrationInstruction();
  ui.startCalibration.disabled = busy || gateLocked;
  ui.resetCalibration.disabled = busy || (!started && !zeroCaptured && !hasSweepResults);
  ui.captureZero.disabled = busy || gateLocked || !started || zeroCaptured;
  ui.startSweep.disabled = busy || gateLocked || !started || !zeroCaptured || sweeping || hasSweepResults;
  ui.stopSweep.disabled = busy || gateLocked || !sweeping;
  ui.saveCalibration.disabled = busy || gateLocked || !hasSweepResults;
  renderCalibrationRows();
}

function resetCalibrationState(reportText = "开始标定后显示结果。") {
  calibration.started = false;
  calibration.zeroTicks = null;
  calibration.sweeping = false;
  calibration.sweepResults = null;
  ui.calibrationReport.textContent = reportText;
  updateCalibrationUI();
}

function renderCalibrationReport(lines) {
  ui.calibrationReport.textContent = lines.filter(Boolean).join("\n");
}

function renderSavedCalibration(summary) {
  const lines = [ui.mockToggle.checked ? "Mock calibration preview:" : "Saved calibration:"];
  for (const item of summary) {
    lines.push(`${item.name}: zero=${item.zero_tick}, dir=${item.direction > 0 ? "+1" : "-1"}, safe=[${item.min_rad.toFixed(4)}, ${item.max_rad.toFixed(4)}] rad`);
  }
  renderCalibrationReport(lines);
}

function setLine(line, x1, y1, x2, y2) {
  line.setAttribute("x1", x1.toFixed(1));
  line.setAttribute("y1", y1.toFixed(1));
  line.setAttribute("x2", x2.toFixed(1));
  line.setAttribute("y2", y2.toFixed(1));
}

function setCircle(circle, x, y) {
  circle.setAttribute("cx", x.toFixed(1));
  circle.setAttribute("cy", y.toFixed(1));
}

function renderRobot(positions = jointPositions, modelPositions = modelJointPositions) {
  const q = { ...initialPositions(), ...positions };
  if (urdfViewer) urdfViewer.setJoints(Object.keys(modelPositions || {}).length ? modelPositions : q);
  const base = { x: 450 + Math.sin(q.shoulder_pan || 0) * 34, y: 414 };
  const shoulder = { x: base.x, y: 292 };
  const lift = q.shoulder_lift || 0;
  const elbow = lift + (q.elbow_flex || 0);
  const wrist = elbow + (q.wrist_flex || 0);
  const upper = 150;
  const forearm = 135;
  const hand = 88;
  const elbowPoint = {
    x: shoulder.x + Math.cos(lift) * upper,
    y: shoulder.y - Math.sin(lift) * upper,
  };
  const wristPoint = {
    x: elbowPoint.x + Math.cos(elbow) * forearm,
    y: elbowPoint.y - Math.sin(elbow) * forearm,
  };
  const toolPoint = {
    x: wristPoint.x + Math.cos(wrist) * hand,
    y: wristPoint.y - Math.sin(wrist) * hand,
  };
  const grip = 18 + Math.abs(q.gripper || 0) * 20;
  const rollOffset = Math.sin(q.wrist_roll || 0) * 9;

  ui.robot.turntable.setAttribute("x", (base.x - 36).toFixed(1));
  setLine(ui.robot.linkUpper, base.x, base.y, shoulder.x, shoulder.y);
  setLine(ui.robot.linkForearm, shoulder.x, shoulder.y, elbowPoint.x, elbowPoint.y);
  setLine(ui.robot.linkWrist, elbowPoint.x, elbowPoint.y, wristPoint.x, wristPoint.y);
  setLine(ui.robot.toolPalm, wristPoint.x, wristPoint.y, toolPoint.x, toolPoint.y);
  setLine(ui.robot.fingerTop, toolPoint.x, toolPoint.y, toolPoint.x + 38, toolPoint.y - grip + rollOffset);
  setLine(ui.robot.fingerBottom, toolPoint.x, toolPoint.y, toolPoint.x + 38, toolPoint.y + grip + rollOffset);
  setCircle(ui.robot.jointBase, base.x, base.y);
  setCircle(ui.robot.jointShoulder, shoulder.x, shoulder.y);
  setCircle(ui.robot.jointElbow, elbowPoint.x, elbowPoint.y);
  setCircle(ui.robot.jointWrist, wristPoint.x, wristPoint.y);

  if (ui.modelPose) {
    ui.modelPose.textContent = `SDK 当前值：pan ${Number(q.shoulder_pan || 0).toFixed(2)} rad / wrist_flex ${Number(q.wrist_flex || 0).toFixed(2)} rad`;
  }
}

function hasPositions(positions) {
  return Object.keys(positions || {}).length > 0;
}

function displayJointPositions() {
  return hasPositions(hardwareJointPositions) ? hardwareJointPositions : jointPositions;
}

function displayModelJointPositions() {
  return hasPositions(hardwareModelJointPositions) ? hardwareModelJointPositions : modelJointPositions;
}

function renderCurrentRobot() {
  renderRobot(displayJointPositions(), displayModelJointPositions());
}

function applyTorqueState(payload) {
  if (typeof payload?.torque_enabled !== "boolean") return;
  jointTorqueEnabled = Boolean(payload.torque_enabled && workflow.control_ready);
  ui.torqueToggle.checked = jointTorqueEnabled;
  if (!jointTorqueEnabled) cancelPendingMotionCommand();
}

function positionsFromJointPayload(joints = {}) {
  const positions = {};
  for (const [name, state] of Object.entries(joints || {})) {
    if (state.position_rad !== null && state.position_rad !== undefined) {
      positions[name] = Number(state.position_rad);
    }
  }
  return positions;
}

function updateFkReport(fk) {
  if (!fk?.end_effector) return;
  const p = fk.end_effector.position;
  ui.fkReport.textContent = [
    "tool0:",
    `  x: ${p[0].toFixed(4)} m`,
    `  y: ${p[1].toFixed(4)} m`,
    `  z: ${p[2].toFixed(4)} m`,
  ].join("\n");
}

function renderJointSliders() {
  ui.jointSliders.innerHTML = "";
  for (const joint of config?.joints || []) {
    const targetValue = Number(jointPositions[joint.name] ?? hardwareJointPositions[joint.name] ?? 0);
    const actualValue = Number(hardwareJointPositions[joint.name] ?? targetValue);
    const row = document.createElement("label");
    row.className = "slider-row";
    row.innerHTML = `
      <span class="slider-name">${joint.name}<small>id ${joint.id}</small></span>
      <input class="joint-range" type="range" min="${joint.min_rad}" max="${joint.max_rad}" step="0.01" value="${targetValue}" data-joint="${joint.name}" />
      <output class="joint-number" data-actual-joint="${joint.name}">${actualValue.toFixed(2)}</output>
    `;
    ui.jointSliders.append(row);
  }
}

function setJointSliderValue(joint, value) {
  const input = ui.jointSliders.querySelector(`input[data-joint="${joint}"]`);
  if (input) input.value = String(value);
}

function setJointActualValue(joint, value) {
  const output = ui.jointSliders.querySelector(`[data-actual-joint="${joint}"]`);
  if (output) output.textContent = Number(value).toFixed(2);
}

function renderSliderValues(positions = jointPositions) {
  for (const [joint, value] of Object.entries(positions)) {
    setJointSliderValue(joint, value);
  }
}

function renderActualValues(positions = hardwareJointPositions) {
  for (const [joint, value] of Object.entries(positions)) {
    setJointActualValue(joint, value);
  }
}

function updateJointControls() {
  const locked = !workflow.control_ready || busy;
  ui.jointSliders.querySelectorAll(".joint-range").forEach((input) => {
    input.disabled = locked || !jointTorqueEnabled;
  });
  ui.resetPose.disabled = locked || !jointTorqueEnabled;
  ui.syncState.textContent = workflow.control_ready
    ? !jointTorqueEnabled
      ? "关节未使能：可手动摆动实机；可视化、滑块和实际值持续跟随反馈。"
      : motionCommandInFlight
        ? "关节已使能：目标正在下发；实际值和可视化持续跟随实机。"
        : "关节已使能：拖动滑块下发目标；实际值和可视化持续跟随实机。"
    : workflow.calibrated
      ? "已有标定；当前状态检查通过后可使用滑条。"
      : "标定完成后可使用滑条。";
}

function applyStatePayload(payload, { forceSliders = false } = {}) {
  applyTorqueState(payload);
  const actualPositions = positionsFromJointPayload(payload.joints);
  hardwareJointPositions = { ...hardwareJointPositions, ...actualPositions };
  hardwareModelJointPositions = payload.model_joints || hardwareModelJointPositions;
  renderActualValues(actualPositions);
  if (forceSliders || !jointTorqueEnabled) {
    jointPositions = { ...jointPositions, ...actualPositions };
    renderSliderValues(jointPositions);
  }
  renderCurrentRobot();
  updateFkReport(payload.fk);
  updateJointControls();
}

function applyMovePayload(payload) {
  applyTorqueState(payload);
  jointPositions = { ...jointPositions, ...(payload.targets || {}) };
  const actualPositions = positionsFromJointPayload(payload.joints);
  hardwareJointPositions = { ...hardwareJointPositions, ...actualPositions };
  hardwareModelJointPositions = payload.model_joints || hardwareModelJointPositions;
  renderActualValues(actualPositions);
  renderSliderValues(jointPositions);
  renderCurrentRobot();
  updateFkReport(payload.fk);
  updateJointControls();
}

async function pollLiveState(options = {}) {
  const force = Boolean(options.force);
  if (!workflow.control_ready || !isVisualizerVisible()) return;
  if (liveStateInFlight) return;
  if (!force && busy) return;
  liveStateInFlight = true;
  try {
    const payload = await api(`/api/state?mock=${ui.mockToggle.checked ? "true" : "false"}`);
    applyStatePayload(payload, { forceSliders: force });
    liveStateErrorShown = false;
  } catch (error) {
    if (!liveStateErrorShown) {
      setMessage(`实机状态同步失败：${error.message}`, true);
      liveStateErrorShown = true;
    }
  } finally {
    liveStateInFlight = false;
  }
}

function cancelPendingMotionCommand() {
  if (motionCommandTimer) clearTimeout(motionCommandTimer);
  motionCommandTimer = null;
  pendingMotionCommand = null;
}

function queueMotionCommand(joint, value, { immediate = false } = {}) {
  pendingMotionCommand = { joint, value: Number(value) };
  if (motionCommandTimer) clearTimeout(motionCommandTimer);
  motionCommandTimer = setTimeout(sendPendingMotionCommand, immediate ? 0 : motionCommandIntervalMs);
}

async function sendPendingMotionCommand() {
  motionCommandTimer = null;
  if (motionCommandInFlight) {
    motionCommandTimer = setTimeout(sendPendingMotionCommand, motionCommandIntervalMs);
    return;
  }
  if (!pendingMotionCommand || !jointTorqueEnabled || !workflow.control_ready) {
    pendingMotionCommand = null;
    return;
  }
  const command = pendingMotionCommand;
  pendingMotionCommand = null;
  motionCommandInFlight = true;
  updateJointControls();
  try {
    const payload = await api("/api/move", {
      method: "POST",
      body: JSON.stringify({
        mock: ui.mockToggle.checked,
        sync: true,
        wait: false,
        targets: { [command.joint]: command.value },
      }),
    });
    applyMovePayload(payload);
    const durationText = payload.duration ? `（${payload.duration.toFixed(2)}s）` : "";
    setMessage(`目标已下发到实机${durationText}。`);
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    motionCommandInFlight = false;
    updateJointControls();
    if (pendingMotionCommand) {
      motionCommandTimer = setTimeout(sendPendingMotionCommand, motionCommandIntervalMs);
    }
  }
}

function updateTargetSlider(joint, value, options = {}) {
  if (!jointTorqueEnabled || !workflow.control_ready) {
    renderSliderValues(hardwareJointPositions);
    return;
  }
  const numericValue = Number(value);
  jointPositions[joint] = numericValue;
  setJointSliderValue(joint, numericValue);
  renderCurrentRobot();
  queueMotionCommand(joint, numericValue, { immediate: Boolean(options.immediate) });
}

async function loadSession() {
  try {
    const payload = await api("/api/session");
    config = payload.config;
    workflow = payload.workflow;
    jointTorqueEnabled = Boolean(payload.torque_enabled && workflow.control_ready);
    ui.torqueToggle.checked = jointTorqueEnabled;
    applyFrequencySettings(config.frequencies);
    ui.mockToggle.checked = Boolean(payload.mock);
    updateHeader();
    renderPortOptions(config.arm.port ? [config.arm.port] : []);
    renderJoints(config.joints.map(emptyJointState));
    jointPositions = initialPositions();
    modelJointPositions = {};
    hardwareJointPositions = {};
    hardwareModelJointPositions = {};
    renderJointSliders();
    renderRobot(jointPositions);
    resetCalibrationState();
    navigateTo(viewFromWorkflow());
    if (workflow.control_ready) {
      await loadRobotModel();
      navigateTo("visualizer");
      setMessage("Config loaded.");
    } else if (workflow.calibrated) {
      setMessage("已有标定，正在检查当前状态和软限位...");
      await checkStatus({ auto: true });
    } else {
      setMessage("Config loaded.");
    }
  } catch (error) {
    setMessage(error.message, true);
    ui.reportText.textContent = error.message;
  }
}

async function loadRobotModel() {
  const payload = await api("/api/robot/model");
  robotModel = payload;
  if (payload.config) {
    config = payload.config;
    applyFrequencySettings(config.frequencies);
  }
  workflow = payload.workflow;
  jointTorqueEnabled = Boolean(payload.torque_enabled && workflow.control_ready);
  ui.torqueToggle.checked = jointTorqueEnabled;
  jointPositions = { ...initialPositions(), ...payload.fk.positions };
  modelJointPositions = payload.model_joints || payload.fk.model_joints || {};
  renderJointSliders();
  updateFkReport(payload.fk);
  await loadUrdfViewer(payload, modelJointPositions);
  renderRobot(jointPositions, modelJointPositions);
  if (ui.modelMode) ui.modelMode.textContent = `模型已加载：${payload.name}.urdf`;
  updateWorkflowUI();
  updateJointControls();
  pollLiveState({ force: true });
}

async function loadUrdfViewer(model, positions) {
  if (urdfViewer) {
    urdfViewer.dispose();
    urdfViewer = null;
  }
  ui.robotStage.classList.remove("urdf-ready");
  if (!model?.urdf_url || !ui.urdfViewport) return;

  try {
    urdfViewerModulePromise = urdfViewerModulePromise || import("/urdf-viewer.js?v=soarm101-shadcn-ui");
    const { createUrdfViewer } = await urdfViewerModulePromise;
    urdfViewer = await createUrdfViewer(ui.urdfViewport, {
      urdfUrl: model.urdf_url,
      joints: positions,
      renderHz: runtimeFrequencies.web_render_hz,
    });
    ui.robotStage.classList.add("urdf-ready");
  } catch (error) {
    console.error(error);
    if (ui.urdfViewport) ui.urdfViewport.replaceChildren();
    setMessage(`URDF 模型加载失败，已使用简化预览：${error.message}`, true);
  }
}

async function scanPorts() {
  setBusy(true);
  setMessage("Scanning candidate servo ports...");
  try {
    const payload = await api("/api/ports");
    renderPortOptions(payload.ports);
    setMessage(`Found ${payload.ports.length} candidate port(s).`);
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

async function saveIdentity(event) {
  event.preventDefault();
  setBusy(true);
  setMessage("Saving arm name and port...");
  try {
    config = await api("/api/config/identity", {
      method: "POST",
      body: JSON.stringify({ name: ui.armName.value, port: portFromInputs() }),
    });
    updateHeader();
    applyFrequencySettings(config.frequencies);
    workflow = { ...workflow, status_passed: false, calibration_ready: false, control_ready: false };
    jointTorqueEnabled = false;
    ui.torqueToggle.checked = false;
    cancelPendingMotionCommand();
    updateWorkflowUI();
    setMessage("Saved arm.name and arm.port.");
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

async function saveFrequencySettings(event) {
  event.preventDefault();
  setBusy(true);
  setMessage("Saving frequency settings...");
  try {
    config = await api("/api/config/frequencies", {
      method: "POST",
      body: JSON.stringify(frequencyPayloadFromInputs()),
    });
    updateHeader();
    applyFrequencySettings(config.frequencies);
    workflow = { ...workflow, status_passed: false, calibration_ready: false, control_ready: false };
    jointTorqueEnabled = false;
    ui.torqueToggle.checked = false;
    cancelPendingMotionCommand();
    updateWorkflowUI();
    setMessage("频率设置已保存。");
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

async function checkStatus(_options = {}) {
  setBusy(true);
  ui.reportMode.textContent = ui.mockToggle.checked ? "Mock bus" : "Hardware";
  ui.resultLabel.textContent = "Checking...";
  setMessage("Running read-only status checks...");
  try {
    const payload = await api("/api/status", {
      method: "POST",
      body: JSON.stringify({ mock: ui.mockToggle.checked }),
    });
    ui.reportText.textContent = payload.lines.join("\n");
    renderJoints(parseStatus(payload.lines));
    workflow = payload.workflow || {
      ...workflow,
      status_passed: payload.passed,
      calibration_ready: Boolean(payload.calibration_ready || payload.passed),
      control_ready: Boolean(workflow.calibrated && payload.passed),
    };
    if (!workflow.control_ready) {
      jointTorqueEnabled = false;
      ui.torqueToggle.checked = false;
      cancelPendingMotionCommand();
    }
    ui.resultLabel.textContent = workflow.control_ready
      ? "Ready"
      : payload.passed
        ? "Passed"
        : workflow.calibration_ready
          ? "Calibration Ready"
          : "Failed";
    ui.resultLabel.className = workflow.calibration_ready || workflow.control_ready ? "pass-text" : "fail-text";
    setMessage(
      workflow.control_ready
        ? "当前状态检查通过，可以进入可视化与控制。"
        : payload.passed
        ? "状态检查通过，可以开始标定。"
        : workflow.calibration_ready
          ? "软限位超出当前配置；可手动回到限位内后重检，或重新标定。"
          : "状态检查未通过。",
      !workflow.calibration_ready,
    );
    if (workflow.control_ready) {
      await loadRobotModel();
      navigateTo("visualizer");
    } else if (workflow.calibration_ready) {
      navigateTo("calibration");
    } else {
      navigateTo("status");
    }
  } catch (error) {
    ui.resultLabel.textContent = "Failed";
    ui.resultLabel.className = "fail-text";
    ui.reportText.textContent = error.message;
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

async function startCalibration() {
  setBusy(true);
  setMessage("Disabling servo torque...");
  try {
    resetCalibrationState("Disabling torque...");
    await api("/api/calibration/disable", {
      method: "POST",
      body: JSON.stringify({ mock: ui.mockToggle.checked }),
    });
    calibration.started = true;
    renderCalibrationReport([
      ui.mockToggle.checked ? "Mock calibration session started." : "Hardware calibration session started.",
      "Torque is disabled. Move the arm to the physical zero pose.",
    ]);
    setMessage("Torque disabled. Capture zero when the arm is aligned.");
  } catch (error) {
    setMessage(error.message, true);
    renderCalibrationReport([error.message]);
  } finally {
    setBusy(false);
  }
}

async function captureCalibrationZero() {
  setBusy(true);
  setMessage("Capturing zero ticks...");
  try {
    const payload = await api("/api/calibration/capture", {
      method: "POST",
      body: JSON.stringify({ mock: ui.mockToggle.checked, sample: "zero" }),
    });
    calibration.zeroTicks = payload.ticks;
    calibration.sweepResults = null;
    const lines = ["Zero ticks captured:"];
    for (const joint of config.joints) lines.push(`${joint.name}: ${payload.ticks[joint.name]}`);
    renderCalibrationReport(lines);
    setMessage("Zero ticks captured. Click ② to start sweep recording.");
  } catch (error) {
    setMessage(error.message, true);
    renderCalibrationReport([error.message]);
  } finally {
    setBusy(false);
  }
}

async function startSweep() {
  if (!calibration.zeroTicks) return;
  setBusy(true);
  setMessage("Starting sweep recording...");
  try {
    const payload = await api("/api/calibration/sweep/start", {
      method: "POST",
      body: JSON.stringify({
        mock: ui.mockToggle.checked,
        zero_ticks: calibration.zeroTicks,
      }),
    });
    calibration.sweeping = true;
    calibration.sweepResults = null;
    const settings = payload.settings || {};
    renderCalibrationReport([
      "录制已开始（50 Hz 采样中）。",
      `▶ Limit margin: ${settings.margin_ticks ?? "?"} ticks。`,
      "▶ 自由扭动全臂，让每个关节覆盖完整行程。",
      "▶ 完成后点击【停止录制】。",
    ]);
    setMessage("录制中……摆动机械臂覆盖全行程，完成后点击停止录制。");
  } catch (error) {
    setMessage(error.message, true);
    renderCalibrationReport([error.message]);
  } finally {
    setBusy(false);
  }
}

async function stopSweep() {
  setBusy(true);
  setMessage("Stopping sweep and computing limits...");
  try {
    const payload = await api("/api/calibration/sweep/stop", {
      method: "POST",
      body: JSON.stringify({ mock: ui.mockToggle.checked }),
    });
    calibration.sweeping = false;
    calibration.sweepResults = payload.results;
    const lines = ["录制完成，自动计算结果："];
    for (const r of payload.results) {
      const warn = r.well_excited ? "" : " ⚠ 欠激励";
      lines.push(
        `${r.name}: dir=${r.inferred_direction > 0 ? "+1" : "-1"} | ` +
        `raw=[${r.raw_min_tick}..${r.raw_max_tick}] (Δ${r.range_ticks}) | ` +
        `limit_ticks=[${r.safe_min_tick}..${r.safe_max_tick}] | ` +
        `limit_rad=[${r.safe_min_rad.toFixed(4)}, ${r.safe_max_rad.toFixed(4)}]${warn}`,
      );
    }
    renderCalibrationReport(lines);
    setMessage("录制完成，检查结果后保存标定。");
  } catch (error) {
    calibration.sweeping = false;
    setMessage(error.message, true);
    renderCalibrationReport([error.message]);
  } finally {
    setBusy(false);
  }
}

async function saveCalibration() {
  setBusy(true);
  setMessage("Saving calibration...");
  try {
    const payload = await api("/api/calibration/save", {
      method: "POST",
      body: JSON.stringify({
        mock: ui.mockToggle.checked,
        sweep_results: calibration.sweepResults,
      }),
    });
    config = payload.config;
    workflow = payload.workflow || {
      phase: "status",
      status_passed: false,
      calibration_ready: true,
      calibrated: true,
      control_ready: false,
    };
    renderSavedCalibration(payload.summary);
    if (payload.post_status?.lines?.length) {
      ui.reportText.textContent = payload.post_status.lines.join("\n");
      renderJoints(parseStatus(payload.post_status.lines));
      const reportLines = ui.calibrationReport.textContent.split("\n");
      reportLines.push("", "Post-calibration status:", ...payload.post_status.lines);
      if (payload.post_status_error) reportLines.push(`Status error: ${payload.post_status_error}`);
      renderCalibrationReport(reportLines);
    }
    ui.resultLabel.textContent = workflow.control_ready
      ? "Ready"
      : payload.saved
        ? "Saved"
        : "Previewed";
    ui.resultLabel.className = workflow.control_ready || workflow.calibration_ready ? "pass-text" : "fail-text";
    setMessage(
      workflow.control_ready
        ? "标定已保存，当前状态也在限位内。"
        : payload.post_status_error
          ? `标定已保存，但状态复检失败：${payload.post_status_error}`
          : "标定已保存，但当前状态/限位未通过；请查看状态报告。",
      !workflow.control_ready,
    );
    if (workflow.control_ready) {
      await loadRobotModel();
      navigateTo("visualizer");
    } else {
      navigateTo(workflow.calibration_ready ? "calibration" : "status");
    }
  } catch (error) {
    setMessage(error.message, true);
    renderCalibrationReport([error.message]);
  } finally {
    setBusy(false);
  }
}

async function setJointTorqueEnabled(enabled) {
  const previous = jointTorqueEnabled;
  setBusy(true);
  setMessage(enabled ? "正在使能关节..." : "正在关闭关节使能...");
  try {
    const payload = await api("/api/torque", {
      method: "POST",
      body: JSON.stringify({
        mock: ui.mockToggle.checked,
        enabled: Boolean(enabled),
      }),
    });
    jointTorqueEnabled = Boolean(payload.torque_enabled);
    ui.torqueToggle.checked = jointTorqueEnabled;
    if (!jointTorqueEnabled) cancelPendingMotionCommand();
    applyStatePayload(payload, { forceSliders: true });
    setMessage(
      jointTorqueEnabled
        ? "关节已使能，目标滑块已对齐当前实机位置。"
        : "关节使能已关闭，滑块和可视化将跟随实机反馈。",
    );
  } catch (error) {
    jointTorqueEnabled = previous;
    ui.torqueToggle.checked = previous;
    setMessage(error.message, true);
  } finally {
    setBusy(false);
    updateJointControls();
  }
}

async function readState() {
  setBusy(true);
  try {
    const payload = await api(`/api/state?mock=${ui.mockToggle.checked ? "true" : "false"}`);
    applyStatePayload(payload);
    setMessage("已读取实机关节状态。");
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

async function resetPose() {
  cancelPendingMotionCommand();
  const actual = hasPositions(hardwareJointPositions) ? hardwareJointPositions : jointPositions;
  jointPositions = { ...jointPositions, ...actual };
  renderSliderValues(jointPositions);
  renderCurrentRobot();
  updateJointControls();
  setMessage("目标滑块已重置为当前实机位置。");
}

async function solveIk() {
  setBusy(true);
  try {
    const payload = await api("/api/ik", {
      method: "POST",
      body: JSON.stringify({
        target: {
          x: Number(ui.ikX.value),
          y: Number(ui.ikY.value),
          z: Number(ui.ikZ.value),
        },
      }),
    });
    if (jointTorqueEnabled) {
      jointPositions = { ...jointPositions, ...payload.positions };
      renderSliderValues(jointPositions);
    }
    updateFkReport(payload.fk);
    ui.fkReport.textContent += `\nIK reachable: ${payload.reachable ? "yes" : "no"}`;
    if (payload.violations?.length) ui.fkReport.textContent += `\n${payload.violations.join("\n")}`;
    setMessage(
      jointTorqueEnabled
        ? "IK 解算结果已写入目标滑块；不会自动下发。"
        : "IK 解算完成；关节未使能时滑块保持跟随实机。",
      !payload.reachable,
    );
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

ui.portSelect.addEventListener("change", () => {
  if (ui.portSelect.value) ui.manualPort.value = ui.portSelect.value;
});
for (const step of [ui.stepStatus, ui.stepCalibration, ui.stepControl]) {
  step.addEventListener("click", () => goToStep(step.dataset.stepTarget));
  step.addEventListener("keydown", (event) => {
    if (event.key !== "Enter" && event.key !== " ") return;
    event.preventDefault();
    goToStep(step.dataset.stepTarget);
  });
}
ui.scanPorts.addEventListener("click", scanPorts);
ui.identityForm.addEventListener("submit", saveIdentity);
ui.frequencyForm.addEventListener("submit", saveFrequencySettings);
ui.controlHz.addEventListener("input", () => {
  ui.serialWriteHz.value = String(Math.max(1, Number(ui.controlHz.value) || DEFAULT_FREQUENCIES.control_hz));
});
ui.checkStatus.addEventListener("click", checkStatus);
ui.startCalibration.addEventListener("click", startCalibration);
ui.resetCalibration.addEventListener("click", () => {
  resetCalibrationState();
  setMessage("Calibration reset.");
});
ui.captureZero.addEventListener("click", captureCalibrationZero);
ui.startSweep.addEventListener("click", startSweep);
ui.stopSweep.addEventListener("click", stopSweep);
ui.saveCalibration.addEventListener("click", saveCalibration);
ui.readState.addEventListener("click", readState);
ui.resetPose.addEventListener("click", resetPose);
ui.solveIk.addEventListener("click", solveIk);
ui.torqueToggle.addEventListener("change", () => {
  setJointTorqueEnabled(ui.torqueToggle.checked);
});
ui.mockToggle.addEventListener("change", () => {
  jointTorqueEnabled = false;
  ui.torqueToggle.checked = false;
  cancelPendingMotionCommand();
  if (calibration.started || calibration.zeroTicks) {
    resetCalibrationState();
    setMessage("Calibration reset after bus mode changed.");
  }
});
ui.jointSliders.addEventListener("input", (event) => {
  const input = event.target.closest("input[data-joint]");
  if (!input) return;
  updateTargetSlider(input.dataset.joint, input.value);
});
ui.jointSliders.addEventListener("change", (event) => {
  const input = event.target.closest("input[data-joint]");
  if (!input) return;
  updateTargetSlider(input.dataset.joint, input.value, { immediate: true });
});

loadSession();
