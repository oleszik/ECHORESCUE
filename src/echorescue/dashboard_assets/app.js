"use strict";

const COLORS = {
  unknown: "#172033",
  free: "#344760",
  occupied: "#94a3b8",
  grid: "rgba(203, 213, 225, 0.20)",
  drone1: "#38bdf8",
  drone2: "#f59e0b",
  survivor: "#fde047",
  base: "#f1f5f9",
  radioDirect: "#34d399",
  radioRelay: "#c084fc",
  radioPeer: "#94a3b8",
  knowledgeGap: "#fb7185",
  textDark: "#071116",
};

const state = {
  replay: null,
  benchmark: null,
  frameIndex: 0,
  playing: false,
  speed: 1,
  timer: null,
  mapView: "operator",
};

const elements = {};

function cacheElements() {
  [
    "seedValue", "knowledgeMode", "missionStatus", "restartButton", "previousButton", "playButton",
    "playIcon", "playLabel", "nextButton", "timeline", "currentStep", "maxStep",
    "speedSelect", "mapViewSelect", "mapViewTitle", "mapViewPurpose", "missionCanvas", "coverageValue", "eventFeed", "eventCount",
    "resultBadge", "metricRecall", "metricReturned", "metricWalls", "metricDrones",
    "metricSteps", "metricDuplicate", "schemaVersion", "singleSteps", "multiSteps",
    "improvementValue", "benchmarkNote", "benchmarkPanel", "fatalError", "droneCard1", "droneCard2",
    "relaySummary", "relaySummaryState", "metricRelayDeployments", "metricRelayCells",
    "metricRelaySurvivors", "metricRelayDelay", "metricRelayEnergy", "metricBaseCoverage",
    "benchmarkTitle", "benchmarkBadge", "baselineLabel", "candidateLabel", "improvementLabel",
  ].forEach((id) => { elements[id] = document.getElementById(id); });
  [1, 2].forEach((number) => {
    ["State", "Position", "Energy", "Battery", "Target", "Communication", "Coverage", "Survivors", "DataAge"].forEach((field) => {
      elements[`drone${field}${number}`] = document.getElementById(`drone${field}${number}`);
    });
  });
}

async function loadJson(url, required = true) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    if (!required) return null;
    throw new Error(`Unable to load ${url} (${response.status})`);
  }
  try {
    return await response.json();
  } catch (error) {
    throw new Error(`Unable to parse ${url} as JSON: ${error.message}`);
  }
}

async function loadOptionalBenchmark(url) {
  try {
    return await loadJson(url, false);
  } catch (error) {
    return { __benchmark_load_error: error.message };
  }
}

function validateReplay(replay) {
  if (!replay || !replay.schema_version || !Array.isArray(replay.frames) || !replay.frames.length) {
    throw new Error("Replay is missing its schema version or mission frames.");
  }
  for (const frame of replay.frames) {
    if (!frame.drones?.["drone-1"] || !frame.drones?.["drone-2"]) {
      throw new Error(`Replay frame ${frame.step} does not contain both drones.`);
    }
  }
}

function initializeReplay(replay, benchmark) {
  validateReplay(replay);
  state.replay = replay;
  state.benchmark = benchmark;
  const knowledgeMode = replay.mission.knowledge_mode || replay.mission.configuration?.knowledge_mode || "shared";
  state.mapView = knowledgeMode === "local" ? "base" : "operator";
  elements.mapViewSelect.value = state.mapView;
  const relayStrategy = replay.mission.relay_strategy || replay.mission.configuration?.relay_strategy || "off";
  elements.knowledgeMode.textContent = relayStrategy === "adaptive"
    ? `${knowledgeMode.toUpperCase()} · ADAPTIVE RELAY`
    : knowledgeMode.toUpperCase();
  elements.seedValue.textContent = replay.mission.seed;
  elements.schemaVersion.textContent = replay.schema_version;
  elements.timeline.max = replay.frames.length - 1;
  elements.maxStep.textContent = replay.frames.at(-1).step;
  elements.missionCanvas.parentElement.style.aspectRatio = `${replay.map.width} / ${replay.map.height}`;
  populateMetrics();
  populateBenchmark();
  setFrame(0);
}

function setFrame(index) {
  if (!state.replay) return;
  state.frameIndex = Math.max(0, Math.min(index, state.replay.frames.length - 1));
  const frame = state.replay.frames[state.frameIndex];
  elements.timeline.value = state.frameIndex;
  elements.currentStep.textContent = frame.step;
  updateMapReadout(frame);
  updateDroneCard(1, frame.drones["drone-1"]);
  updateDroneCard(2, frame.drones["drone-2"]);
  updateStatus(frame);
  updateEventFeed();
  drawMission();
  if (state.frameIndex === state.replay.frames.length - 1 && state.playing) pause();
}

function updateDroneCard(number, drone) {
  const [x, y] = drone.position;
  const relayHold = drone.relay?.holding_for_relay ? "RELAY HOLD · " : "";
  elements[`droneState${number}`].textContent = `${drone.yielding ? "YIELDING · " : relayHold}${drone.state.replaceAll("_", " ")}`;
  elements[`droneCard${number}`].classList.toggle("yielding", Boolean(drone.yielding));
  elements[`droneCard${number}`].classList.toggle("relay-active", Boolean(drone.relay?.active));
  elements[`dronePosition${number}`].textContent = `${x}, ${y}`;
  elements[`droneEnergy${number}`].textContent = `${drone.energy_remaining.toFixed(1)} units`;
  elements[`droneBattery${number}`].style.width = `${Math.max(0, Math.min(100, drone.energy_remaining_percent))}%`;
  elements[`droneTarget${number}`].textContent = drone.target ? `${drone.target[0]}, ${drone.target[1]}` : "—";
  elements[`droneCoverage${number}`].textContent = `${drone.knowledge.known_coverage.toFixed(1)}%`;
  elements[`droneSurvivors${number}`].textContent = `${drone.knowledge.confirmed_survivors || 0} confirmed`;
  elements[`droneDataAge${number}`].textContent = `${drone.knowledge.average_data_age.toFixed(1)} avg · ${drone.knowledge.oldest_data_age} max`;
  const communication = drone.communication;
  const status = elements[`droneCommunication${number}`];
  status.className = "communication-status";
  if (communication.direct_to_base) {
    status.textContent = "Direct to base";
    status.classList.add("direct");
  } else if (communication.via_relay) {
    const relay = communication.relay_path.at(-2);
    status.textContent = `Relay via ${relay}`;
    status.classList.add("relay");
  } else {
    status.textContent = "Disconnected";
    status.classList.add("offline");
  }
}

function selectedKnowledgeMap(frame) {
  return frame.knowledge_maps[state.mapView] || frame.knowledge_maps.operator;
}

function updateMapReadout(frame) {
  const knowledgeMap = selectedKnowledgeMap(frame);
  elements.coverageValue.textContent = `${knowledgeMap.known_coverage.toFixed(1)}%`;
  const labels = {
    operator: ["Global operator map", "Evaluation aggregate · never used for local decisions"],
    "drone-1": ["Local map · drone-1", "Decision knowledge held by drone-1"],
    "drone-2": ["Local map · drone-2", "Decision knowledge held by drone-2"],
    base: ["Base knowledge", "Operational knowledge received over radio"],
  };
  const [title, purpose] = labels[state.mapView] || labels.operator;
  elements.mapViewTitle.textContent = title;
  elements.mapViewPurpose.textContent = purpose;
}

function updateStatus(frame) {
  const last = state.frameIndex === state.replay.frames.length - 1;
  const success = state.replay.metrics.mission_success;
  elements.missionStatus.className = `status-pill ${last ? (success ? "success" : "failure") : "running"}`;
  elements.missionStatus.lastElementChild.textContent = last ? (success ? "Mission complete" : "Mission failed") : "Replay in progress";
  elements.resultBadge.className = `result-badge ${last ? (success ? "success" : "failure") : "pending"}`;
  elements.resultBadge.textContent = last ? (success ? "Success" : "Failure") : "In progress";
}

function eventColor(event) {
  if (event.event_type === "safety_shield_intervention") return "#fb7185";
  if (["relay_link_achieved", "relay_payload_forwarded"].includes(event.event_type)) return "#34d399";
  if (event.event_type.startsWith("relay_role_") || event.event_type === "relay_position_selected") return COLORS.radioRelay;
  if (["local_collision_avoided", "yield_started", "yield_ended", "deadlock_replanned"].includes(event.event_type)) return "#34d399";
  if (event.event_type === "corridor_deadlock_detected") return "#f59e0b";
  if (event.drone_id === "drone-1") return COLORS.drone1;
  if (event.drone_id === "drone-2") return COLORS.drone2;
  return "#8d9cb0";
}

function eventLabel(type) {
  return type.replaceAll("_", " ").replace(/\b\w/g, (letter) => letter.toUpperCase());
}

function updateEventFeed() {
  const knowledgeMode = state.replay.mission.knowledge_mode || "shared";
  const eventVisible = (event) => {
    if (knowledgeMode !== "local") return true;
    if (["survivor_detected", "survivor_confirmed"].includes(event.event_type)) {
      return state.mapView === event.drone_id;
    }
    if (event.event_type === "survivor_knowledge_synchronized") {
      if (["operator", "base"].includes(state.mapView)) return event.drone_id === "base";
      return event.drone_id === state.mapView;
    }
    return true;
  };
  const events = state.replay.frames
    .slice(0, state.frameIndex + 1)
    .flatMap((frame) => frame.events.map((event) => ({ ...event, frameStep: frame.step })))
    .filter(eventVisible)
    .slice(-12)
    .reverse();
  elements.eventFeed.replaceChildren();
  elements.eventCount.textContent = events.length;
  if (!events.length) {
    const empty = document.createElement("li");
    empty.className = "event-item";
    empty.textContent = "No mission events yet";
    elements.eventFeed.append(empty);
    return;
  }
  for (const event of events) {
    const item = document.createElement("li");
    item.className = "event-item";
    const step = document.createElement("span");
    step.className = "event-step";
    step.textContent = String(event.step).padStart(3, "0");
    const node = document.createElement("span");
    node.className = "event-node";
    node.style.setProperty("--event-color", eventColor(event));
    const copy = document.createElement("div");
    copy.className = "event-copy";
    const title = document.createElement("strong");
    title.textContent = eventLabel(event.event_type);
    const detail = document.createElement("span");
    const cells = event.cell_count == null ? "" : ` · ${event.cell_count} cells`;
    const survivors = event.survivor_count == null ? "" : ` · ${event.survivor_count} survivors`;
    detail.textContent = `${event.drone_id} · [${event.position.join(", ")}]${cells}${survivors}`;
    copy.append(title, detail);
    item.append(step, node, copy);
    elements.eventFeed.append(item);
  }
}

function populateMetrics() {
  const metrics = state.replay.metrics;
  elements.metricRecall.textContent = `${(metrics.survivor_recall * 100).toFixed(0)}%`;
  elements.metricReturned.textContent = `${metrics.drones_returned}/${metrics.drones_total}`;
  elements.metricWalls.textContent = metrics.collisions;
  elements.metricDrones.textContent = metrics.drone_drone_collisions;
  elements.metricSteps.textContent = metrics.steps;
  elements.metricDuplicate.textContent = `${(metrics.duplicate_exploration_ratio * 100).toFixed(2)}%`;
  const adaptive = metrics.relay_strategy === "adaptive";
  elements.relaySummary.classList.toggle("inactive", !adaptive);
  elements.relaySummaryState.textContent = adaptive ? "ENABLED" : "OFF";
  const successfulDeployments = metrics.successful_relay_deployments;
  const deployments = metrics.relay_deployments;
  elements.metricRelayDeployments.textContent = Number.isFinite(successfulDeployments) && Number.isFinite(deployments)
    ? `${successfulDeployments}/${deployments}` : "—";
  elements.metricRelayCells.textContent = formatOptionalNumber(metrics.relay_unique_cells_forwarded, 0);
  elements.metricRelaySurvivors.textContent = formatOptionalNumber(metrics.relay_survivor_confirmations_forwarded, 0);
  elements.metricRelayDelay.textContent = formatOptionalNumber(metrics.relay_mission_delay_steps, 0, " steps");
  elements.metricRelayEnergy.textContent = formatOptionalNumber(metrics.relay_energy_consumed, 1, " units");
  elements.metricBaseCoverage.textContent = formatOptionalNumber(metrics.base_known_coverage, 1, "%");
}

function isRecord(value) {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function hasOwn(object, key) {
  return Object.prototype.hasOwnProperty.call(object, key);
}

function requiredObject(object, key, formatName) {
  if (!isRecord(object[key])) {
    throw new Error(`${formatName} benchmark is missing required object "${key}".`);
  }
  return object[key];
}

function optionalNumber(object, key, path) {
  if (!hasOwn(object, key) || object[key] === null) return null;
  if (typeof object[key] !== "number" || !Number.isFinite(object[key])) {
    throw new Error(`Benchmark field "${path}.${key}" must be a finite number.`);
  }
  return object[key];
}

function formatOptionalNumber(value, digits = 2, suffix = "") {
  return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "—";
}

function signedMetric(value, suffix) {
  if (!Number.isFinite(value)) return "—";
  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(2)}${suffix}`;
}

function joinBenchmarkNote(parts) {
  return parts.length ? parts.join(" ") : "Optional benchmark values are not available.";
}

function normalizeAdaptiveRelayBenchmark(benchmark) {
  const off = requiredObject(benchmark, "active_local_relay_off", "Adaptive Relay");
  const adaptive = requiredObject(benchmark, "active_local_adaptive_relay", "Adaptive Relay");
  const tradeOff = hasOwn(benchmark, "trade_off")
    ? requiredObject(benchmark, "trade_off", "Adaptive Relay") : {};
  const offSteps = optionalNumber(off, "average_mission_steps", "active_local_relay_off");
  const adaptiveSteps = optionalNumber(adaptive, "average_mission_steps", "active_local_adaptive_relay");
  const offUptime = optionalNumber(off, "average_communication_uptime", "active_local_relay_off");
  const adaptiveUptime = optionalNumber(adaptive, "average_communication_uptime", "active_local_adaptive_relay");
  const uptimeGain = optionalNumber(tradeOff, "communication_uptime_percentage_points", "trade_off");
  const cells = optionalNumber(adaptive, "relay_unique_cells_forwarded", "active_local_adaptive_relay");
  const deployments = optionalNumber(adaptive, "successful_relay_deployments", "active_local_adaptive_relay");
  const parts = [];
  if (Number.isFinite(offUptime) && Number.isFinite(adaptiveUptime)) {
    parts.push(`Communication uptime: ${(offUptime * 100).toFixed(2)}% off versus ${(adaptiveUptime * 100).toFixed(2)}% adaptive.`);
  }
  if (Number.isFinite(offSteps) && Number.isFinite(adaptiveSteps)) {
    parts.push(`Average duration: ${offSteps.toFixed(2)} versus ${adaptiveSteps.toFixed(2)} steps.`);
  }
  if (Number.isFinite(cells) && Number.isFinite(deployments)) {
    parts.push(`${deployments.toFixed(0)} successful deployments forwarded ${cells.toFixed(0)} cells.`);
  }
  return {
    status: "ready",
    format: "adaptive_relay",
    title: "Adaptive relay trade-off",
    baselineLabel: "Relay off",
    candidateLabel: "Adaptive relay",
    baselineSteps: offSteps,
    candidateSteps: adaptiveSteps,
    improvementValue: signedMetric(uptimeGain, " pp"),
    improvementLabel: "uptime gain",
    note: joinBenchmarkNote(parts),
  };
}

function normalizeParallelBenchmark(benchmark) {
  const single = requiredObject(benchmark, "single_drone", "Parallel exploration");
  const multi = requiredObject(benchmark, "two_drone", "Parallel exploration");
  const comparison = hasOwn(benchmark, "comparison")
    ? requiredObject(benchmark, "comparison", "Parallel exploration") : {};
  const singleSteps = optionalNumber(single, "average_mission_steps", "single_drone");
  const multiSteps = optionalNumber(multi, "average_mission_steps", "two_drone");
  const reduction = optionalNumber(comparison, "mission_duration_reduction_percent", "comparison");
  const duplicate = optionalNumber(multi, "average_duplicate_exploration_ratio", "two_drone");
  const pathIncrease = optionalNumber(comparison, "combined_path_length_increase_percent", "comparison");
  const parts = [];
  if (Number.isFinite(duplicate)) parts.push(`Duplicate exploration was ${(duplicate * 100).toFixed(2)}%.`);
  if (Number.isFinite(pathIncrease)) parts.push(`Combined fleet path length was ${pathIncrease.toFixed(1)}% higher.`);
  return {
    status: "ready",
    format: "parallel_exploration",
    title: "Parallel exploration impact",
    baselineLabel: "Single drone",
    candidateLabel: "Two drones",
    baselineSteps: singleSteps,
    candidateSteps: multiSteps,
    improvementValue: Number.isFinite(reduction) ? `${reduction.toFixed(2)}%` : "—",
    improvementLabel: "shorter duration",
    note: joinBenchmarkNote(parts),
  };
}

function normalizeKnowledgeModesBenchmark(benchmark) {
  const modes = requiredObject(benchmark, "modes", "Knowledge modes");
  const shared = requiredObject(modes, "shared", "Knowledge modes");
  const local = requiredObject(modes, "local", "Knowledge modes");
  return {
    status: "ready",
    format: "knowledge_modes",
    title: "Knowledge-mode comparison",
    baselineLabel: "Shared",
    candidateLabel: "Active local",
    baselineSteps: optionalNumber(shared, "average_mission_steps", "modes.shared"),
    candidateSteps: optionalNumber(local, "average_mission_steps", "modes.local"),
    improvementValue: "—",
    improvementLabel: "not available",
    note: "Shared and Active Local mission durations from the versioned mode benchmark.",
  };
}

function normalizeDeconflictionBenchmark(benchmark) {
  const legacy = requiredObject(benchmark, "legacy_safety_shield_baseline", "Deconfliction");
  const distributed = requiredObject(benchmark, "distributed_deconfliction", "Deconfliction");
  const delta = hasOwn(benchmark, "mission_duration_delta")
    ? requiredObject(benchmark, "mission_duration_delta", "Deconfliction") : {};
  const deltaPercent = optionalNumber(delta, "percent", "mission_duration_delta");
  return {
    status: "ready",
    format: "distributed_deconfliction",
    title: "Distributed deconfliction",
    baselineLabel: "Legacy shield",
    candidateLabel: "Distributed",
    baselineSteps: optionalNumber(legacy, "average_mission_steps", "legacy_safety_shield_baseline"),
    candidateSteps: optionalNumber(distributed, "average_mission_steps", "distributed_deconfliction"),
    improvementValue: signedMetric(deltaPercent, "%"),
    improvementLabel: "duration delta",
    note: "Legacy Safety-Shield and distributed-deconfliction mission durations.",
  };
}

function normalizeMissionTelemetryBenchmark(benchmark) {
  const behavior = requiredObject(benchmark, "mission_behavior", "Mission telemetry");
  return {
    status: "ready",
    format: hasOwn(benchmark, "communication") ? "communication" : "shadow_mode",
    title: hasOwn(benchmark, "communication") ? "Communication telemetry" : "Shadow-map telemetry",
    baselineLabel: "Mission behavior",
    candidateLabel: "Comparison",
    baselineSteps: optionalNumber(behavior, "average_mission_steps", "mission_behavior"),
    candidateSteps: null,
    improvementValue: "—",
    improvementLabel: "not available",
    note: "This benchmark contains one mission series; comparative values are not available.",
  };
}

function normalizeBenchmark(benchmark) {
  if (!isRecord(benchmark)) throw new Error("Benchmark root must be a JSON object.");
  if (hasOwn(benchmark, "schema_version") && typeof benchmark.schema_version !== "string") {
    throw new Error('Benchmark field "schema_version" must be a string.');
  }
  if (hasOwn(benchmark, "active_local_relay_off") || hasOwn(benchmark, "active_local_adaptive_relay")) {
    return normalizeAdaptiveRelayBenchmark(benchmark);
  }
  if (hasOwn(benchmark, "single_drone") || hasOwn(benchmark, "two_drone")) {
    return normalizeParallelBenchmark(benchmark);
  }
  if (hasOwn(benchmark, "modes")) return normalizeKnowledgeModesBenchmark(benchmark);
  if (hasOwn(benchmark, "distributed_deconfliction") || hasOwn(benchmark, "legacy_safety_shield_baseline")) {
    return normalizeDeconflictionBenchmark(benchmark);
  }
  if (hasOwn(benchmark, "mission_behavior")) return normalizeMissionTelemetryBenchmark(benchmark);
  const version = typeof benchmark.schema_version === "string" ? benchmark.schema_version : "missing";
  throw new Error(`Unrecognized benchmark format (schema_version: ${version}).`);
}

function safeBenchmarkView(benchmark) {
  if (benchmark === null || benchmark === undefined) {
    return { status: "unavailable", message: "Benchmark artifact unavailable." };
  }
  if (isRecord(benchmark) && typeof benchmark.__benchmark_load_error === "string") {
    return { status: "invalid", message: `Benchmark unavailable: ${benchmark.__benchmark_load_error}` };
  }
  try {
    return normalizeBenchmark(benchmark);
  } catch (error) {
    return { status: "invalid", message: `Benchmark unavailable: ${error.message}` };
  }
}

function populateBenchmark() {
  const view = safeBenchmarkView(state.benchmark);
  elements.benchmarkPanel.classList.remove("unavailable", "invalid");
  elements.singleSteps.textContent = "—";
  elements.multiSteps.textContent = "—";
  elements.improvementValue.textContent = "—";
  if (view.status !== "ready") {
    elements.benchmarkPanel.classList.add(view.status);
    elements.benchmarkBadge.textContent = view.status === "invalid" ? "Invalid artifact" : "Unavailable";
    elements.benchmarkNote.textContent = view.message;
    return;
  }
  elements.benchmarkTitle.textContent = view.title;
  elements.benchmarkBadge.textContent = "Reproducible";
  elements.baselineLabel.textContent = view.baselineLabel;
  elements.candidateLabel.textContent = view.candidateLabel;
  elements.singleSteps.textContent = formatOptionalNumber(view.baselineSteps, 2);
  elements.multiSteps.textContent = formatOptionalNumber(view.candidateSteps, 2);
  elements.improvementValue.textContent = view.improvementValue;
  elements.improvementLabel.textContent = view.improvementLabel;
  elements.benchmarkNote.textContent = view.note;
}

function canvasGeometry() {
  const canvas = elements.missionCanvas;
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width * ratio));
  const height = Math.max(1, Math.floor(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const mapWidth = state.replay.map.width;
  const mapHeight = state.replay.map.height;
  const padding = 20 * ratio;
  const cell = Math.min((width - padding * 2) / mapWidth, (height - padding * 2) / mapHeight);
  return {
    context: canvas.getContext("2d"), ratio, cell,
    offsetX: (width - cell * mapWidth) / 2,
    offsetY: (height - cell * mapHeight) / 2,
  };
}

function cellCenter(position, geometry) {
  return [
    geometry.offsetX + (position[0] + 0.5) * geometry.cell,
    geometry.offsetY + (position[1] + 0.5) * geometry.cell,
  ];
}

function dockingMarkerOffset(frame, droneId, geometry) {
  const dockedIds = Object.entries(frame.drones)
    .filter(([, drone]) => (
      drone.state === "LANDED"
      && drone.position[0] === state.replay.map.base[0]
      && drone.position[1] === state.replay.map.base[1]
    ))
    .map(([id]) => id)
    .sort();
  if (dockedIds.length < 2 || !dockedIds.includes(droneId)) return [0, 0];
  const slot = dockedIds.indexOf(droneId) - (dockedIds.length - 1) / 2;
  const spacing = Math.min(geometry.cell * 0.34, 20 * geometry.ratio);
  return [slot * spacing, 0];
}

function drawPolyline(context, points, geometry, color, width, dashed = false) {
  if (!points || points.length < 2) return;
  context.save();
  context.strokeStyle = color;
  context.lineWidth = width * geometry.ratio;
  context.lineJoin = "round";
  context.lineCap = "round";
  if (dashed) context.setLineDash([5 * geometry.ratio, 5 * geometry.ratio]);
  context.beginPath();
  points.forEach((position, index) => {
    const [x, y] = cellCenter(position, geometry);
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.stroke();
  context.restore();
}

function drawCommunicationLinks(context, frame, geometry) {
  const activeRelay = Object.entries(frame.drones).find(([, drone]) => drone.relay?.active);
  const relayId = activeRelay?.[0];
  const scoutId = activeRelay?.[1].relay?.scout_id;
  for (const link of frame.communication.links) {
    const first = frame.communication.nodes[link.from];
    const second = frame.communication.nodes[link.to];
    const [firstX, firstY] = cellCenter(first, geometry);
    const [secondX, secondY] = cellCenter(second, geometry);
    context.save();
    const adaptiveChain = relayId && scoutId && (
      new Set([link.from, link.to]).size === 2
      && (([link.from, link.to].includes("base") && [link.from, link.to].includes(relayId))
        || ([link.from, link.to].includes(relayId) && [link.from, link.to].includes(scoutId)))
    );
    context.strokeStyle = adaptiveChain ? COLORS.radioRelay : link.kind === "direct_base"
      ? COLORS.radioDirect
      : link.kind === "relay" ? COLORS.radioRelay : COLORS.radioPeer;
    context.globalAlpha = adaptiveChain ? 1 : link.kind === "peer" ? 0.52 : 0.82;
    context.lineWidth = (adaptiveChain ? 3.2 : link.kind === "relay" ? 2.2 : 1.7) * geometry.ratio;
    if (adaptiveChain || link.kind === "relay") {
      context.setLineDash([7 * geometry.ratio, 4 * geometry.ratio]);
    } else if (link.kind === "peer") {
      context.setLineDash([2 * geometry.ratio, 5 * geometry.ratio]);
    }
    context.beginPath();
    context.moveTo(firstX, firstY);
    context.lineTo(secondX, secondY);
    context.stroke();
    context.restore();
  }
}

function drawMotionReservation(context, drone, geometry, color) {
  const intent = drone.motion_intent;
  if (!intent?.reservation?.length) return;
  const points = [intent.current_position, ...intent.reservation];
  context.save();
  context.globalAlpha = drone.yielding ? 0.95 : 0.68;
  context.strokeStyle = color;
  context.lineWidth = 2.6 * geometry.ratio;
  context.lineCap = "round";
  context.setLineDash([1.5 * geometry.ratio, 4 * geometry.ratio]);
  context.beginPath();
  points.forEach((position, index) => {
    const [x, y] = cellCenter(position, geometry);
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.stroke();
  context.restore();
}

function drawMission() {
  if (!state.replay) return;
  const frame = state.replay.frames[state.frameIndex];
  const geometry = canvasGeometry();
  const { context, ratio, cell, offsetX, offsetY } = geometry;
  const knowledgeMap = selectedKnowledgeMap(frame);
  const differences = new Set(
    knowledgeMap.differences_from_shadow.map((position) => position.join(","))
  );
  context.clearRect(0, 0, context.canvas.width, context.canvas.height);

  knowledgeMap.occupancy.forEach((row, y) => {
    [...row].forEach((symbol, x) => {
      context.fillStyle = symbol === "?" ? COLORS.unknown : symbol === "." ? COLORS.free : COLORS.occupied;
      context.fillRect(offsetX + x * cell, offsetY + y * cell, cell, cell);
      context.strokeStyle = COLORS.grid;
      context.lineWidth = Math.max(0.6, ratio * 0.55);
      context.strokeRect(offsetX + x * cell, offsetY + y * cell, cell, cell);
      if (differences.has(`${x},${y}`)) {
        context.strokeStyle = COLORS.knowledgeGap;
        context.lineWidth = Math.max(1.2, ratio * 1.4);
        context.strokeRect(
          offsetX + x * cell + ratio,
          offsetY + y * cell + ratio,
          cell - ratio * 2,
          cell - ratio * 2,
        );
      }
    });
  });

  drawCommunicationLinks(context, frame, geometry);

  ["drone-1", "drone-2"].forEach((droneId, droneIndex) => {
    const color = droneIndex === 0 ? COLORS.drone1 : COLORS.drone2;
    const trail = state.replay.frames.slice(0, state.frameIndex + 1).map((item) => item.drones[droneId].position);
    drawPolyline(context, trail, geometry, `${color}88`, 1.5);
    drawPolyline(context, frame.drones[droneId].planned_path, geometry, color, 1.5, true);
    drawMotionReservation(context, frame.drones[droneId], geometry, color);
  });

  const [baseX, baseY] = cellCenter(state.replay.map.base, geometry);
  const baseSize = Math.max(6 * ratio, cell * 0.26);
  context.save();
  context.strokeStyle = COLORS.base;
  context.lineWidth = 1.5 * ratio;
  context.strokeRect(baseX - baseSize, baseY - baseSize, baseSize * 2, baseSize * 2);
  context.fillStyle = COLORS.base;
  context.font = `800 ${Math.max(8 * ratio, baseSize * 0.9)}px Inter, system-ui, sans-serif`;
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText("B", baseX, baseY);
  context.restore();

  (knowledgeMap.confirmed_survivors || frame.confirmed_survivors).forEach((position) => {
    const [x, y] = cellCenter(position, geometry);
    context.beginPath();
    context.fillStyle = COLORS.survivor;
    context.arc(x, y, Math.max(3.5 * ratio, cell * 0.18), 0, Math.PI * 2);
    context.fill();
    context.strokeStyle = "#fff3b0";
    context.lineWidth = ratio;
    context.stroke();
  });

  ["drone-1", "drone-2"].forEach((droneId, droneIndex) => {
    const drone = frame.drones[droneId];
    const color = droneIndex === 0 ? COLORS.drone1 : COLORS.drone2;
    if (drone.target) {
      const [x, y] = cellCenter(drone.target, geometry);
      const size = Math.max(4 * ratio, cell * 0.2);
      context.save();
      context.translate(x, y);
      context.rotate(Math.PI / 4);
      context.strokeStyle = drone.relay?.active ? COLORS.radioRelay : color;
      context.lineWidth = (drone.relay?.active ? 2.5 : 1.5) * ratio;
      context.strokeRect(-size, -size, size * 2, size * 2);
      context.restore();
      if (drone.relay?.active) {
        context.fillStyle = COLORS.radioRelay;
        context.font = `800 ${Math.max(8 * ratio, cell * 0.22)}px Inter, system-ui, sans-serif`;
        context.textAlign = "center";
        context.textBaseline = "bottom";
        context.fillText("R", x, y - size - 2 * ratio);
      }
    }
    const [cellX, cellY] = cellCenter(drone.position, geometry);
    const [dockX, dockY] = dockingMarkerOffset(frame, droneId, geometry);
    const x = cellX + dockX;
    const y = cellY + dockY;
    const radius = Math.max(7 * ratio, cell * 0.31);
    context.beginPath();
    context.fillStyle = color;
    context.arc(x, y, radius, 0, Math.PI * 2);
    context.fill();
    context.fillStyle = COLORS.textDark;
    context.font = `800 ${Math.max(9 * ratio, radius)}px Inter, system-ui, sans-serif`;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(String(droneIndex + 1), x, y + ratio * 0.3);
  });
}

function scheduleNext() {
  clearTimeout(state.timer);
  if (!state.playing) return;
  const delay = 430 / state.speed;
  state.timer = setTimeout(() => {
    if (state.frameIndex < state.replay.frames.length - 1) {
      setFrame(state.frameIndex + 1);
      scheduleNext();
    } else {
      pause();
    }
  }, delay);
}

function play() {
  if (state.frameIndex === state.replay.frames.length - 1) setFrame(0);
  state.playing = true;
  elements.playIcon.textContent = "Ⅱ";
  elements.playLabel.textContent = "Pause";
  elements.playButton.setAttribute("aria-label", "Pause replay");
  scheduleNext();
}

function pause() {
  state.playing = false;
  clearTimeout(state.timer);
  elements.playIcon.textContent = "▶";
  elements.playLabel.textContent = "Play";
  elements.playButton.setAttribute("aria-label", "Play replay");
}

function bindControls() {
  elements.playButton.addEventListener("click", () => state.playing ? pause() : play());
  elements.restartButton.addEventListener("click", () => { pause(); setFrame(0); });
  elements.previousButton.addEventListener("click", () => { pause(); setFrame(state.frameIndex - 1); });
  elements.nextButton.addEventListener("click", () => { pause(); setFrame(state.frameIndex + 1); });
  elements.timeline.addEventListener("input", (event) => { pause(); setFrame(Number(event.target.value)); });
  elements.speedSelect.addEventListener("change", (event) => {
    state.speed = Number(event.target.value);
    if (state.playing) scheduleNext();
  });
  elements.mapViewSelect.addEventListener("change", (event) => {
    state.mapView = event.target.value;
    updateMapReadout(state.replay.frames[state.frameIndex]);
    updateEventFeed();
    drawMission();
  });
  window.addEventListener("keydown", (event) => {
    if (event.target.matches("input, select, button")) return;
    if (event.code === "Space") { event.preventDefault(); state.playing ? pause() : play(); }
    if (event.code === "ArrowLeft") { pause(); setFrame(state.frameIndex - 1); }
    if (event.code === "ArrowRight") { pause(); setFrame(state.frameIndex + 1); }
  });
  new ResizeObserver(drawMission).observe(elements.missionCanvas.parentElement);
}

function showFatalError(error) {
  elements.fatalError.hidden = false;
  elements.fatalError.textContent = `Replay dashboard could not start: ${error.message}`;
}

async function main() {
  cacheElements();
  bindControls();
  try {
    const [replay, benchmark] = await Promise.all([
      loadJson("/replay.json"),
      loadOptionalBenchmark("/benchmark.json"),
    ]);
    initializeReplay(replay, benchmark);
  } catch (error) {
    showFatalError(error);
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { normalizeBenchmark, safeBenchmarkView };
}

if (typeof document !== "undefined") {
  document.addEventListener("DOMContentLoaded", main);
}
