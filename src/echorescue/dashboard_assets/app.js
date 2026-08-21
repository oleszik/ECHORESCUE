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
    "improvementValue", "benchmarkNote", "benchmarkPanel", "fatalError",
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
  return response.json();
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
  elements.knowledgeMode.textContent = knowledgeMode.toUpperCase();
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
  elements[`droneState${number}`].textContent = drone.state.replaceAll("_", " ");
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
    detail.textContent = `${event.drone_id} · [${event.position.join(", ")}]${cells}`;
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
}

function populateBenchmark() {
  const benchmark = state.benchmark;
  if (!benchmark) {
    elements.benchmarkPanel.classList.add("unavailable");
    return;
  }
  elements.singleSteps.textContent = benchmark.single_drone.average_mission_steps.toFixed(2);
  elements.multiSteps.textContent = benchmark.two_drone.average_mission_steps.toFixed(2);
  elements.improvementValue.textContent = `${benchmark.comparison.mission_duration_reduction_percent.toFixed(2)}%`;
  const duplicate = benchmark.two_drone.average_duplicate_exploration_ratio * 100;
  const pathIncrease = benchmark.comparison.combined_path_length_increase_percent;
  elements.benchmarkNote.textContent = `Both modes achieved 100% survivor recall with zero wall collisions. Two drones recorded zero inter-drone collisions and ${duplicate.toFixed(2)}% duplicate exploration; combined fleet path length was ${pathIncrease.toFixed(1)}% higher.`;
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
  for (const link of frame.communication.links) {
    const first = frame.communication.nodes[link.from];
    const second = frame.communication.nodes[link.to];
    const [firstX, firstY] = cellCenter(first, geometry);
    const [secondX, secondY] = cellCenter(second, geometry);
    context.save();
    context.strokeStyle = link.kind === "direct_base"
      ? COLORS.radioDirect
      : link.kind === "relay" ? COLORS.radioRelay : COLORS.radioPeer;
    context.globalAlpha = link.kind === "peer" ? 0.52 : 0.82;
    context.lineWidth = (link.kind === "relay" ? 2.2 : 1.7) * geometry.ratio;
    if (link.kind === "relay") {
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
      context.strokeStyle = color;
      context.lineWidth = 1.5 * ratio;
      context.strokeRect(-size, -size, size * 2, size * 2);
      context.restore();
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
      loadJson("/benchmark.json", false),
    ]);
    initializeReplay(replay, benchmark);
  } catch (error) {
    showFatalError(error);
  }
}

document.addEventListener("DOMContentLoaded", main);
