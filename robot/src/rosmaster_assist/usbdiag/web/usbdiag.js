// usb diagnosis page: polls /api/state and draws the wiring as an svg plan
"use strict";

const POLL_MS = 1000;
const SVG_NS = "http://www.w3.org/2000/svg";
const NOTHING = "nichts";
const RANK = { bad: 4, warn: 3, unknown: 2, ok: 1, empty: 0 };

// fixed drawing geometry, matching the module ids DiagState sends
const PI = { x: 20, y: 30, w: 200, h: 330 };
const SOCKET = { x: 176, w: 112, h: 46 };
const SOCKET_ROWS = { "pi-usb3-a": 62, "pi-usb3-b": 120, "pi-usb2-a": 202, "pi-usb2-b": 260 };
const SOCKET_CHIP_X = 316;
const BOARD = { x: 520, y: 30, w: 170, h: 300 };
const PORT = { x: 536, w: 138, h: 46, rows: [74, 134, 194, 254] };
const BOARD_CHIP_X = 720;
const MOTOR = { x: 520, y: 382, w: 170, h: 74 };
const CHIP = { h: 46, piW: 190, boardW: 224 };
const VIEWBOX = "0 0 960 470";

const view = {
  state: null,
  selected: null,
  userPicked: false,
  signature: "",
  tiles: {},       // tile id -> {g, sub}
  chips: {},       // tile id -> {g, line}
  links: [],       // {path, label}
  detailKey: "",
  techOpen: false,
  incidentsKey: "",
  openIncidents: new Set(),
};

function el(tag, attrs, parent, text) {
  const node = tag.startsWith("svg:")
    ? document.createElementNS(SVG_NS, tag.slice(4))
    : document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) node.setAttribute(k, v);
  if (text !== undefined) node.textContent = text;
  if (parent) parent.appendChild(node);
  return node;
}

function allTiles(state) {
  return state.modules.flatMap((m) => m.tiles);
}

function findTile(state, id) {
  return allTiles(state).find((t) => t.id === id) || null;
}

function isEmpty(tile) {
  return tile.device === NOTHING;
}

function worstTile(state) {
  let best = null;
  for (const t of allTiles(state)) {
    if (isEmpty(t)) continue;
    if (!best || RANK[t.level] > RANK[best.level]) best = t;
  }
  return best || allTiles(state)[0] || null;
}

// header, verdict, vitals

function renderTop(state) {
  document.getElementById("running").textContent = "läuft seit " + state.running;
  const n = state.incident_count;
  document.getElementById("incident-count").textContent = n === 1 ? "1 Vorfall" : n + " Vorfälle";
  document.getElementById("uptime").textContent = "Pi an seit " + state.uptime;

  const verdict = document.getElementById("verdict");
  verdict.className = "verdict lv-" + state.verdict.level;
  document.getElementById("verdict-text").textContent = state.verdict.text;

  const list = document.getElementById("vitals");
  list.replaceChildren();
  for (const v of state.vitals) {
    const li = el("li", { class: "vital" }, list);
    el("span", { class: "vital-label" }, li, v.label);
    el("span", { class: "vital-value" }, li, v.value);
    el("span", { class: "vital-judgement lv-" + v.level }, li, v.judgement);
  }
  document.getElementById("sampling-note").textContent = state.sampling_note || "";
}

function showOffline() {
  const verdict = document.getElementById("verdict");
  verdict.className = "verdict offline";
  document.getElementById("verdict-text").textContent =
    "Verbindung zum Diagnose-Dienst verloren – versuche erneut …";
}

// diagram

function chipTiles(state) {
  const linked = new Set(state.links.map((l) => l.from));
  return allTiles(state).filter((t) => t.kind !== "motor" && !isEmpty(t) && !linked.has(t.id));
}

function structureSignature(state) {
  return JSON.stringify([
    state.modules.map((m) => [m.id, m.title, m.note, m.tiles.map((t) => [t.id, t.label, t.kind, isEmpty(t)])]),
    state.links.map((l) => [l.from, l.to]),
    chipTiles(state).map((t) => [t.id, t.device]),
    state.links.map((l) => (findTile(state, l.from) || {}).device),
  ]);
}

// tile id -> rectangle, so links and chips can attach to it
function placeTiles(state) {
  const boxes = {};
  for (const m of state.modules) {
    m.tiles.forEach((t, i) => {
      if (m.id === "pi") {
        const y = SOCKET_ROWS[t.id] !== undefined ? SOCKET_ROWS[t.id] : 62 + i * 58;
        boxes[t.id] = { x: SOCKET.x, y, w: SOCKET.w, h: SOCKET.h, side: "pi" };
      } else if (m.id === "board") {
        const y = PORT.rows[i] !== undefined ? PORT.rows[i] : 74 + i * 60;
        boxes[t.id] = { x: PORT.x, y, w: PORT.w, h: PORT.h, side: "board" };
      } else if (m.id === "motor") {
        boxes[t.id] = { x: MOTOR.x, y: MOTOR.y, w: MOTOR.w, h: MOTOR.h, side: "motor" };
      }
      // unknown modules have no place in the plan; they still show up in the incident list
    });
  }
  return boxes;
}

function drawPi(svg, mod) {
  const g = el("svg:g", {}, svg);
  el("svg:rect", { class: "mod-body mod-pi", x: PI.x, y: PI.y, width: PI.w, height: PI.h, rx: 14 }, g);
  // gpio header and soc only hint at the real board so the sockets read as "on the pi"
  for (let i = 0; i < 20; i++) {
    el("svg:rect", { class: "gpio", x: PI.x + 10, y: PI.y + 48 + i * 13, width: 6, height: 6, rx: 1 }, g);
    el("svg:rect", { class: "gpio", x: PI.x + 20, y: PI.y + 48 + i * 13, width: 6, height: 6, rx: 1 }, g);
  }
  el("svg:rect", { class: "soc", x: PI.x + 50, y: PI.y + 128, width: 76, height: 76, rx: 6 }, g);
  el("svg:text", { class: "mod-title", x: PI.x + 40, y: PI.y + 28 }, g, mod.title);
  if (mod.note) el("svg:text", { class: "mod-note", x: PI.x, y: PI.y + PI.h + 22 }, g, mod.note);
}

function drawBoard(svg, mod) {
  const g = el("svg:g", {}, svg);
  el("svg:rect", { class: "mod-body mod-board", x: BOARD.x, y: BOARD.y, width: BOARD.w, height: BOARD.h, rx: 10 }, g);
  el("svg:text", { class: "mod-title", x: BOARD.x + 16, y: BOARD.y + 28 }, g, mod.title);
  if (mod.note) el("svg:text", { class: "mod-note", x: BOARD.x, y: BOARD.y + BOARD.h + 18 }, g, mod.note);
}

function drawWheels(svg) {
  // mecanum wheels on both sides of the motor controller
  const xs = [MOTOR.x - 16, MOTOR.x + MOTOR.w + 2];
  const ys = [MOTOR.y + 4, MOTOR.y + MOTOR.h - 32];
  for (const x of xs) {
    for (const y of ys) {
      el("svg:rect", { class: "wheel", x, y, width: 14, height: 28, rx: 4 }, svg);
      for (let k = 1; k < 4; k++) {
        el("svg:line", { class: "wheel-roller", x1: x + 2, y1: y + k * 7 - 3, x2: x + 12, y2: y + k * 7 + 3 }, svg);
      }
    }
  }
}

function drawTile(svg, tile, box) {
  const g = el("svg:g", {
    class: "tile",
    role: "button",
    tabindex: "0",
    "data-tile": tile.id,
  }, svg);
  el("svg:rect", { class: "halo", x: box.x - 4, y: box.y - 4, width: box.w + 8, height: box.h + 8, rx: 10 }, g);
  el("svg:rect", { class: "face", x: box.x, y: box.y, width: box.w, height: box.h, rx: 7 }, g);
  el("svg:rect", { class: "ring", x: box.x - 3, y: box.y - 3, width: box.w + 6, height: box.h + 6, rx: 9 }, g);

  let textX = box.x + 12;
  if (tile.kind === "socket") {
    // the socket mouth, coloured like the real plastic inside the pi sockets
    const usb3 = tile.label.startsWith("USB3");
    el("svg:rect", { class: "frame " + (usb3 ? "frame-usb3" : "frame-usb2"), x: box.x + 10, y: box.y + 9, width: 22, height: 28, rx: 2 }, g);
    el("svg:rect", { class: "tongue", x: box.x + 15, y: box.y + 14, width: 12, height: 8 }, g);
    textX = box.x + 42;
  }

  let first = tile.label;
  let second = "";
  if (tile.kind === "motor") {
    second = tile.line;
  } else {
    // "Port 4 · Board-MCU" and "USB3 oben" read better on two lines, "Port 2" stays whole
    const sep = tile.label.includes(" · ") ? " · " : tile.kind === "socket" ? " " : null;
    const cut = sep ? tile.label.indexOf(sep) : -1;
    if (cut > 0) {
      first = tile.label.slice(0, cut);
      second = tile.label.slice(cut + sep.length);
    }
  }
  const y1 = second ? box.y + 20 : box.y + box.h / 2 + 5;
  el("svg:text", { class: "tile-label", x: textX, y: y1 }, g, first);
  const sub = second ? el("svg:text", { class: "tile-sub", x: textX, y: box.y + 36 }, g, second) : null;
  view.tiles[tile.id] = { g, sub };
}

function drawChip(svg, tile, box) {
  const x = box.side === "pi" ? SOCKET_CHIP_X : BOARD_CHIP_X;
  const w = box.side === "pi" ? CHIP.piW : CHIP.boardW;
  const cy = box.y + box.h / 2;
  el("svg:line", { class: "lead", x1: box.x + box.w, y1: cy, x2: x, y2: cy }, svg);
  const g = el("svg:g", { class: "chip", "data-tile": tile.id }, svg);
  const tip = el("svg:title", {}, g, tile.device + ": " + tile.line);
  el("svg:rect", { class: "chip-face", x, y: box.y, width: w, height: CHIP.h, rx: 6 }, g);
  fitText(el("svg:text", { class: "chip-name", x: x + 10, y: box.y + 19 }, g), tile.device, w - 20);
  const line = el("svg:text", { class: "chip-line", x: x + 10, y: box.y + 37 }, g);
  fitText(line, tile.line, w - 20);
  view.chips[tile.id] = { g, line, tip, w, device: tile.device };
}

// long status lines get an ellipsis instead of spilling out of their chip; the tooltip keeps the full text
function fitText(node, text, maxWidth) {
  node.textContent = text;
  node.setAttribute("data-full", text);
  if (!node.getComputedTextLength || node.getComputedTextLength() <= maxWidth) return;
  let cut = text.length;
  while (cut > 1 && node.getComputedTextLength() > maxWidth) {
    cut -= 1;
    node.textContent = text.slice(0, cut).trimEnd() + " …";
  }
}

function linkPath(from, to) {
  if (to === "motor") {
    const x = from.x + from.w / 2;
    return { d: `M${x},${from.y + from.h} V${MOTOR.y}`, lx: x + 8, ly: (from.y + from.h + MOTOR.y) / 2 + 4 };
  }
  // cables run straight into the board when the rows line up, otherwise with one jog
  const sx = from.x + from.w;
  const sy = from.y + from.h / 2;
  const ty = Math.min(Math.max(sy, BOARD.y + 40), BOARD.y + BOARD.h - 20);
  const mx = BOARD.x - 24;
  const d = sy === ty ? `M${sx},${sy} H${BOARD.x}` : `M${sx},${sy} H${mx} V${ty} H${BOARD.x}`;
  return { d, lx: sx + 22, ly: sy - 10 };
}

function buildDiagram(state) {
  const svg = document.getElementById("diagram");
  const focused = document.activeElement && document.activeElement.getAttribute
    ? document.activeElement.getAttribute("data-tile") : null;
  svg.replaceChildren();
  svg.setAttribute("viewBox", VIEWBOX);
  view.tiles = {};
  view.chips = {};
  view.links = [];
  const boxes = placeTiles(state);

  for (const m of state.modules) {
    if (m.id === "pi") drawPi(svg, m);
    if (m.id === "board") drawBoard(svg, m);
    if (m.id === "motor") drawWheels(svg);
  }

  for (const l of state.links) {
    const from = boxes[l.from];
    if (!from) continue;
    const p = linkPath(from, l.to);
    const path = el("svg:path", { class: "link", d: p.d }, svg);
    const source = findTile(state, l.from);
    const name = l.to === "board" && source && !isEmpty(source) ? source.device : "";
    const label = el("svg:text", { class: "link-label", x: p.lx, y: p.ly }, svg, name);
    view.links.push({ path, label });
  }

  const chips = new Set(chipTiles(state).map((t) => t.id));
  for (const t of allTiles(state)) {
    const box = boxes[t.id];
    if (!box) continue;
    drawTile(svg, t, box);
    if (chips.has(t.id)) {
      drawChip(svg, t, box);
    } else if (isEmpty(t)) {
      // board ports sit inside the board outline, so their note goes outside it
      const x = box.side === "board" ? BOARD_CHIP_X : box.x + box.w + 14;
      el("svg:text", { class: "free", x, y: box.y + box.h / 2 + 4 }, svg, "frei");
    }
  }

  if (focused && view.tiles[focused]) view.tiles[focused].g.focus();
}

function updateDiagram(state) {
  for (const t of allTiles(state)) {
    const ref = view.tiles[t.id];
    if (!ref) continue;
    ref.g.setAttribute("class", "tile lv-" + t.level + (t.id === view.selected ? " selected" : ""));
    ref.g.setAttribute("aria-label", t.label + ": " + (isEmpty(t) ? "frei" : t.device + ", " + t.line));
    ref.g.setAttribute("aria-pressed", t.id === view.selected ? "true" : "false");
    if (t.kind === "motor" && ref.sub) ref.sub.textContent = t.line;
    const chip = view.chips[t.id];
    if (chip) {
      chip.g.setAttribute("class", "chip lv-" + t.level);
      if (chip.line.getAttribute("data-full") !== t.line) fitText(chip.line, t.line, chip.w - 20);
      chip.tip.textContent = chip.device + ": " + t.line;
    }
  }
  state.links.forEach((l, i) => {
    const ref = view.links[i];
    if (!ref) return;
    ref.path.setAttribute("class", "link lv-" + l.level);
    ref.label.setAttribute("class", "link-label lv-" + l.level);
  });
}

// detail panel

function section(parent, title) {
  el("h3", {}, parent, title);
}

function renderHistory(parent, history) {
  section(parent, "Vorfälle in der letzten Stunde");
  const max = Math.max(1, ...history);
  const bars = el("div", { class: "bars", role: "img", "aria-label": "Vorfälle in der letzten Stunde: " + history.join(", ") }, parent);
  history.forEach((n, i) => {
    const bar = el("div", { class: "bar" + (n ? " has" : "") + (i === history.length - 1 ? " last" : "") }, bars);
    bar.style.height = Math.max(4, (n / max) * 100) + "%";
    bar.title = n === 1 ? "1 Vorfall" : n + " Vorfälle";
  });
  const axis = el("div", { class: "bars-axis" }, parent);
  el("span", {}, axis, "vor einer Stunde");
  el("span", {}, axis, "jetzt");
}

function renderEvents(parent, events, emptyText) {
  if (!events.length) {
    el("p", { class: "empty-text" }, parent, emptyText);
    return;
  }
  const ul = el("ul", { class: "events" }, parent);
  for (const e of events) {
    const li = el("li", {}, ul);
    el("time", {}, li, e.time);
    el("span", {}, li, e.text);
  }
}

function renderReasons(parent, reasons) {
  for (const r of reasons) {
    const box = el("div", { class: "reason" }, parent);
    el("h4", {}, box, r.title);
    if (r.evidence && r.evidence.length) {
      const ul = el("ul", {}, box);
      for (const ev of r.evidence) el("li", {}, ul, ev);
    }
    const action = el("p", { class: "action" }, box);
    el("b", {}, action, "Was tun: ");
    action.appendChild(document.createTextNode(r.action));
  }
}

function renderSpark(parent, values) {
  if (!values || values.length < 2) return;
  section(parent, "Akkuspannung, letzte Minute");
  const lo = Math.min(...values);
  const hi = Math.max(...values);
  const span = Math.max(hi - lo, 0.2);  // a flat line should not look like a big swing
  const w = 300;
  const h = 44;
  const pts = values.map((v, i) => {
    const x = (i / (values.length - 1)) * w;
    const y = h - 4 - ((v - lo) / span) * (h - 8);
    return x.toFixed(1) + "," + y.toFixed(1);
  }).join(" ");
  const svg = el("svg:svg", { class: "spark", viewBox: `0 0 ${w} ${h}`, preserveAspectRatio: "none", role: "img",
    "aria-label": "Akkuspannung zwischen " + lo.toFixed(1).replace(".", ",") + " und " + hi.toFixed(1).replace(".", ",") + " V" }, parent);
  el("svg:polyline", { points: pts }, svg);
}

function renderDetail(state) {
  const tile = findTile(state, view.selected);
  const detail = tile ? state.details[tile.id] : null;
  const curve = tile && tile.kind === "motor" && state.motor ? state.motor.battery_curve : null;
  const key = JSON.stringify([view.selected, tile, detail, curve]);
  if (key === view.detailKey) return;
  view.detailKey = key;

  const aside = document.getElementById("detail");
  const scroll = aside.scrollTop;
  aside.replaceChildren();
  if (!tile || !detail) {
    el("p", { class: "empty-text" }, aside, "Links eine Buchse oder einen Port anklicken.");
    return;
  }
  el("h2", {}, aside, detail.title);
  el("p", { class: "lead-text" }, aside, isEmpty(tile) ? "Hier ist nichts angeschlossen." : tile.line);

  const dl = el("dl", { class: "rows" }, aside);
  for (const r of detail.rows) {
    el("dt", {}, dl, r.label);
    el("dd", {}, dl, r.value);
  }
  if (curve) renderSpark(aside, curve);
  if (detail.reasons.length) {
    section(aside, "Wahrscheinliche Ursache");
    renderReasons(aside, detail.reasons);
  }
  renderHistory(aside, detail.history);
  section(aside, "Letzte Ereignisse");
  renderEvents(aside, detail.events, "Seit dem Start keine Auffälligkeiten.");

  if (detail.technical.length) {
    const tech = el("details", { class: "tech" }, aside);
    tech.open = view.techOpen;
    tech.addEventListener("toggle", () => { view.techOpen = tech.open; });
    el("summary", {}, tech, "Technische Details");
    el("pre", {}, tech, detail.technical.join("\n"));
  }
  aside.scrollTop = scroll;
}

// incident list

function renderIncidents(state) {
  const key = JSON.stringify(state.incidents);
  if (key === view.incidentsKey) return;
  view.incidentsKey = key;
  const box = document.getElementById("incidents");
  box.replaceChildren();
  if (!state.incidents.length) {
    el("p", { class: "empty-text" }, box, "Seit dem Start kein USB-Vorfall. Alles ruhig.");
    return;
  }
  for (const inc of state.incidents) {
    const d = el("details", { class: "incident" }, box);
    d.open = view.openIncidents.has(inc.id);
    d.addEventListener("toggle", () => {
      if (d.open) view.openIncidents.add(inc.id); else view.openIncidents.delete(inc.id);
    });
    const s = el("summary", {}, d);
    el("span", { class: "when" }, s, inc.time);
    el("span", { class: "why" }, s, inc.reason);
    el("span", { class: "where" }, s, inc.where);
    el("span", { class: "dur" }, s, inc.duration);
    el("span", { class: inc.recovered ? "state-ok" : "state-bad" }, s, inc.recovered ? "wieder da" : "noch gestört");

    const body = el("div", { class: "incident-body" }, d);
    const left = el("div", {}, body);
    el("h3", {}, left, "Wahrscheinliche Ursache");
    if (inc.reasons.length) renderReasons(left, inc.reasons);
    else el("p", { class: "empty-text" }, left, "Keine eindeutige Ursache gefunden.");
    const right = el("div", {}, body);
    el("h3", {}, right, "Ablauf");
    renderEvents(right, inc.events, "Keine Einzelereignisse aufgezeichnet.");
  }
}

// wiring it together

function select(id, byUser) {
  view.selected = id;
  if (byUser) view.userPicked = true;
  if (view.state) {
    updateDiagram(view.state);
    renderDetail(view.state);
  }
}

function render(state) {
  view.state = state;
  renderTop(state);
  if (!view.userPicked || !findTile(state, view.selected)) {
    const t = worstTile(state);
    view.selected = t ? t.id : null;
  }
  const sig = structureSignature(state);
  if (sig !== view.signature) {
    view.signature = sig;
    buildDiagram(state);
  }
  updateDiagram(state);
  renderDetail(state);
  renderIncidents(state);
}

async function poll() {
  try {
    const res = await fetch("/api/state", { cache: "no-store" });
    if (!res.ok) throw new Error("status " + res.status);
    render(await res.json());
  } catch (err) {
    // keep the last picture on screen, only the verdict says the data is stale
    showOffline();
  } finally {
    setTimeout(poll, POLL_MS);
  }
}

function setup() {
  document.getElementById("back").href = location.protocol + "//" + location.hostname + ":8000/diagnose";
  const svg = document.getElementById("diagram");
  svg.addEventListener("click", (e) => {
    const target = e.target.closest("[data-tile]");
    if (target) select(target.getAttribute("data-tile"), true);
  });
  svg.addEventListener("keydown", (e) => {
    if (e.key !== "Enter" && e.key !== " ") return;
    const target = e.target.closest(".tile");
    if (!target) return;
    e.preventDefault();
    select(target.getAttribute("data-tile"), true);
  });
  poll();
}

setup();
