// page /tests: start measuring runs, follow them live, add notes and upload
// the server does all the work, the page only polls

"use strict";

const el = (id) => document.getElementById(id);

const ui = {
  status: el("status"),
  statusText: el("status-text"),
  sim: el("ts-sim"),
  live: el("ts-live"),
  liveTitle: el("ts-live-title"),
  stop: el("ts-stop"),
  phaseNr: el("ts-phase-nr"),
  phaseTitle: el("ts-phase-title"),
  lead: el("ts-lead"),
  instruction: el("ts-instruction"),
  sentence: el("ts-sentence"),
  rest: el("ts-rest"),
  values: el("ts-values"),
  message: el("ts-message"),
  categories: el("ts-categories"),
  list: el("ts-list"),
  runs: el("ts-runs"),
  none: el("ts-none"),
  folder: el("ts-folder"),
  uploadSelected: el("ts-upload-selected"),
  uploadNew: el("ts-upload-new"),
  uploadText: el("ts-upload-text"),
  uploadStatus: el("ts-upload-status"),
  token: el("ts-token"),
  tokenState: el("ts-token-state"),
};

let catalog = { kategorien: {}, tests: [] };
let category = null;
let active = false;
let lastRun = null;     // name of the last stored run; when it changes, reload the list
let uploading = false;
let serverUploading = false;  // automatic upload of the server in progress

// poll more often during a test so countdown and level look smooth
const POLL_ACTIVE_MS = 250;
const POLL_IDLE_MS = 2000;

const LYING_TEXT = { auffaellig: "liegt", ok: "steht/sitzt", unbekannt: "?" };
// gehoert.ergebnis of the speech recognition
const HEARD_TEXT = { befehl: "Befehl", verworfen: "verworfen" };

function setConnected(yes) {
  ui.status.className = `status ${yes ? "online" : "offline"}`;
  ui.statusText.textContent = yes ? "verbunden" : "getrennt";
}

async function fetchJson(path, options) {
  const response = await fetch(path, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.fehler || data.detail || `HTTP ${response.status}`);
  return data;
}

function post(path, body) {
  return fetchJson(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
}

function node(tag, className, text) {
  const n = document.createElement(tag);
  if (className) n.className = className;
  if (text !== undefined && text !== null) n.textContent = text;
  return n;
}

function showMessage(text, error) {
  ui.message.hidden = !text;
  ui.message.textContent = text || "";
  ui.message.classList.toggle("ts-error", !!error);
}

// iso time from the server ("2026-11-05T14:03:12") as a readable local time
function dateTime(iso) {
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? iso : d.toLocaleString("de-DE");
}

// --- catalog --------------------------------------------------------------

function renderTabs() {
  ui.categories.replaceChildren();
  for (const [key, name] of Object.entries(catalog.kategorien)) {
    const count = catalog.tests.filter((t) => t.kategorie === key).length;
    const button = node("button", "ts-tab", `${name} (${count})`);
    button.type = "button";
    button.setAttribute("role", "tab");
    button.setAttribute("aria-selected", String(key === category));
    button.addEventListener("click", () => {
      category = key;
      renderTabs();
      renderTests();
    });
    ui.categories.append(button);
  }
}

function renderTests() {
  ui.list.replaceChildren();
  for (const test of catalog.tests.filter((t) => t.kategorie === category)) {
    const card = node("article", "ts-card");
    card.append(node("h3", null, test.titel), node("p", "ts-summary", test.kurz));

    if (test.anleitung && test.anleitung.length) {
      const ol = node("ol", "ts-steps");
      for (const step of test.anleitung) ol.append(node("li", null, step));
      card.append(node("h4", null, "Vorbereiten"), ol);
    }

    const total = test.phasen.reduce((s, p) => s + (p.dauer_s || 0), 0);
    card.append(node("h4", null, `Ablauf (${test.phasen.length} Phasen, etwa ${Math.round(total)} s)`));
    const phases = node("ol", "ts-phases");
    for (const p of test.phasen) {
      const li = node("li");
      li.append(node("strong", null, p.titel), document.createTextNode(` (${p.dauer_s} s): ${p.anweisung}`));
      if (p.satz) li.append(node("span", "ts-sentence-small", ` Sagen: „${p.satz}“`));
      phases.append(li);
    }
    card.append(phases);

    const start = node("button", "btn btn-primary ts-start", "Start");
    start.type = "button";
    start.disabled = active;
    start.addEventListener("click", () => startTest(test.id));
    card.append(start);
    ui.list.append(card);
  }
}

async function loadCatalog() {
  catalog = await fetchJson("/tests/katalog");
  if (!category || !(category in catalog.kategorien)) category = Object.keys(catalog.kategorien)[0];
  renderTabs();
  renderTests();
}

async function startTest(id) {
  try {
    showStatus(await post("/tests/start", { test: id }));
    showMessage("");
    window.scrollTo({ top: 0, behavior: "smooth" });
  } catch (e) {
    showMessage(`Start nicht möglich: ${e.message}`, true);
  }
}

ui.stop.addEventListener("click", async () => {
  try {
    showStatus(await post("/tests/stop"));
  } catch (e) {
    showMessage(`Abbrechen fehlgeschlagen: ${e.message}`, true);
  }
});

// --- running test -----------------------------------------------------------

function value(name, content) {
  const row = node("div", "ts-value");
  row.append(node("span", "ts-value-name", name));
  if (content instanceof Node) row.append(content);
  else row.append(node("span", "ts-value-content", content));
  return row;
}

function heardText(g) {
  let text = `„${g.text}“`;
  if (g.befehl) text += ` → ${g.befehl}`;
  if (typeof g.konfidenz === "number") text += ` (Sicherheit ${Math.round(g.konfidenz * 100)} %)`;
  // a rejected sentence explains itself through the reason
  if (g.ergebnis && g.ergebnis !== "befehl") {
    text += ` · ${HEARD_TEXT[g.ergebnis] || g.ergebnis}`;
    if (g.grund) text += `: ${g.grund}`;
  }
  return text;
}

function renderValues(st) {
  ui.values.replaceChildren();
  const live = st.live || {};
  if (st.art === "mikro") {
    const bar = node("div", "ts-level");
    const fill = node("div", "ts-level-fill");
    const level = typeof live.pegel === "number" ? Math.max(0, Math.min(1, live.pegel)) : 0;
    fill.style.width = `${Math.round(level * 100)}%`;
    bar.append(fill);
    ui.values.append(value("Pegel", bar));
    ui.values.append(value("hört gerade", live.teil || "–"));
    ui.values.append(value("zuletzt erkannt", st.gehoert ? heardText(st.gehoert) : "noch nichts"));
    return;
  }
  // the lead-in phase has no measurement yet
  if (st.vorlauf && !st.live) return;
  ui.values.append(value("Personen (Pose)", String(live.personen ?? "–")));
  if (live.liegend && live.liegend.length) {
    const text = live.liegend.map((z, i) => {
      const w = live.rumpfwinkel ? live.rumpfwinkel[i] : null;
      return `${LYING_TEXT[z] || z}${typeof w === "number" ? ` (Rumpf ${Math.round(w)}°)` : ""}`;
    }).join(", ");
    ui.values.append(value("Haltung", text));
  }
  // only the people-detection tests run the hog detector as well
  if ("hog" in live) ui.values.append(value("Personen (HOG)", String(live.hog)));
}

function showUploadStatus(up) {
  if (!up) {
    ui.uploadStatus.hidden = true;
    return;
  }
  serverUploading = !!up.laeuft;
  const parts = [];
  if (up.laeuft) parts.push("Lädt gerade hoch …");
  if (up.ausstehend) parts.push(`${up.ausstehend} Lauf/Läufe noch nicht hochgeladen.`);
  if (up.letzter) {
    parts.push(`Zuletzt hochgeladen: ${dateTime(up.letzter.zeit)}, ${up.letzter.anzahl} Lauf/Läufe`
      + ` (Commit ${(up.letzter.commit || "").slice(0, 7)}).`);
  }
  if (up.auto && !up.hinweis) parts.push("Neue Läufe werden automatisch hochgeladen.");
  if (up.hinweis) parts.push(up.hinweis);
  ui.uploadStatus.hidden = parts.length === 0;
  ui.uploadStatus.textContent = parts.join(" ");
  ui.uploadStatus.className = `hb-note${up.fehler ? " ts-error" : up.hinweis ? " ts-warn" : ""}`;
}

function showStatus(st) {
  const wasActive = active;
  active = !!st.aktiv;
  ui.sim.hidden = !st.simulation;
  ui.folder.textContent = st.ordner || "–";
  ui.tokenState.textContent = st.token ? "· Token vorhanden" : "· kein Token";
  ui.tokenState.className = st.token ? "ts-ok" : "ts-warn";
  // without a token open the instructions right away, otherwise people search for them
  if (!st.token && !ui.token.dataset.touched) ui.token.open = true;
  showUploadStatus(st.upload);

  ui.live.hidden = !active;
  if (active) {
    ui.liveTitle.textContent = st.titel || st.test || "Test";
    ui.phaseNr.textContent = `Phase ${(st.phase ?? 0) + 1}/${st.phasen ?? "?"}`;
    ui.phaseTitle.textContent = st.phase_titel || "";
    ui.lead.hidden = !st.vorlauf;
    ui.instruction.textContent = st.anweisung || "";
    ui.sentence.hidden = !st.satz;
    ui.sentence.textContent = st.satz ? `Sagen: „${st.satz}“` : "";
    ui.rest.textContent = typeof st.rest_s === "number" ? Math.ceil(st.rest_s) : "–";
    renderValues(st);
  }
  if (st.meldung && !active) showMessage(st.meldung, false);

  if (wasActive !== active) renderTests();
  if (st.letzter !== lastRun) {
    lastRun = st.letzter;
    loadList();
  }
}

// --- results -----------------------------------------------------------------

function size(bytes) {
  if (!bytes) return "";
  return bytes > 1e6 ? `${(bytes / 1e6).toFixed(1)} MB` : `${Math.round(bytes / 1e3)} kB`;
}

// run names start with "YYYY-MM-DD_HH-MM-SS", used when "beginn" is missing
function runTime(run) {
  if (run.beginn) return dateTime(run.beginn);
  return run.name.slice(0, 19).replace("_", " ").replace(/-(\d\d)-(\d\d)$/, ":$1:$2");
}

function renderRun(run) {
  const health = run.typ === "health";
  const card = node("article", "ts-run");
  const head = node("label", "ts-run-head");
  const pick = node("input");
  pick.type = "checkbox";
  pick.value = run.name;
  pick.className = "ts-pick";
  head.append(pick, node("strong", null, run.titel || run.test || run.name), node("span", "ts-time", runTime(run)));
  if (health) head.append(node("span", "hb-badge", "Gesundheitscheck"));
  if (run.abgebrochen) head.append(node("span", "hb-badge z-abgebrochen", "abgebrochen"));
  if (run.hochgeladen) {
    const badge = node("span", "hb-badge z-ansprechbar", "hochgeladen");
    badge.title = `${run.upload_zeit ? dateTime(run.upload_zeit) : ""} Commit ${(run.upload_commit || "").slice(0, 7)}`.trim();
    head.append(badge);
  } else if (run.upload_fehler) {
    const badge = node("span", "hb-badge z-hilfe", "Upload fehlgeschlagen");
    badge.title = run.upload_fehler;
    head.append(badge);
  } else {
    head.append(node("span", "hb-badge", "neu"));
  }
  card.append(head);

  if (run.urteil) card.append(node("p", "ts-verdict", run.urteil));
  if (run.fehler) card.append(node("p", "ts-error", run.fehler));
  if (run.upload_fehler && !run.hochgeladen) card.append(node("p", "ts-error", `Hochladen: ${run.upload_fehler}`));
  // health records have no phases; their content is in the verdict
  if (!health && run.phasen && run.phasen.length) {
    const ul = node("ul", "ts-results");
    for (const p of run.phasen) {
      const li = node("li");
      li.append(node("strong", null, `${p.titel}: `), document.createTextNode(p.kurz || "keine Daten"));
      ul.append(li);
    }
    card.append(ul);
  }

  const row = node("div", "hb-row");
  const note = node("input");
  note.type = "text";
  note.placeholder = "Notiz: Was fiel auf? (Licht, Abstand, Kleidung …)";
  note.value = run.notiz || "";
  note.maxLength = 2000;
  const save = node("button", "btn", "Notiz speichern");
  save.type = "button";
  save.addEventListener("click", async () => {
    try {
      await post("/tests/notiz", { name: run.name, text: note.value });
      save.textContent = "gespeichert";
      setTimeout(loadList, 800);
    } catch (e) {
      save.textContent = `Fehler: ${e.message}`;
    }
  });
  const zip = node("a", "btn", `ZIP ${size(run.bytes)}`.trim());
  zip.href = `/tests/zip?name=${encodeURIComponent(run.name)}`;
  zip.setAttribute("download", `${run.name}.zip`);
  row.append(note, save, zip);
  card.append(row);
  return card;
}

async function loadList() {
  try {
    const { laeufe } = await fetchJson("/tests/liste");
    // remember ticked runs so redrawing keeps the selection
    const picked = new Set([...ui.runs.querySelectorAll(".ts-pick:checked")].map((k) => k.value));
    // do not redraw while someone types a note
    if (ui.runs.contains(document.activeElement) && document.activeElement.type === "text") return;
    ui.runs.replaceChildren(...laeufe.map(renderRun));
    for (const k of ui.runs.querySelectorAll(".ts-pick")) k.checked = picked.has(k.value);
    ui.none.hidden = laeufe.length > 0;
    ui.uploadNew.disabled = uploading || !laeufe.some((l) => !l.hochgeladen);
  } catch (e) {
    ui.uploadText.textContent = `Liste nicht abrufbar: ${e.message}`;
  }
}

async function upload(names) {
  if (uploading) return;
  uploading = true;
  ui.uploadSelected.disabled = ui.uploadNew.disabled = true;
  ui.uploadText.className = "hb-note";
  ui.uploadText.textContent = "Lade hoch …";
  try {
    const r = await post("/tests/hochladen", names ? { namen: names } : {});
    if (r.ok) {
      ui.uploadText.className = "hb-note ts-ok";
      ui.uploadText.textContent = r.hochgeladen.length
        ? `Hochgeladen: ${r.hochgeladen.length} Lauf/Läufe (Commit ${(r.commit || "").slice(0, 7)}).`
        : "Nichts Neues zum Hochladen.";
    } else {
      ui.uploadText.className = "hb-note ts-error";
      ui.uploadText.textContent = `Hochladen fehlgeschlagen: ${r.fehler}`;
    }
  } catch (e) {
    ui.uploadText.className = "hb-note ts-error";
    ui.uploadText.textContent = `Hochladen fehlgeschlagen: ${e.message}`;
  } finally {
    uploading = false;
    ui.uploadSelected.disabled = false;
    loadList();
  }
}

ui.uploadNew.addEventListener("click", () => upload(null));
ui.uploadSelected.addEventListener("click", () => {
  const names = [...ui.runs.querySelectorAll(".ts-pick:checked")].map((k) => k.value);
  if (!names.length) {
    ui.uploadText.className = "hb-note ts-warn";
    ui.uploadText.textContent = "Erst Läufe anhaken.";
    return;
  }
  upload(names);
});
ui.token.addEventListener("toggle", () => { ui.token.dataset.touched = "1"; });

// --- polling loop ------------------------------------------------------------

async function poll() {
  const wasUploading = serverUploading;
  try {
    showStatus(await fetchJson("/tests/status"));
    setConnected(true);
  } catch {
    setConnected(false);
  }
  // the automatic upload marks runs as uploaded without a new run, so refresh the badges
  if (wasUploading && !serverUploading) loadList();
  setTimeout(poll, active ? POLL_ACTIVE_MS : POLL_IDLE_MS);
}

async function init() {
  try {
    await loadCatalog();
  } catch (e) {
    showMessage(`Testliste nicht abrufbar: ${e.message}`, true);
    setTimeout(init, POLL_IDLE_MS);
    return;
  }
  loadList();
  poll();
}

init();
