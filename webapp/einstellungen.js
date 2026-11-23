/*
 * Settings -- as a page of its own (/einstellungen) and as a modal in the
 * control page. Both have the same elements (#einstellungen, #einst-meldung,
 * #alle-zurueck). Everything inside one function so the names here do not
 * get in the way of those from app.js.
 *
 * Everything shown here comes from the robot (GET /settings): blocks,
 * limits, defaults, help texts. The page does not know a single setting
 * itself -- if the robot gains a new one, it appears here without any
 * change to this file.
 *
 * Every change is saved immediately (POST /settings), not only with a
 * button: otherwise you try something out on the robot and wonder why
 * nothing happens.
 */

(() => {

const main = document.getElementById("einstellungen");
const message = document.getElementById("einst-meldung");

/** key -> {entry, row, inputs...}, to update them after saving. */
const rows = new Map();

function showMessage(text, isError = false) {
  message.textContent = text;
  message.classList.toggle("fehler", isError);
  message.hidden = false;
  clearTimeout(showMessage.timer);
  showMessage.timer = setTimeout(() => { message.hidden = true; }, isError ? 8000 : 3000);
}

/** Decimal places from the step size, so 0.1 does not show up as 0.10000001. */
function decimals(step) {
  if (!step || Number.isInteger(step)) return 0;
  return String(step).split(".")[1]?.length ?? 2;
}

function formatValue(e, value) {
  if (e.typ === "bool") return value ? "an" : "aus";
  if (e.typ === "float") return Number(value).toFixed(decimals(e.schritt));
  return String(value);
}

async function post(path, data) {
  let response;
  try {
    response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data),
    });
  } catch {
    showMessage("Roboter nicht erreichbar", true);
    return null;
  }
  const json = await response.json().catch(() => ({}));
  if (!response.ok) {
    showMessage(json.fehler || `Fehler ${response.status}`, true);
    return null;
  }
  return json;
}

async function setValue(key, value) {
  const json = await post("/settings", { [key]: value });
  if (json) {
    applyValues(json);
    if (json.geaendert?.length) {
      showMessage(json.neustart_noetig?.length
        ? "Gespeichert, wirkt nach Neustart des Servers"
        : "Gespeichert");
    }
  } else {
    // Rejected: reset the display to the value that actually applies.
    load();
  }
}

/** Write the values from a response into the existing rows. */
function applyValues(list) {
  for (const group of list.gruppen) {
    for (const e of group.einstellungen) {
      const r = rows.get(e.key);
      if (!r) continue;
      r.entry = e;
      r.setValue(e.wert);
      const changed = JSON.stringify(e.wert) !== JSON.stringify(e.standard);
      r.row.classList.toggle("geaendert", changed);
      r.resetButton.hidden = !changed;
    }
  }
}

function buildControl(e) {
  const box = document.createElement("div");
  box.className = "einst-steuerung";
  let setDisplay;

  if (e.typ === "bool") {
    const toggle = document.createElement("input");
    toggle.type = "checkbox";
    toggle.className = "einst-schalter";
    toggle.addEventListener("change", () => setValue(e.key, toggle.checked));
    box.append(toggle);
    setDisplay = (w) => { toggle.checked = Boolean(w); };
  } else if (e.typ === "auswahl") {
    const select = document.createElement("select");
    for (const option of e.optionen) {
      const o = document.createElement("option");
      o.value = String(option);
      o.textContent = e.einheit ? `${option} ${e.einheit}` : String(option);
      select.append(o);
    }
    select.addEventListener("change", () => setValue(e.key, select.value));
    box.append(select);
    setDisplay = (w) => { select.value = String(w); };
  } else {
    // Slider for dragging, number field for typing an exact value.
    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = e.min;
    slider.max = e.max;
    slider.step = e.schritt ?? "any";
    const number = document.createElement("input");
    number.type = "number";
    number.min = e.min;
    number.max = e.max;
    number.step = e.schritt ?? "any";
    const unit = document.createElement("span");
    unit.className = "einst-einheit";
    unit.textContent = e.einheit;

    // While dragging only display; saving happens on release, otherwise
    // dozens of requests per second would go out.
    slider.addEventListener("input", () => {
      number.value = formatValue(e, slider.value);
    });
    slider.addEventListener("change", () => setValue(e.key, Number(slider.value)));
    number.addEventListener("change", () => {
      if (number.value === "") return;
      setValue(e.key, Number(number.value));
    });
    box.append(slider, number, unit);
    setDisplay = (w) => {
      slider.value = w;
      number.value = formatValue(e, w);
    };
  }
  return { box, setDisplay };
}

function buildRow(e) {
  const row = document.createElement("div");
  row.className = "einst-zeile";

  const head = document.createElement("div");
  head.className = "einst-label";
  const label = document.createElement("label");
  label.textContent = e.label;
  head.append(label);
  if (e.neustart) {
    const n = document.createElement("span");
    n.className = "einst-neustart";
    n.textContent = "Neustart";
    head.append(n);
  }

  const { box, setDisplay } = buildControl(e);

  const resetButton = document.createElement("button");
  resetButton.type = "button";
  resetButton.className = "einst-zurueck";
  resetButton.title = `Standard: ${formatValue(e, e.standard)} ${e.einheit}`.trim();
  resetButton.textContent = "↺";
  resetButton.addEventListener("click", async () => {
    const json = await post("/settings/reset", { keys: [e.key] });
    if (json) {
      applyValues(json);
      showMessage(`${e.label}: zurück auf ${formatValue(e, e.standard)} ${e.einheit}`.trim());
    }
  });
  box.append(resetButton);

  row.append(head, box);
  if (e.hilfe) {
    const help = document.createElement("p");
    help.className = "einst-hilfe";
    help.textContent = e.hilfe;
    row.append(help);
  }
  const standard = document.createElement("p");
  standard.className = "einst-standard";
  standard.textContent = `Standard: ${formatValue(e, e.standard)} ${e.einheit}`.trim();
  row.append(standard);

  rows.set(e.key, { entry: e, row, resetButton, setValue: setDisplay });
  return row;
}

/* --- Cameras: what actually arrives ------------------------------------ */

let cameraList = null;

function buildCameraDisplay(panel) {
  const title = document.createElement("h3");
  title.className = "einst-unter";
  title.textContent = "Kameras gerade";
  cameraList = document.createElement("ul");
  cameraList.className = "einst-kameras";
  panel.append(title, cameraList);
  refreshCameras();
  setInterval(refreshCameras, 2000);
}

async function refreshCameras() {
  // Only poll while the list is visible -- in the closed modal these
  // would be pointless requests every two seconds.
  if (!cameraList || cameraList.offsetParent === null) return;
  let data;
  try {
    const response = await fetch("/cameras", { cache: "no-store" });
    if (!response.ok) return;
    data = await response.json();
  } catch {
    return;
  }
  cameraList.replaceChildren(...(data.kameras || []).map((k) => {
    const li = document.createElement("li");
    const name = document.createElement("span");
    name.textContent = `${k.kamera} (${k.name})`;
    const value = document.createElement("span");
    value.className = k.geoeffnet ? "ok" : "fehlt";
    value.textContent = k.geoeffnet
      ? `${k.fps_ist ?? "?"} von ${k.fps_soll ?? "?"} fps · ${k.geraet ?? ""}`
      : (k.letzter_fehler || "nicht geöffnet");
    li.append(name, value);
    return li;
  }));
}

/* --- Building the page ------------------------------------------------- */

async function load() {
  let list;
  try {
    const response = await fetch("/settings", { cache: "no-store" });
    list = await response.json();
  } catch {
    showMessage("Roboter nicht erreichbar", true);
    return;
  }

  // Already built (modal opened again): only refresh the values --
  // they may have changed meanwhile, e.g. from another device.
  if (rows.size) {
    applyValues(list);
    refreshCameras();
    return;
  }

  for (const group of list.gruppen) {
    if (!group.einstellungen.length) continue;
    const panel = document.createElement("section");
    panel.className = "panel einst-gruppe";
    const h2 = document.createElement("h2");
    h2.textContent = group.titel;
    panel.append(h2);
    if (group.text) {
      const p = document.createElement("p");
      p.className = "einst-text";
      p.textContent = group.text;
      panel.append(p);
    }
    for (const e of group.einstellungen) panel.append(buildRow(e));
    if (group.id === "kamera") buildCameraDisplay(panel);
    main.append(panel);
  }
  applyValues(list);
  refreshCameras(); // only visible now, before it would have been skipped
}

document.getElementById("alle-zurueck").addEventListener("click", async () => {
  if (!confirm("Alle Einstellungen auf die Werte aus config.py zurücksetzen?")) return;
  const json = await post("/settings/reset", {});
  if (json) {
    applyValues(json);
    showMessage(json.geaendert?.length
      ? `${json.geaendert.length} Einstellungen zurückgesetzt`
      : "Es war schon alles auf Standard");
  }
});

// On its own page load right away; in the control page's modal only on
// opening, so the page does not poll needlessly at startup.
window.einstellungenLaden = load;
if (!document.getElementById("einst-dialog")) load();
})();
