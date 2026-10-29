/*
 * Einstellungen -- als eigene Seite (/einstellungen) und als Modal in der
 * Steuerung. Beide haben dieselben Elemente (#einstellungen, #einst-meldung,
 * #alle-zurueck). Alles in einer Funktion, damit sich die Namen hier nicht
 * mit denen aus app.js in die Quere kommen.
 *
 * Alles, was hier steht, kommt vom Roboter (GET /settings): Blöcke,
 * Grenzen, Standardwerte, Hilfetexte. Die Seite kennt keine einzige
 * Einstellung selbst -- kommt auf dem Roboter eine dazu, erscheint sie
 * hier ohne Änderung an dieser Datei.
 *
 * Gespeichert wird jede Änderung sofort (POST /settings), nicht erst mit
 * einem Knopf: sonst probiert man am Roboter etwas aus und wundert sich,
 * warum sich nichts tut.
 */

(() => {

const main = document.getElementById("einstellungen");
const meldung = document.getElementById("einst-meldung");

/** key -> {eintrag, zeile, eingaben...}, um nach dem Speichern nachzuziehen. */
const zeilen = new Map();

function zeigeMeldung(text, fehler = false) {
  meldung.textContent = text;
  meldung.classList.toggle("fehler", fehler);
  meldung.hidden = false;
  clearTimeout(zeigeMeldung.timer);
  zeigeMeldung.timer = setTimeout(() => { meldung.hidden = true; }, fehler ? 8000 : 3000);
}

/** Nachkommastellen aus der Schrittweite, damit 0.1 nicht als 0.10000001 erscheint. */
function stellen(schritt) {
  if (!schritt || Number.isInteger(schritt)) return 0;
  return String(schritt).split(".")[1]?.length ?? 2;
}

function formatiere(e, wert) {
  if (e.typ === "bool") return wert ? "an" : "aus";
  if (e.typ === "float") return Number(wert).toFixed(stellen(e.schritt));
  return String(wert);
}

async function senden(pfad, daten) {
  let antwort;
  try {
    antwort = await fetch(pfad, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(daten),
    });
  } catch {
    zeigeMeldung("Roboter nicht erreichbar", true);
    return null;
  }
  const json = await antwort.json().catch(() => ({}));
  if (!antwort.ok) {
    zeigeMeldung(json.fehler || `Fehler ${antwort.status}`, true);
    return null;
  }
  return json;
}

async function setzen(key, wert) {
  const json = await senden("/settings", { [key]: wert });
  if (json) {
    uebernehmen(json);
    if (json.geaendert?.length) {
      zeigeMeldung(json.neustart_noetig?.length
        ? "Gespeichert, wirkt nach Neustart des Servers"
        : "Gespeichert");
    }
  } else {
    // Abgelehnt: die Anzeige auf den Wert zurückstellen, der wirklich gilt.
    laden();
  }
}

/** Werte aus einer Antwort in die bestehenden Zeilen schreiben. */
function uebernehmen(liste) {
  for (const gruppe of liste.gruppen) {
    for (const e of gruppe.einstellungen) {
      const z = zeilen.get(e.key);
      if (!z) continue;
      z.eintrag = e;
      z.setzeWert(e.wert);
      const geaendert = JSON.stringify(e.wert) !== JSON.stringify(e.standard);
      z.zeile.classList.toggle("geaendert", geaendert);
      z.zurueck.hidden = !geaendert;
    }
  }
}

function baueSteuerung(e) {
  const box = document.createElement("div");
  box.className = "einst-steuerung";
  let setzeWert;

  if (e.typ === "bool") {
    const schalter = document.createElement("input");
    schalter.type = "checkbox";
    schalter.className = "einst-schalter";
    schalter.addEventListener("change", () => setzen(e.key, schalter.checked));
    box.append(schalter);
    setzeWert = (w) => { schalter.checked = Boolean(w); };
  } else if (e.typ === "auswahl") {
    const auswahl = document.createElement("select");
    for (const option of e.optionen) {
      const o = document.createElement("option");
      o.value = String(option);
      o.textContent = e.einheit ? `${option} ${e.einheit}` : String(option);
      auswahl.append(o);
    }
    auswahl.addEventListener("change", () => setzen(e.key, auswahl.value));
    box.append(auswahl);
    setzeWert = (w) => { auswahl.value = String(w); };
  } else {
    // Regler zum Schieben, Zahlenfeld zum genauen Eintippen.
    const regler = document.createElement("input");
    regler.type = "range";
    regler.min = e.min;
    regler.max = e.max;
    regler.step = e.schritt ?? "any";
    const zahl = document.createElement("input");
    zahl.type = "number";
    zahl.min = e.min;
    zahl.max = e.max;
    zahl.step = e.schritt ?? "any";
    const einheit = document.createElement("span");
    einheit.className = "einst-einheit";
    einheit.textContent = e.einheit;

    // Beim Ziehen nur anzeigen; gespeichert wird beim Loslassen, sonst
    // gingen dutzende Anfragen pro Sekunde raus.
    regler.addEventListener("input", () => {
      zahl.value = formatiere(e, regler.value);
    });
    regler.addEventListener("change", () => setzen(e.key, Number(regler.value)));
    zahl.addEventListener("change", () => {
      if (zahl.value === "") return;
      setzen(e.key, Number(zahl.value));
    });
    box.append(regler, zahl, einheit);
    setzeWert = (w) => {
      regler.value = w;
      zahl.value = formatiere(e, w);
    };
  }
  return { box, setzeWert };
}

function baueZeile(e) {
  const zeile = document.createElement("div");
  zeile.className = "einst-zeile";

  const kopf = document.createElement("div");
  kopf.className = "einst-label";
  const label = document.createElement("label");
  label.textContent = e.label;
  kopf.append(label);
  if (e.neustart) {
    const n = document.createElement("span");
    n.className = "einst-neustart";
    n.textContent = "Neustart";
    kopf.append(n);
  }

  const { box, setzeWert } = baueSteuerung(e);

  const zurueck = document.createElement("button");
  zurueck.type = "button";
  zurueck.className = "einst-zurueck";
  zurueck.title = `Standard: ${formatiere(e, e.standard)} ${e.einheit}`.trim();
  zurueck.textContent = "↺";
  zurueck.addEventListener("click", async () => {
    const json = await senden("/settings/reset", { keys: [e.key] });
    if (json) {
      uebernehmen(json);
      zeigeMeldung(`${e.label}: zurück auf ${formatiere(e, e.standard)} ${e.einheit}`.trim());
    }
  });
  box.append(zurueck);

  zeile.append(kopf, box);
  if (e.hilfe) {
    const hilfe = document.createElement("p");
    hilfe.className = "einst-hilfe";
    hilfe.textContent = e.hilfe;
    zeile.append(hilfe);
  }
  const standard = document.createElement("p");
  standard.className = "einst-standard";
  standard.textContent = `Standard: ${formatiere(e, e.standard)} ${e.einheit}`.trim();
  zeile.append(standard);

  zeilen.set(e.key, { eintrag: e, zeile, zurueck, setzeWert });
  return zeile;
}

/* --- Kameras: was tatsächlich ankommt --------------------------------- */

let kameraListe = null;

function baueKameraAnzeige(panel) {
  const titel = document.createElement("h3");
  titel.className = "einst-unter";
  titel.textContent = "Kameras gerade";
  kameraListe = document.createElement("ul");
  kameraListe.className = "einst-kameras";
  panel.append(titel, kameraListe);
  kamerasAktualisieren();
  setInterval(kamerasAktualisieren, 2000);
}

async function kamerasAktualisieren() {
  // Nur nachfragen, solange die Liste zu sehen ist -- im geschlossenen
  // Modal wären das sinnlose Anfragen alle zwei Sekunden.
  if (!kameraListe || kameraListe.offsetParent === null) return;
  let daten;
  try {
    const antwort = await fetch("/cameras", { cache: "no-store" });
    if (!antwort.ok) return;
    daten = await antwort.json();
  } catch {
    return;
  }
  kameraListe.replaceChildren(...(daten.kameras || []).map((k) => {
    const li = document.createElement("li");
    const name = document.createElement("span");
    name.textContent = `${k.kamera} (${k.name})`;
    const wert = document.createElement("span");
    wert.className = k.geoeffnet ? "ok" : "fehlt";
    wert.textContent = k.geoeffnet
      ? `${k.fps_ist ?? "?"} von ${k.fps_soll ?? "?"} fps · ${k.geraet ?? ""}`
      : (k.letzter_fehler || "nicht geöffnet");
    li.append(name, wert);
    return li;
  }));
}

/* --- Aufbau ------------------------------------------------------------ */

async function laden() {
  let liste;
  try {
    const antwort = await fetch("/settings", { cache: "no-store" });
    liste = await antwort.json();
  } catch {
    zeigeMeldung("Roboter nicht erreichbar", true);
    return;
  }

  // Schon aufgebaut (Modal erneut geöffnet): nur die Werte auffrischen --
  // sie können sich inzwischen geändert haben, etwa von einem anderen Gerät.
  if (zeilen.size) {
    uebernehmen(liste);
    kamerasAktualisieren();
    return;
  }

  for (const gruppe of liste.gruppen) {
    if (!gruppe.einstellungen.length) continue;
    const panel = document.createElement("section");
    panel.className = "panel einst-gruppe";
    const h2 = document.createElement("h2");
    h2.textContent = gruppe.titel;
    panel.append(h2);
    if (gruppe.text) {
      const p = document.createElement("p");
      p.className = "einst-text";
      p.textContent = gruppe.text;
      panel.append(p);
    }
    for (const e of gruppe.einstellungen) panel.append(baueZeile(e));
    if (gruppe.id === "kamera") baueKameraAnzeige(panel);
    main.append(panel);
  }
  uebernehmen(liste);
  kamerasAktualisieren(); // erst jetzt sichtbar, vorher würde es übersprungen
}

document.getElementById("alle-zurueck").addEventListener("click", async () => {
  if (!confirm("Alle Einstellungen auf die Werte aus config.py zurücksetzen?")) return;
  const json = await senden("/settings/reset", {});
  if (json) {
    uebernehmen(json);
    zeigeMeldung(json.geaendert?.length
      ? `${json.geaendert.length} Einstellungen zurückgesetzt`
      : "Es war schon alles auf Standard");
  }
});

// Auf der eigenen Seite gleich laden; im Modal der Steuerung erst beim
// Öffnen, damit die Seite beim Start nicht unnötig nachfragt.
window.einstellungenLaden = laden;
if (!document.getElementById("einst-dialog")) laden();
})();
