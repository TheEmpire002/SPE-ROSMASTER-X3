"use strict";

// Seite „Tests“: Messläufe starten, live verfolgen, Ergebnisse notieren und
// hochladen. Der Server macht die ganze Arbeit; die Seite fragt nur ab.

const el = (id) => document.getElementById(id);

const ui = {
  status: el("status"),
  statusText: el("status-text"),
  sim: el("ts-sim"),
  live: el("ts-live"),
  liveTitel: el("ts-live-titel"),
  stop: el("ts-stop"),
  phaseNr: el("ts-phase-nr"),
  phaseTitel: el("ts-phase-titel"),
  vorlauf: el("ts-vorlauf"),
  anweisung: el("ts-anweisung"),
  satz: el("ts-satz"),
  rest: el("ts-rest"),
  werte: el("ts-werte"),
  meldung: el("ts-meldung"),
  kategorien: el("ts-kategorien"),
  liste: el("ts-liste"),
  laeufe: el("ts-laeufe"),
  keine: el("ts-keine"),
  ordner: el("ts-ordner"),
  hochAuswahl: el("ts-hoch-auswahl"),
  hochNeu: el("ts-hoch-neu"),
  hochText: el("ts-hoch-text"),
  token: el("ts-token"),
  tokenStand: el("ts-token-stand"),
};

let katalog = { kategorien: {}, tests: [] };
let kategorie = null;
let aktiv = false;
let letzter = null;     // Name des zuletzt gespeicherten Laufs; ändert er sich, Liste neu laden
let hochladen = false;

// Läuft ein Test, öfter fragen, damit Countdown und Pegel flüssig wirken.
const TAKT_AKTIV_MS = 250;
const TAKT_RUHE_MS = 2000;

const LIEGEND_TEXT = { auffaellig: "liegt", ok: "steht/sitzt", unbekannt: "?" };

function verbunden(ja) {
  ui.status.className = `status ${ja ? "online" : "offline"}`;
  ui.statusText.textContent = ja ? "verbunden" : "getrennt";
}

async function holen(pfad, optionen) {
  const antwort = await fetch(pfad, optionen);
  const daten = await antwort.json().catch(() => ({}));
  if (!antwort.ok) throw new Error(daten.fehler || daten.detail || `HTTP ${antwort.status}`);
  return daten;
}

function senden(pfad, body) {
  return holen(pfad, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
}

function knoten(tag, klasse, text) {
  const k = document.createElement(tag);
  if (klasse) k.className = klasse;
  if (text !== undefined && text !== null) k.textContent = text;
  return k;
}

function meldung(text, fehler) {
  ui.meldung.hidden = !text;
  ui.meldung.textContent = text || "";
  ui.meldung.classList.toggle("ts-fehler", !!fehler);
}

// --- Katalog ----------------------------------------------------------------

function tabsZeichnen() {
  ui.kategorien.replaceChildren();
  for (const [schluessel, name] of Object.entries(katalog.kategorien)) {
    const anzahl = katalog.tests.filter((t) => t.kategorie === schluessel).length;
    const knopf = knoten("button", "ts-tab", `${name} (${anzahl})`);
    knopf.type = "button";
    knopf.setAttribute("role", "tab");
    knopf.setAttribute("aria-selected", String(schluessel === kategorie));
    knopf.addEventListener("click", () => {
      kategorie = schluessel;
      tabsZeichnen();
      testsZeichnen();
    });
    ui.kategorien.append(knopf);
  }
}

function testsZeichnen() {
  ui.liste.replaceChildren();
  for (const test of katalog.tests.filter((t) => t.kategorie === kategorie)) {
    const karte = knoten("article", "ts-karte");
    karte.append(knoten("h3", null, test.titel), knoten("p", "ts-kurz", test.kurz));

    if (test.anleitung && test.anleitung.length) {
      const ol = knoten("ol", "ts-anleitung");
      for (const schritt of test.anleitung) ol.append(knoten("li", null, schritt));
      karte.append(knoten("h4", null, "Vorbereiten"), ol);
    }

    const gesamt = test.phasen.reduce((s, p) => s + (p.dauer_s || 0), 0);
    karte.append(knoten("h4", null, `Ablauf (${test.phasen.length} Phasen, etwa ${Math.round(gesamt)} s)`));
    const phasen = knoten("ol", "ts-phasen");
    for (const p of test.phasen) {
      const li = knoten("li");
      li.append(knoten("strong", null, p.titel), document.createTextNode(` (${p.dauer_s} s): ${p.anweisung}`));
      if (p.satz) li.append(knoten("span", "ts-satz-klein", ` Sagen: „${p.satz}“`));
      phasen.append(li);
    }
    karte.append(phasen);

    const start = knoten("button", "btn btn-primary ts-start", "Start");
    start.type = "button";
    start.disabled = aktiv;
    start.addEventListener("click", () => starten(test.id));
    karte.append(start);
    ui.liste.append(karte);
  }
}

async function katalogLaden() {
  katalog = await holen("/tests/katalog");
  if (!kategorie || !(kategorie in katalog.kategorien)) kategorie = Object.keys(katalog.kategorien)[0];
  tabsZeichnen();
  testsZeichnen();
}

async function starten(id) {
  try {
    zeigeStatus(await senden("/tests/start", { test: id }));
    meldung("");
    window.scrollTo({ top: 0, behavior: "smooth" });
  } catch (e) {
    meldung(`Start nicht möglich: ${e.message}`, true);
  }
}

ui.stop.addEventListener("click", async () => {
  try {
    zeigeStatus(await senden("/tests/stop"));
  } catch (e) {
    meldung(`Abbrechen fehlgeschlagen: ${e.message}`, true);
  }
});

// --- Laufender Test -----------------------------------------------------------

function wert(name, inhalt) {
  const zeile = knoten("div", "ts-wert");
  zeile.append(knoten("span", "ts-wert-name", name));
  if (inhalt instanceof Node) zeile.append(inhalt);
  else zeile.append(knoten("span", "ts-wert-inhalt", inhalt));
  return zeile;
}

function werteZeichnen(st) {
  ui.werte.replaceChildren();
  const live = st.live || {};
  if (st.art === "mikro") {
    const balken = knoten("div", "ts-pegel");
    const fuellung = knoten("div", "ts-pegel-wert");
    const pegel = typeof live.pegel === "number" ? Math.max(0, Math.min(1, live.pegel)) : 0;
    fuellung.style.width = `${Math.round(pegel * 100)}%`;
    balken.append(fuellung);
    ui.werte.append(wert("Pegel", balken));
    ui.werte.append(wert("hört gerade", live.teil || "–"));
    const g = st.gehoert;
    if (g) {
      const text = `„${g.text}“` + (g.befehl ? ` → ${g.befehl}` : "")
        + (typeof g.konfidenz === "number" ? ` (Sicherheit ${Math.round(g.konfidenz * 100)} %)` : "");
      ui.werte.append(wert("zuletzt erkannt", text));
    } else {
      ui.werte.append(wert("zuletzt erkannt", "noch nichts"));
    }
    return;
  }
  if (st.vorlauf && !st.live) return;
  ui.werte.append(wert("Personen (Pose)", String(live.personen ?? "–")));
  if (live.liegend && live.liegend.length) {
    const text = live.liegend.map((z, i) => {
      const w = live.rumpfwinkel ? live.rumpfwinkel[i] : null;
      return `${LIEGEND_TEXT[z] || z}${typeof w === "number" ? ` (Rumpf ${Math.round(w)}°)` : ""}`;
    }).join(", ");
    ui.werte.append(wert("Haltung", text));
  }
  if ("hog" in live) ui.werte.append(wert("Personen (HOG)", String(live.hog)));
}

function zeigeStatus(st) {
  const warAktiv = aktiv;
  aktiv = !!st.aktiv;
  ui.sim.hidden = !st.simulation;
  ui.ordner.textContent = st.ordner || "–";
  ui.tokenStand.textContent = st.token ? "· Token vorhanden" : "· kein Token";
  ui.tokenStand.className = st.token ? "ts-ok" : "ts-warn";
  // Ohne Token die Anleitung gleich aufklappen, sonst sucht man sie.
  if (!st.token && !ui.token.dataset.angefasst) ui.token.open = true;

  ui.live.hidden = !aktiv;
  if (aktiv) {
    ui.liveTitel.textContent = st.titel || st.test || "Test";
    ui.phaseNr.textContent = `Phase ${(st.phase ?? 0) + 1}/${st.phasen ?? "?"}`;
    ui.phaseTitel.textContent = st.phase_titel || "";
    ui.vorlauf.hidden = !st.vorlauf;
    ui.anweisung.textContent = st.anweisung || "";
    ui.satz.hidden = !st.satz;
    ui.satz.textContent = st.satz ? `Sagen: „${st.satz}“` : "";
    ui.rest.textContent = typeof st.rest_s === "number" ? Math.ceil(st.rest_s) : "–";
    werteZeichnen(st);
  }
  if (st.meldung && !aktiv) meldung(st.meldung, false);

  if (warAktiv !== aktiv) testsZeichnen();
  if (st.letzter !== letzter) {
    letzter = st.letzter;
    listeLaden();
  }
}

// --- Ergebnisse ----------------------------------------------------------------

function groesse(bytes) {
  if (!bytes) return "";
  return bytes > 1e6 ? `${(bytes / 1e6).toFixed(1)} MB` : `${Math.round(bytes / 1e3)} kB`;
}

function laufZeichnen(lauf) {
  const karte = knoten("article", "ts-lauf");
  const kopf = knoten("label", "ts-lauf-kopf");
  const wahl = knoten("input");
  wahl.type = "checkbox";
  wahl.value = lauf.name;
  wahl.className = "ts-wahl";
  kopf.append(wahl, knoten("strong", null, lauf.titel || lauf.test || lauf.name),
    knoten("span", "ts-zeit", lauf.name.slice(0, 19).replace("_", " ").replace(/-(\d\d)-(\d\d)$/, ":$1:$2")));
  if (lauf.abgebrochen) kopf.append(knoten("span", "hb-marke z-abgebrochen", "abgebrochen"));
  kopf.append(lauf.hochgeladen
    ? knoten("span", "hb-marke z-ansprechbar", "hochgeladen")
    : knoten("span", "hb-marke", "neu"));
  karte.append(kopf);

  if (lauf.fehler) karte.append(knoten("p", "ts-fehler", lauf.fehler));
  if (lauf.phasen && lauf.phasen.length) {
    const ul = knoten("ul", "ts-ergebnis");
    for (const p of lauf.phasen) {
      const li = knoten("li");
      li.append(knoten("strong", null, `${p.titel}: `), document.createTextNode(p.kurz || "keine Daten"));
      ul.append(li);
    }
    karte.append(ul);
  }

  const zeile = knoten("div", "hb-zeile");
  const notiz = knoten("input");
  notiz.type = "text";
  notiz.placeholder = "Notiz: Was fiel auf? (Licht, Abstand, Kleidung …)";
  notiz.value = lauf.notiz || "";
  notiz.maxLength = 2000;
  const speichern = knoten("button", "btn", "Notiz speichern");
  speichern.type = "button";
  speichern.addEventListener("click", async () => {
    try {
      await senden("/tests/notiz", { name: lauf.name, text: notiz.value });
      speichern.textContent = "gespeichert";
      setTimeout(listeLaden, 800);
    } catch (e) {
      speichern.textContent = `Fehler: ${e.message}`;
    }
  });
  const zip = knoten("a", "btn", `ZIP ${groesse(lauf.bytes)}`.trim());
  zip.href = `/tests/zip?name=${encodeURIComponent(lauf.name)}`;
  zip.setAttribute("download", `${lauf.name}.zip`);
  zeile.append(notiz, speichern, zip);
  karte.append(zeile);
  return karte;
}

async function listeLaden() {
  try {
    const { laeufe } = await holen("/tests/liste");
    // Angehakte Läufe merken, damit das Neuzeichnen die Auswahl nicht löscht.
    const gewaehlt = new Set([...ui.laeufe.querySelectorAll(".ts-wahl:checked")].map((k) => k.value));
    // Nicht neu zeichnen, während jemand eine Notiz tippt.
    if (ui.laeufe.contains(document.activeElement) && document.activeElement.type === "text") return;
    ui.laeufe.replaceChildren(...laeufe.map(laufZeichnen));
    for (const k of ui.laeufe.querySelectorAll(".ts-wahl")) k.checked = gewaehlt.has(k.value);
    ui.keine.hidden = laeufe.length > 0;
    ui.hochNeu.disabled = hochladen || !laeufe.some((l) => !l.hochgeladen);
  } catch (e) {
    ui.hochText.textContent = `Liste nicht abrufbar: ${e.message}`;
  }
}

async function hoch(namen) {
  if (hochladen) return;
  hochladen = true;
  ui.hochAuswahl.disabled = ui.hochNeu.disabled = true;
  ui.hochText.className = "hb-hinweis";
  ui.hochText.textContent = "Lade hoch …";
  try {
    const r = await senden("/tests/hochladen", namen ? { namen } : {});
    if (r.ok) {
      ui.hochText.className = "hb-hinweis ts-ok";
      ui.hochText.textContent = r.hochgeladen.length
        ? `Hochgeladen: ${r.hochgeladen.length} Lauf/Läufe (Commit ${(r.commit || "").slice(0, 7)}).`
        : "Nichts Neues zum Hochladen.";
    } else {
      ui.hochText.className = "hb-hinweis ts-fehler";
      ui.hochText.textContent = `Hochladen fehlgeschlagen: ${r.fehler}`;
    }
  } catch (e) {
    ui.hochText.className = "hb-hinweis ts-fehler";
    ui.hochText.textContent = `Hochladen fehlgeschlagen: ${e.message}`;
  } finally {
    hochladen = false;
    ui.hochAuswahl.disabled = false;
    listeLaden();
  }
}

ui.hochNeu.addEventListener("click", () => hoch(null));
ui.hochAuswahl.addEventListener("click", () => {
  const namen = [...ui.laeufe.querySelectorAll(".ts-wahl:checked")].map((k) => k.value);
  if (!namen.length) {
    ui.hochText.className = "hb-hinweis ts-warn";
    ui.hochText.textContent = "Erst Läufe anhaken.";
    return;
  }
  hoch(namen);
});
ui.token.addEventListener("toggle", () => { ui.token.dataset.angefasst = "1"; });

// --- Abfrage-Schleife --------------------------------------------------------------

async function abfragen() {
  try {
    zeigeStatus(await holen("/tests/status"));
    verbunden(true);
  } catch {
    verbunden(false);
  }
  setTimeout(abfragen, aktiv ? TAKT_AKTIV_MS : TAKT_RUHE_MS);
}

async function start() {
  try {
    await katalogLaden();
  } catch (e) {
    meldung(`Testliste nicht abrufbar: ${e.message}`, true);
    setTimeout(start, TAKT_RUHE_MS);
    return;
  }
  listeLaden();
  abfragen();
}

start();
