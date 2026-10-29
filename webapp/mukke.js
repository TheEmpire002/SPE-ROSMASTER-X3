/*
 * MUKKE: YouTube-Link einfügen, der Roboter spielt den Ton auf seinem
 * Lautsprecher ab (POST /mukke/...). Abgespielt wird auf dem Roboter, nicht
 * im Browser -- die App ist nur die Fernbedienung.
 *
 * Läuft nach app.js und nutzt von dort dir und logLine. In einer Funktion,
 * damit die Namen hier nicht mit app.js zusammenstoßen.
 */

(() => {

const $ = (id) => document.getElementById(id);
const dialog = $("mukke-dialog");
const knopf = $("btn-mukke");
const url = $("mukke-url");
const titel = $("mukke-titel-jetzt");
const pause = $("mukke-pause");
const stopp = $("mukke-stop");
const laut = $("mukke-lautstaerke");
const lautOut = $("mukke-lautstaerke-out");
const meldung = $("mukke-meldung");

/** Wie oft nachgefragt wird: im offenen Fenster öfter, sonst nur für den Knopf. */
const TAKT_OFFEN_MS = 1500;
const TAKT_ZU_MS = 6000;

let zuletztAbgefragt = 0;
let ziehtRegler = false;

function zeigeMeldung(text, fehler = false) {
  meldung.textContent = text;
  meldung.classList.toggle("fehler", fehler);
  meldung.hidden = false;
  clearTimeout(zeigeMeldung.timer);
  zeigeMeldung.timer = setTimeout(() => { meldung.hidden = true; }, fehler ? 10000 : 3000);
}

async function senden(pfad, daten = {}) {
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
  zeige(json);
  return json;
}

function zeige(s) {
  if (!s) return;
  const spielt = s.laeuft === true;
  knopf.classList.toggle("spielt", spielt);
  knopf.title = spielt ? `Spielt: ${s.titel || "lädt …"}` : "Musik auf dem Lautsprecher";

  if (spielt) {
    titel.textContent = (s.pausiert ? "⏸ " : "♪ ") + (s.titel || "lädt …");
  } else if (s.fehler) {
    titel.textContent = "nichts";
    // Nur einmal melden, nicht bei jeder Abfrage erneut.
    if (s.fehler !== zeige.letzterFehler) zeigeMeldung(s.fehler, true);
  } else {
    titel.textContent = s.verfuegbar === false ? (s.hinweis || "nicht verfügbar") : "nichts";
  }
  zeige.letzterFehler = s.fehler;

  pause.disabled = stopp.disabled = !spielt;
  pause.textContent = s.pausiert ? "Weiter" : "Pause";
  if (!ziehtRegler && typeof s.lautstaerke === "number") {
    laut.value = s.lautstaerke;
    lautOut.textContent = `${s.lautstaerke} %`;
  }
}

async function abfragen() {
  zuletztAbgefragt = Date.now();
  try {
    const antwort = await fetch("/mukke/status", { cache: "no-store" });
    if (antwort.ok) zeige(await antwort.json());
  } catch {
    // Roboter kurz weg -- beim nächsten Takt wieder.
  }
}

// Ein Zeitgeber, der je nach Fenster öfter oder seltener fragt.
setInterval(() => {
  const takt = dialog.open ? TAKT_OFFEN_MS : TAKT_ZU_MS;
  if (Date.now() - zuletztAbgefragt >= takt) abfragen();
}, 500);

knopf.addEventListener("click", () => {
  // Handeingaben loslassen, wie beim Einstellungsfenster.
  dir.x = dir.y = dir.rot = 0;
  dialog.showModal();
  abfragen();
  url.focus();
});

$("mukke-schliessen").addEventListener("click", () => dialog.close());
dialog.addEventListener("click", (e) => {
  if (e.target === dialog) dialog.close();
});

$("mukke-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const link = url.value.trim();
  if (!link) return;
  titel.textContent = "♪ lädt …";
  if (await senden("/mukke/play", { url: link })) {
    zeigeMeldung("Spielt gleich ab -- YouTube braucht ein paar Sekunden");
    logLine("MUKKE: spielt ab");
    url.value = "";
  }
});

pause.addEventListener("click", () => senden("/mukke/pause"));
stopp.addEventListener("click", () => senden("/mukke/stop"));

laut.addEventListener("input", () => {
  ziehtRegler = true;
  lautOut.textContent = `${laut.value} %`;
});
laut.addEventListener("change", async () => {
  await senden("/mukke/volume", { lautstaerke: Number(laut.value) });
  ziehtRegler = false;
});

abfragen();

})();
