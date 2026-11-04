/*
 * Genau fahren: "1,5 m vor", "90° links drehen". Der Roboter misst mit dem
 * Lidar, wie weit er wirklich ist (navigation/manoever.py); hier werden nur
 * die Befehle geschickt und der Fortschritt angezeigt (GET /manoever/status).
 * send() kommt aus app.js (dieselbe WebSocket-Verbindung wie das Steuerkreuz).
 */

(() => {

const meter = document.getElementById("genau-meter");
const grad = document.getElementById("genau-grad");
const text = document.getElementById("genau-text");
/** Abfragetakt: schnell, solange ein Befehl läuft, sonst gemächlich. */
const TAKT_AKTIV_MS = 400;
const TAKT_RUHE_MS = 2000;

document.querySelectorAll("[data-richtung]").forEach((knopf) => {
  knopf.addEventListener("click", () => {
    send({ type: "move", richtung: knopf.dataset.richtung, meter: Number(meter.value) });
    nachfragen(300);
  });
});

document.querySelectorAll("[data-drehen]").forEach((knopf) => {
  knopf.addEventListener("click", () => {
    send({ type: "turn", grad: Number(knopf.dataset.drehen) * Number(grad.value) });
    nachfragen(300);
  });
});

let timer = null;

function nachfragen(nach) {
  clearTimeout(timer);
  timer = setTimeout(abfragen, nach);
}

async function abfragen() {
  let aktiv = false;
  try {
    const antwort = await fetch("/manoever/status", { signal: AbortSignal.timeout(2000) });
    if (antwort.ok) {
      const z = await antwort.json();
      aktiv = z.aktiv;
      if (!z.verfuegbar) {
        text.textContent = "Geht nur mit Lidar.";
      } else if (z.aktiv || z.grund) {
        text.textContent = z.aktiv ? z.text : `Zuletzt: ${z.grund}`;
      }
    }
  } catch {
    // Roboter gerade nicht erreichbar -- beim nächsten Mal wieder.
  }
  nachfragen(aktiv ? TAKT_AKTIV_MS : TAKT_RUHE_MS);
}

abfragen();

})();
