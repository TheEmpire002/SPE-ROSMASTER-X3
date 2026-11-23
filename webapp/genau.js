/*
 * Precise driving: "1.5 m forward", "turn 90° left". The robot measures with
 * the lidar how far it has really gone (navigation/maneuver.py); this file
 * only sends the commands and shows the progress (GET /manoever/status).
 * send() comes from app.js (same WebSocket connection as the d-pad).
 */

(() => {

const metersInput = document.getElementById("genau-meter");
const degreesInput = document.getElementById("genau-grad");
const statusText = document.getElementById("genau-text");
/** Polling interval: fast while a command is running, otherwise leisurely. */
const POLL_ACTIVE_MS = 400;
const POLL_IDLE_MS = 2000;

document.querySelectorAll("[data-richtung]").forEach((button) => {
  button.addEventListener("click", () => {
    send({ type: "move", richtung: button.dataset.richtung, meter: Number(metersInput.value) });
    schedulePoll(300);
  });
});

document.querySelectorAll("[data-drehen]").forEach((button) => {
  button.addEventListener("click", () => {
    send({ type: "turn", grad: Number(button.dataset.drehen) * Number(degreesInput.value) });
    schedulePoll(300);
  });
});

let timer = null;

function schedulePoll(delay) {
  clearTimeout(timer);
  timer = setTimeout(poll, delay);
}

async function poll() {
  let active = false;
  try {
    const response = await fetch("/manoever/status", { signal: AbortSignal.timeout(2000) });
    if (response.ok) {
      const st = await response.json();
      active = st.aktiv;
      if (!st.verfuegbar) {
        statusText.textContent = "Geht nur mit Lidar.";
      } else if (st.aktiv || st.grund) {
        statusText.textContent = st.aktiv ? st.text : `Zuletzt: ${st.grund}`;
      }
    }
  } catch {
    // Robot not reachable right now -- try again next time.
  }
  schedulePoll(active ? POLL_ACTIVE_MS : POLL_IDLE_MS);
}

poll();

})();
