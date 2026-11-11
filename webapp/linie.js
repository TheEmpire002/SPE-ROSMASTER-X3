/*
 * line following: the robot follows tape on the floor with the camera
 * (navigation/line_follow.py); this file only sends start/stop and the
 * junction preference and shows the state (GET /linie/status) plus the
 * bird's-eye view of the floor (GET /linie/bild) for setting up
 * send() comes from app.js (same WebSocket connection as the d-pad)
 */

(() => {

const button = document.getElementById("btn-linie");
const stateOut = document.getElementById("val-linie-zustand");
const widthOut = document.getElementById("val-linie-breite");
const distanceOut = document.getElementById("val-linie-strecke");
const text = document.getElementById("linie-text");
const picture = document.getElementById("linie-bild");
const choiceButtons = document.querySelectorAll("[data-abzweig]");

const POLL_ACTIVE_MS = 400;
const POLL_IDLE_MS = 2000;
const STATE_TEXT = {
  lernt: "lernt die Linienfarbe",
  folgt: "folgt der Linie",
  blind: "fährt auf der gemerkten Linie",
  dreht: "dreht in die Kurve",
  wartet: "wartet",
  sucht: "sucht die Linie",
  aus: "aus",
};

let active = false;
let prefer = "gerade";
let timer = null;

function showChoice() {
  choiceButtons.forEach((b) => b.classList.toggle("aktiv", b.dataset.abzweig === prefer));
}

function setButton(on) {
  active = on;
  button.textContent = on ? "Linie folgen beenden" : "Linie folgen";
  button.classList.toggle("aktiv", on);
}

choiceButtons.forEach((b) => {
  b.addEventListener("click", () => {
    prefer = b.dataset.abzweig;
    showChoice();
    // while running this only changes the preference for the next junction
    if (active) send({ type: "line_follow", abzweig: prefer });
  });
});

button.addEventListener("click", () => {
  send(active ? { type: "line_follow_stop" } : { type: "line_follow", abzweig: prefer });
  // toggle right away, the next status corrects it if the robot refuses
  setButton(!active);
  schedulePoll(300);
});

function schedulePoll(delay) {
  clearTimeout(timer);
  timer = setTimeout(poll, delay);
}

function refreshPicture() {
  // new url each time, otherwise the browser shows the cached picture
  const next = new Image();
  next.onload = () => {
    picture.src = next.src;
    picture.hidden = false;
  };
  next.src = `/linie/bild?t=${Date.now()}`;
}

async function poll() {
  try {
    const response = await fetch("/linie/status", { signal: AbortSignal.timeout(2000) });
    if (response.ok) {
      const st = await response.json();
      setButton(st.aktiv === true);
      button.disabled = st.verfuegbar === false;
      if (st.aktiv && st.abzweig && st.abzweig !== prefer) {
        prefer = st.abzweig;
        showChoice();
      }
      stateOut.textContent = st.verfuegbar === false
        ? (st.fehler || "keine Kamera")
        : (STATE_TEXT[st.zustand] || st.zustand || "—");
      widthOut.textContent = typeof st.linie_breite_cm === "number"
        ? `${st.linie_breite_cm.toFixed(1)} cm breit · ${st.kreuzungen ?? 0} Abzweige`
        : "—";
      distanceOut.textContent = typeof st.gefahren_m === "number" && (st.aktiv || st.gefahren_m > 0)
        ? `${st.gefahren_m.toFixed(1)} m`
        : "—";
      text.textContent = st.aktiv ? (st.text || "—") : (st.grund ? `Zuletzt beendet: ${st.grund}` : "—");
      if (st.aktiv || picture.hidden) refreshPicture();
    }
  } catch {
    // robot not reachable right now, try again next time
  }
  schedulePoll(active ? POLL_ACTIVE_MS : POLL_IDLE_MS);
}

showChoice();
poll();

})();
