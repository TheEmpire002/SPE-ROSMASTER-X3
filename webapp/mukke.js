/*
 * MUKKE: paste a YouTube link, the robot plays the audio on its speaker
 * (POST /mukke/...). Playback happens on the robot, not in the browser --
 * the app is only the remote control.
 *
 * Runs after app.js and uses dir and logLine from there. Wrapped in a
 * function so the names here do not collide with app.js.
 */

(() => {

const $ = (id) => document.getElementById(id);
const dialog = $("mukke-dialog");
const button = $("btn-mukke");
const url = $("mukke-url");
const nowPlaying = $("mukke-titel-jetzt");
const pause = $("mukke-pause");
const stop = $("mukke-stop");
const volume = $("mukke-lautstaerke");
const volumeOut = $("mukke-lautstaerke-out");
const message = $("mukke-meldung");

/** How often to poll: more often while the dialog is open, otherwise only for the button. */
const POLL_OPEN_MS = 1500;
const POLL_CLOSED_MS = 6000;

let lastPolled = 0;
let draggingSlider = false;

function showMessage(text, isError = false) {
  message.textContent = text;
  message.classList.toggle("fehler", isError);
  message.hidden = false;
  clearTimeout(showMessage.timer);
  showMessage.timer = setTimeout(() => { message.hidden = true; }, isError ? 10000 : 3000);
}

async function post(path, data = {}) {
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
  render(json);
  return json;
}

function render(s) {
  if (!s) return;
  const playing = s.laeuft === true;
  button.classList.toggle("spielt", playing);
  button.title = playing ? `Spielt: ${s.titel || "lädt …"}` : "Musik auf dem Lautsprecher";

  if (playing) {
    nowPlaying.textContent = (s.pausiert ? "⏸ " : "♪ ") + (s.titel || "lädt …");
  } else if (s.fehler) {
    nowPlaying.textContent = "nichts";
    // Report only once, not again on every poll.
    if (s.fehler !== render.lastError) showMessage(s.fehler, true);
  } else {
    nowPlaying.textContent = s.verfuegbar === false ? (s.hinweis || "nicht verfügbar") : "nichts";
  }
  render.lastError = s.fehler;

  pause.disabled = stop.disabled = !playing;
  pause.textContent = s.pausiert ? "Weiter" : "Pause";
  if (!draggingSlider && typeof s.lautstaerke === "number") {
    volume.value = s.lautstaerke;
    volumeOut.textContent = `${s.lautstaerke} %`;
  }
}

async function poll() {
  lastPolled = Date.now();
  try {
    const response = await fetch("/mukke/status", { cache: "no-store" });
    if (response.ok) render(await response.json());
  } catch {
    // Robot briefly gone -- try again on the next tick.
  }
}

// One timer that polls more or less often depending on the dialog.
setInterval(() => {
  const interval = dialog.open ? POLL_OPEN_MS : POLL_CLOSED_MS;
  if (Date.now() - lastPolled >= interval) poll();
}, 500);

button.addEventListener("click", () => {
  // Release manual inputs, like the settings dialog does.
  dir.x = dir.y = dir.rot = 0;
  dialog.showModal();
  poll();
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
  nowPlaying.textContent = "♪ lädt …";
  if (await post("/mukke/play", { url: link })) {
    showMessage("Spielt gleich ab -- YouTube braucht ein paar Sekunden");
    logLine("MUKKE: spielt ab");
    url.value = "";
  }
});

pause.addEventListener("click", () => post("/mukke/pause"));
stop.addEventListener("click", () => post("/mukke/stop"));

volume.addEventListener("input", () => {
  draggingSlider = true;
  volumeOut.textContent = `${volume.value} %`;
});
volume.addEventListener("change", async () => {
  await post("/mukke/volume", { lautstaerke: Number(volume.value) });
  draggingSlider = false;
});

poll();

})();
