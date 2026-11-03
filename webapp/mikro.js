/*
 * Mikrofon live: mitlesen (was die Spracherkennung versteht) und mithören
 * (den Ton des Roboter-Mikrofons im Browser). Beides einzeln schaltbar und
 * aus, solange niemand es einschaltet -- dann kostet es am Roboter nichts.
 *
 * Mitlesen fragt GET /sprache/live ab, Mithören hängt am WebSocket
 * /sprache/ton (erst {"rate": Hz}, dann Rohton 16 bit mono als Binärdaten).
 * In einer Funktion, damit die Namen nicht mit app.js zusammenstoßen.
 */

(() => {

const $ = (id) => document.getElementById(id);
const lesen = $("mikro-lesen");
const lesenStatus = $("mikro-lesen-state");
const hoeren = $("mikro-hoeren");
const hoerenStatus = $("mikro-hoeren-state");
const live = $("mikro-live");
const balken = $("mikro-pegel-balken");
const teil = $("mikro-teil");
const liste = $("mikro-ereignisse");

/** Abfragetakt beim Mitlesen. Der Roboter liefert den Pegel 4x pro Sekunde. */
const TAKT_MS = 250;
/** Mithören: so weit im Voraus planen und höchstens so weit hinterherhängen. */
const VORLAUF_S = 0.15;
const MAX_RUECKSTAND_S = 1.0;

function status(el, text, an) {
  el.textContent = text;
  el.classList.toggle("an", an);
}

// --- Mitlesen ----------------------------------------------------------------

let lesenTimer = null;

const URTEIL = {
  befehl: (e) => `Befehl: ${e.befehl}`,
  verworfen: (e) => `verworfen (${e.grund || e.befehl})`,
  weckwort: () => "Weckwort",
  ohne: () => "kein Befehl",
};

function zeigeLive(d) {
  if (!d.verfuegbar) {
    status(lesenStatus, "kein Mikrofon", false);
    teil.textContent = "Spracherkennung: " + (d.zustand || "aus");
    balken.style.width = "0";
  } else {
    status(lesenStatus, d.taub ? "spricht selbst" : (d.geweckt ? "hört zu …" : "an"), true);
    balken.style.width = `${Math.round(d.pegel * 100)}%`;
    teil.textContent = d.taub ? "(hört weg, solange er spricht)" : (d.teil ? `„${d.teil} …“` : "—");
  }
  live.classList.toggle("taub", !!d.taub);

  liste.replaceChildren(...d.ereignisse.slice().reverse().map((e) => {
    const li = document.createElement("li");
    li.className = e.ergebnis;
    const zeit = new Date(e.zeit * 1000).toLocaleTimeString("de-DE");
    const text = document.createElement("span");
    text.textContent = `${zeit}  „${e.text}“`;
    const urteil = document.createElement("span");
    urteil.className = "urteil";
    urteil.textContent = (URTEIL[e.ergebnis] || URTEIL.ohne)(e);
    li.append(text, urteil);
    return li;
  }));
}

async function abfragen() {
  try {
    const antwort = await fetch("/sprache/live");
    if (antwort.ok) zeigeLive(await antwort.json());
  } catch {
    status(lesenStatus, "Roboter nicht erreichbar", false);
  }
  if (lesen.checked) lesenTimer = setTimeout(abfragen, TAKT_MS);
}

lesen.addEventListener("change", () => {
  clearTimeout(lesenTimer);
  live.hidden = !lesen.checked;
  if (lesen.checked) {
    abfragen();
  } else {
    status(lesenStatus, "aus", false);
  }
});

// --- Mithören ----------------------------------------------------------------

let ws = null;
let ctx = null;
let rate = 16000;
let naechster = 0;

function abspielen(puffer) {
  const werte = new Int16Array(puffer);
  if (!werte.length) return;
  const ton = ctx.createBuffer(1, werte.length, rate);
  const kanal = ton.getChannelData(0);
  for (let i = 0; i < werte.length; i++) kanal[i] = werte[i] / 32768;

  const jetzt = ctx.currentTime;
  // Hinkt die Wiedergabe hinterher (WLAN-Stau), lieber springen als
  // immer später hören.
  if (naechster < jetzt + 0.02 || naechster > jetzt + MAX_RUECKSTAND_S) {
    naechster = jetzt + VORLAUF_S;
  }
  const quelle = ctx.createBufferSource();
  quelle.buffer = ton;
  quelle.connect(ctx.destination);
  quelle.start(naechster);
  naechster += ton.duration;
}

function mithoerenAn() {
  // Der AudioContext muss aus einem Klick heraus entstehen, sonst bleibt er
  // im Browser stumm -- deshalb hier und nicht beim Laden der Seite.
  ctx = ctx || new AudioContext();
  ctx.resume();
  naechster = 0;
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/sprache/ton`);
  ws.binaryType = "arraybuffer";
  status(hoerenStatus, "verbinde …", false);
  ws.onmessage = (ev) => {
    if (typeof ev.data === "string") {
      const msg = JSON.parse(ev.data);
      if (msg.rate) {
        rate = msg.rate;
        status(hoerenStatus, `an (${rate / 1000} kHz)`, true);
      } else {
        status(hoerenStatus, "kein Mikrofon", false);
      }
      return;
    }
    abspielen(ev.data);
  };
  ws.onclose = () => {
    ws = null;
    if (hoeren.checked) {
      // Server neu gestartet oder WLAN weg: still wieder versuchen.
      status(hoerenStatus, "getrennt -- verbinde neu …", false);
      setTimeout(() => { if (hoeren.checked && !ws) mithoerenAn(); }, 2000);
    }
  };
}

function mithoerenAus() {
  if (ws) {
    const alt = ws;
    ws = null;
    alt.onclose = null;
    alt.close();
  }
  if (ctx) ctx.suspend();
  status(hoerenStatus, "aus", false);
}

hoeren.addEventListener("change", () => {
  if (hoeren.checked) mithoerenAn(); else mithoerenAus();
});

})();
