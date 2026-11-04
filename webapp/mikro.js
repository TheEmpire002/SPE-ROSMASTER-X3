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
/** Der Roboter meldet sich mindestens jede Sekunde (Ton oder Lebenszeichen).
 *  Kommt so lange gar nichts, hängt die Verbindung: neu verbinden. */
const STILLE_MS = 5000;
/** Länger als das wartet das Mitlesen nicht auf eine Antwort. */
const ABFRAGE_FRIST_MS = 2000;

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
    // Mit Frist: Hängt der Roboter, bliebe fetch sonst ewig offen und das
    // Mitlesen stünde, auch wenn er längst wieder da ist.
    const antwort = await fetch("/sprache/live", { signal: AbortSignal.timeout(ABFRAGE_FRIST_MS) });
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
let zuletzt = 0;  // wann zuletzt etwas vom Roboter kam (performance.now)
let neuTimer = null;

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
  zuletzt = performance.now();
  status(hoerenStatus, "verbinde …", false);
  ws.onmessage = (ev) => {
    zuletzt = performance.now();
    // Der Browser hält den Ton gelegentlich an (Tab im Hintergrund,
    // Bildschirmsperre); dann still wieder anwerfen.
    if (ctx.state === "suspended") ctx.resume();
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
    neuVerbinden("getrennt");
  };
}

/** Alte Verbindung fallen lassen, ohne auf ihr Ende zu warten. */
function trennen() {
  if (!ws) return;
  const alt = ws;
  ws = null;
  alt.onclose = null;
  alt.onmessage = null;
  alt.close();
}

function neuVerbinden(grund) {
  if (!hoeren.checked) return;
  // Server neu gestartet, Pi hing oder WLAN weg: still wieder versuchen,
  // so lange, bis er wieder da ist.
  status(hoerenStatus, `${grund} -- verbinde neu …`, false);
  clearTimeout(neuTimer);
  neuTimer = setTimeout(() => { if (hoeren.checked && !ws) mithoerenAn(); }, 2000);
}

// Eine hängende Verbindung meldet sich nicht von selbst (kein onclose, wenn
// der Pi stehen bleibt). Daher selbst auf Lebenszeichen achten.
setInterval(() => {
  if (ws && performance.now() - zuletzt > STILLE_MS) {
    trennen();
    neuVerbinden("keine Antwort");
  }
}, 1000);

function mithoerenAus() {
  clearTimeout(neuTimer);
  trennen();
  if (ctx) ctx.suspend();
  status(hoerenStatus, "aus", false);
}

hoeren.addEventListener("change", () => {
  if (hoeren.checked) mithoerenAn(); else mithoerenAus();
});

// --- Durchsage: über den Roboter sprechen --------------------------------------
//
// Eigenes Mikrofon (Laptop/Handy) -> WebSocket /sprache/durchsage -> Lautsprecher
// des Roboters. Erst {"rate": Hz}, dann Rohton 16 bit mono.

const sprechen = $("mikro-sprechen");
const sprechenStatus = $("mikro-sprechen-state");
const hinweis = $("mikro-sprechen-hinweis");
/** Für Sprache reicht das; so bleibt es im WLAN klein (32 kB/s). */
const SENDE_RATE = 16000;
/** Port der HTTPS-Variante (config.HTTPS_PORT). */
const HTTPS_PORT = 8443;

let dsWs = null;
let dsCtx = null;
let dsStrom = null;
let dsTimer = null;

function hinweisZeigen(knoten) {
  hinweis.replaceChildren(...(knoten ? [knoten].flat() : []));
  hinweis.hidden = !knoten;
}

/** Mikrofon-Ton auf SENDE_RATE bringen und als 16 bit verpacken. */
function verpacken(ein, schritt) {
  const n = Math.floor(ein.length / schritt);
  const aus = new Int16Array(n);
  for (let i = 0; i < n; i++) {
    // Mittelwert über das Fenster: ein einfacher Tiefpass, sonst klirrt es.
    const von = Math.floor(i * schritt);
    const bis = Math.min(ein.length, Math.floor((i + 1) * schritt));
    let summe = 0;
    for (let j = von; j < bis; j++) summe += ein[j];
    const wert = Math.max(-1, Math.min(1, summe / Math.max(1, bis - von)));
    aus[i] = wert * 32767;
  }
  return aus;
}

async function sprechenAn() {
  // Browser geben das Mikrofon nur auf sicheren Seiten frei (HTTPS oder
  // localhost). Unter http://10.42.0.62:8000 gibt es getUserMedia gar nicht.
  if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) {
    const ziel = `https://${location.hostname}:${HTTPS_PORT}${location.pathname}`;
    const link = document.createElement("a");
    link.href = ziel;
    link.textContent = ziel;
    hinweisZeigen(["Das Mikrofon gibt der Browser nur über HTTPS frei: ", link,
                   " öffnen und die Zertifikatswarnung einmal bestätigen."]);
    sprechen.checked = false;
    status(sprechenStatus, "nur über HTTPS", false);
    return;
  }
  hinweisZeigen(null);
  status(sprechenStatus, "Mikrofon …", false);
  try {
    dsStrom = await navigator.mediaDevices.getUserMedia({
      audio: { channelCount: 1, echoCancellation: true, noiseSuppression: true, autoGainControl: true },
    });
  } catch (fehler) {
    sprechen.checked = false;
    status(sprechenStatus, "kein Zugriff", false);
    hinweisZeigen(`Mikrofon nicht freigegeben (${fehler.name}). In den Seiteneinstellungen des Browsers erlauben.`);
    return;
  }
  if (!sprechen.checked) { sprechenAus(); return; }  // inzwischen wieder aus

  dsCtx = new AudioContext();
  const quelle = dsCtx.createMediaStreamSource(dsStrom);
  // ScriptProcessor ist veraltet, läuft aber in allen Browsern ohne eigene
  // Worklet-Datei. 2048 Werte sind bei 48 kHz gut 40 ms.
  const knoten = dsCtx.createScriptProcessor(2048, 1, 1);
  const schritt = dsCtx.sampleRate / SENDE_RATE;
  knoten.onaudioprocess = (ev) => {
    if (!dsWs || dsWs.readyState !== WebSocket.OPEN) return;
    // Staut es sich im WLAN, lieber Ton weglassen als immer später ankommen.
    if (dsWs.bufferedAmount > SENDE_RATE) return;
    dsWs.send(verpacken(ev.inputBuffer.getChannelData(0), schritt).buffer);
  };
  quelle.connect(knoten);
  // Der Knoten rechnet nur, wenn er am Ausgang hängt -- stumm, sonst hörte
  // man sich selbst aus dem eigenen Lautsprecher.
  const stumm = dsCtx.createGain();
  stumm.gain.value = 0;
  knoten.connect(stumm);
  stumm.connect(dsCtx.destination);
  dsVerbinden();
}

function dsVerbinden() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  dsWs = new WebSocket(`${proto}://${location.host}/sprache/durchsage`);
  dsWs.binaryType = "arraybuffer";
  status(sprechenStatus, "verbinde …", false);
  dsWs.onopen = () => dsWs.send(JSON.stringify({ rate: SENDE_RATE }));
  dsWs.onmessage = (ev) => {
    if (typeof ev.data !== "string") return;
    const msg = JSON.parse(ev.data);
    if (msg.bereit) {
      status(sprechenStatus, "an -- sprich", true);
    } else {
      status(sprechenStatus, msg.grund || "geht nicht", false);
    }
  };
  dsWs.onclose = () => {
    dsWs = null;
    if (!sprechen.checked) return;
    // Server neu gestartet oder WLAN weg: wieder verbinden, das Mikrofon bleibt offen.
    status(sprechenStatus, "getrennt -- verbinde neu …", false);
    clearTimeout(dsTimer);
    dsTimer = setTimeout(() => { if (sprechen.checked && !dsWs) dsVerbinden(); }, 2000);
  };
}

function sprechenAus() {
  clearTimeout(dsTimer);
  if (dsWs) {
    const alt = dsWs;
    dsWs = null;
    alt.onclose = null;
    alt.close();
  }
  // Mikrofon wirklich freigeben, sonst zeigt der Browser weiter "Aufnahme".
  if (dsStrom) dsStrom.getTracks().forEach((t) => t.stop());
  dsStrom = null;
  if (dsCtx) dsCtx.close();
  dsCtx = null;
  status(sprechenStatus, "aus", false);
}

sprechen.addEventListener("change", () => {
  if (sprechen.checked) sprechenAn(); else sprechenAus();
});

})();
