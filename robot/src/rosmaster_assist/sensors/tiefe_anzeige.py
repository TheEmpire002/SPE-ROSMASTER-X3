"""Das Tiefenbild zum Ansehen: Farben nach Entfernung, Hindernisse markiert.

Für ``/tiefe/stream`` und die Kamera-Ansicht der App. Damit lässt sich am
Roboter prüfen, was die Tiefenkamera misst und was davon als Hindernis
zählt -- ohne Zahlen aus ``/tiefe/status`` deuten zu müssen.

* rot = nah, blau = fern (``TIEFE_ANZEIGE_NAH_M`` bis ``…_FERN_M``)
* schwarz = keine Messung: näher als ~0,6 m, zu weit, spiegelnd oder schwarz
* magenta = zählt als Hindernis (4–40 cm über dem Boden, im Bereich)

Überlagert wird über das Farbbild derselben Orbbec. Die beiden Kameras
haben verschiedene Bildwinkel und sitzen etwas auseinander. Die Größe kommt
aus den Bildwinkeln, der Rest ist von Hand einstellbar
(``TIEFE_UEBER_VERSATZ_PX``, ``TIEFE_UEBER_ZOOM``). Es deckt sich also nur
ungefähr -- zum Ansehen reicht das, gerechnet wird damit nichts.
"""

from __future__ import annotations

import math

from .. import config

_MAGENTA = (255, 0, 255)  # BGR


def _text(cv2, bild, text: str, x: int, y: int, groesse: float = 0.5) -> None:
    """Weiß mit schwarzem Rand -- lesbar auf jedem Hintergrund."""
    cv2.putText(bild, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, groesse, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(bild, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, groesse, (255, 255, 255), 1, cv2.LINE_AA)


def _m(wert: float) -> str:
    return f"{wert:.2f} m".replace(".", ",")


def zeichnen(bild, hfov_tiefe: float, hintergrund: bytes | None = None,
             hfov_farbe_grad: float | None = None) -> bytes:
    """Ausgewertetes Tiefenbild (``TiefenBild`` mit ``roh``) -> JPEG.

    Mit ``hintergrund`` (JPEG des Farbbilds) halbdurchsichtig darüber,
    sonst das Tiefenbild allein in doppelter Größe.
    """
    import cv2
    import numpy as np

    roh = bild.roh
    h, w = roh.shape
    meter = roh.astype(np.float32) / 1000.0
    gueltig = roh > 0

    nah, fern = config.TIEFE_ANZEIGE_NAH_M, config.TIEFE_ANZEIGE_FERN_M
    anteil = np.clip((meter - nah) / max(0.1, fern - nah), 0.0, 1.0)
    # JET: 0 = blau, 255 = rot -- also umdrehen, damit nah rot ist.
    farben = cv2.applyColorMap(((1.0 - anteil) * 255).astype(np.uint8), cv2.COLORMAP_JET)
    farben[~gueltig] = 0
    hindernis = np.zeros((h, w), dtype=bool)
    if bild.maske is not None:
        # Die Maske hat halbe Auflösung (jeder zweite Punkt ausgewertet).
        voll = np.repeat(np.repeat(bild.maske, 2, axis=0), 2, axis=1)[:h, :w]
        hindernis[: voll.shape[0], : voll.shape[1]] = voll
        farben[hindernis] = _MAGENTA

    farbbild = None
    if hintergrund is not None:
        farbbild = cv2.imdecode(np.frombuffer(hintergrund, dtype=np.uint8), cv2.IMREAD_COLOR)

    if farbbild is None:
        ausgabe = cv2.resize(farben, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST)
        lage = (0, 0, w * 2, h * 2)  # wo das Tiefenbild in der Ausgabe liegt
    else:
        ausgabe = farbbild
        H, W = ausgabe.shape[:2]
        # Breite des Tiefenbilds im Farbbild aus dem Verhältnis der Bildwinkel.
        verhaeltnis = 1.0
        if hfov_tiefe and hfov_farbe_grad:
            verhaeltnis = math.tan(hfov_tiefe / 2.0) / math.tan(math.radians(hfov_farbe_grad) / 2.0)
        bw = max(1, int(W * verhaeltnis * config.TIEFE_UEBER_ZOOM))
        bh = max(1, int(bw * h / w))
        dx, dy = config.TIEFE_UEBER_VERSATZ_PX
        x0, y0 = (W - bw) // 2 + int(dx), (H - bh) // 2 + int(dy)
        lage = (x0, y0, bw, bh)
        gross = cv2.resize(farben, (bw, bh), interpolation=cv2.INTER_NEAREST)
        g_gueltig = cv2.resize(gueltig.astype(np.uint8), (bw, bh), interpolation=cv2.INTER_NEAREST) > 0
        g_hind = cv2.resize(hindernis.astype(np.uint8), (bw, bh), interpolation=cv2.INTER_NEAREST) > 0
        # Nur den Teil, der ins Farbbild fällt.
        ax, ay = max(0, x0), max(0, y0)
        ex, ey = min(W, x0 + bw), min(H, y0 + bh)
        if ex > ax and ey > ay:
            ziel = ausgabe[ay:ey, ax:ex].astype(np.float32)
            quelle = gross[ay - y0:ey - y0, ax - x0:ex - x0].astype(np.float32)
            deck = np.where(g_hind[ay - y0:ey - y0, ax - x0:ex - x0], 0.8,
                            np.where(g_gueltig[ay - y0:ey - y0, ax - x0:ex - x0],
                                     config.TIEFE_UEBER_DECKKRAFT, 0.0))[..., None]
            ausgabe[ay:ey, ax:ex] = (ziel * (1.0 - deck) + quelle * deck).astype(np.uint8)
        cv2.rectangle(ausgabe, (x0, y0), (x0 + bw - 1, y0 + bh - 1), (200, 200, 200), 1)

    # Entfernung in der Bildmitte: Median eines kleinen Fensters, damit ein
    # einzelner Ausreißer die Zahl nicht springen lässt.
    fenster = meter[h // 2 - 4:h // 2 + 5, w // 2 - 4:w // 2 + 5]
    werte = fenster[fenster > 0]
    lx, ly, lb, lh = lage
    mx, my = lx + lb // 2, ly + lh // 2
    cv2.drawMarker(ausgabe, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
    _text(cv2, ausgabe, _m(float(np.median(werte))) if len(werte) else "keine Messung",
          mx + 12, my - 8)

    zeilen = []
    if len(bild.hindernisse):
        abstand = np.hypot(bild.hindernisse[:, 0], bild.hindernisse[:, 1])
        zeilen.append(f"naechstes Hindernis: {_m(float(abstand.min()))}")
    else:
        zeilen.append("kein Hindernis")
    zeilen.append("Boden erkannt" if bild.boden is not None else "Boden NICHT erkannt")
    for i, zeile in enumerate(zeilen):
        _text(cv2, ausgabe, zeile, 8, 20 + 20 * i)

    # Legende unten: Farbverlauf nah -> fern, dazu was Schwarz und Magenta heißt.
    H, W = ausgabe.shape[:2]
    breite = min(200, W // 3)
    verlauf = cv2.applyColorMap(np.linspace(255, 0, breite).astype(np.uint8)[None, :], cv2.COLORMAP_JET)
    ausgabe[H - 22:H - 12, 8:8 + breite] = np.repeat(verlauf, 10, axis=0)
    _text(cv2, ausgabe, _m(nah), 8, H - 28, 0.4)
    _text(cv2, ausgabe, _m(fern), 8 + breite - 45, H - 28, 0.4)
    _text(cv2, ausgabe, "schwarz: keine Messung (<0,6 m)  magenta: Hindernis",
          16 + breite, H - 12, 0.4)

    ok, jpeg = cv2.imencode(".jpg", ausgabe, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok:
        raise RuntimeError("JPEG kodieren fehlgeschlagen")
    return jpeg.tobytes()
