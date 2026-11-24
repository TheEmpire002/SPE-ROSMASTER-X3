// shared map drawing for the web pages: occupancy shading and world <-> canvas
//
// same look as the /map page (templates/map.html): x to the right, y up,
// unknown dark grey, free light, obstacles black, so a map looks the same on
// every page

"use strict";

// occupancy grid (GET /api/mapping/map) as an image, one pixel per cell
// shading identical to updateMap() in templates/map.html
function occupancyCanvas(payload) {
  const info = (payload && payload.info) || {};
  const width = Number(info.width || 0);
  const height = Number(info.height || 0);
  if (!width || !height || !payload.data_b64) return null;
  let binary;
  try {
    binary = atob(payload.data_b64);
  } catch {
    return null;
  }
  if (binary.length !== width * height) return null;
  const c = document.createElement("canvas");
  c.width = width;
  c.height = height;
  const ctx = c.getContext("2d");
  const image = ctx.createImageData(width, height);
  const bytes = image.data;
  for (let y = 0; y < height; y++) {
    // map row 0 is the smallest y, image row 0 the top
    const sourceRow = (height - 1 - y) * width;
    for (let x = 0; x < width; x++) {
      const raw = binary.charCodeAt(sourceRow + x);
      const occupancy = raw > 127 ? raw - 256 : raw;
      let shade;
      if (occupancy < 0) shade = 105; // unknown
      else if (occupancy >= 65) shade = 12; // obstacle
      else if (occupancy <= 19) shade = 242; // free space
      else shade = 155; // uncertain
      const i = (y * width + x) * 4;
      bytes[i] = bytes[i + 1] = bytes[i + 2] = shade;
      bytes[i + 3] = 255;
    }
  }
  ctx.putImageData(image, 0, 0);
  const origin = (info.origin && info.origin.position) || { x: 0, y: 0 };
  return {
    image: c,
    width,
    height,
    resolution: Number(info.resolution),
    x0: Number(origin.x),
    y0: Number(origin.y),
  };
}

// a view on a canvas: centre (world metres) and scale (css px per metre);
// width and height are the drawing size in css px (the context may be scaled
// for sharp lines on high dpi screens)
class MapView {
  constructor(width, height) {
    this.width = width;
    this.height = height;
    this.mx = 0;
    this.my = 0;
    this.scale = 50;
  }

  // fit the world box [x0, y0, x1, y1] into the canvas with a margin
  fit(x0, y0, x1, y1, margin = 0.9) {
    const w = Math.max(x1 - x0, 0.5);
    const h = Math.max(y1 - y0, 0.5);
    this.mx = (x0 + x1) / 2;
    this.my = (y0 + y1) / 2;
    this.scale = Math.min(this.width / w, this.height / h) * margin;
  }

  // the whole occupancy map, like fitMap() on /map
  fitMap(map) {
    this.fit(map.x0, map.y0, map.x0 + map.width * map.resolution, map.y0 + map.height * map.resolution);
  }

  toCanvas(x, y) {
    return [this.width / 2 + (x - this.mx) * this.scale, this.height / 2 - (y - this.my) * this.scale];
  }

  toWorld(cx, cy) {
    return [this.mx + (cx - this.width / 2) / this.scale, this.my - (cy - this.height / 2) / this.scale];
  }

  meters(m) {
    return m * this.scale;
  }

  drawMap(ctx, map) {
    const [left, top] = this.toCanvas(map.x0, map.y0 + map.height * map.resolution);
    const size = map.resolution * this.scale;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(map.image, left, top, map.width * size, map.height * size);
  }

  // one metre grid over the visible area
  drawGrid(ctx, color = "#2a2e37") {
    const [x0, y1] = this.toWorld(0, 0);
    const [x1, y0] = this.toWorld(this.width, this.height);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    for (let m = Math.ceil(x0); m <= x1; m++) {
      const [px] = this.toCanvas(m, 0);
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, this.height); ctx.stroke();
    }
    for (let m = Math.ceil(y0); m <= y1; m++) {
      const [, py] = this.toCanvas(0, m);
      ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(this.width, py); ctx.stroke();
    }
  }

  drawLine(ctx, points, color, width, dash) {
    if (!points || points.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineJoin = "round";
    if (dash) ctx.setLineDash(dash);
    ctx.beginPath();
    points.forEach(([x, y], i) => {
      const [cx, cy] = this.toCanvas(x, y);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
    ctx.restore();
  }

  drawDot(ctx, x, y, color, radius) {
    const [cx, cy] = this.toCanvas(x, y);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fill();
  }

  // robot to scale (radius ~0.17 m) with a heading line, like on /map
  drawRobot(ctx, pose, color) {
    const [cx, cy] = this.toCanvas(pose.x, pose.y);
    const r = Math.max(7, this.meters(0.17));
    const w = (pose.winkel * Math.PI) / 180;
    ctx.fillStyle = color + "55";
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(w) * r * 1.6, cy - Math.sin(w) * r * 1.6);
    ctx.stroke();
  }
}
