import { readFileSync, readdirSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const projectRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const visualizerDirectory = resolve(projectRoot, "visualizer");
const fieldSize = 144;

function curvePoint(points, t) {
  let layer = points.map(({ x, y }) => ({ x, y }));
  while (layer.length > 1) {
    layer = layer.slice(0, -1).map((point, index) => ({
      x: point.x + (layer[index + 1].x - point.x) * t,
      y: point.y + (layer[index + 1].y - point.y) * t
    }));
  }
  return layer[0];
}

function shortestHeading(start, end, t) {
  const delta = ((end - start + 540) % 360) - 180;
  return start + delta * t;
}

function robotCorners(x, y, headingDegrees, width, height) {
  const heading = headingDegrees * Math.PI / 180;
  const cos = Math.cos(heading);
  const sin = Math.sin(heading);
  const halfWidth = width / 2;
  const halfHeight = height / 2;
  return [
    [-halfWidth, -halfHeight],
    [halfWidth, -halfHeight],
    [halfWidth, halfHeight],
    [-halfWidth, halfHeight]
  ].map(([dx, dy]) => ({
    x: x + dx * cos - dy * sin,
    y: y + dx * sin + dy * cos
  }));
}

function cross(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function onSegment(a, b, p) {
  const epsilon = 1e-8;
  return Math.abs(cross(a, b, p)) <= epsilon
    && p.x >= Math.min(a.x, b.x) - epsilon
    && p.x <= Math.max(a.x, b.x) + epsilon
    && p.y >= Math.min(a.y, b.y) - epsilon
    && p.y <= Math.max(a.y, b.y) + epsilon;
}

function segmentsIntersect(a, b, c, d) {
  const abC = cross(a, b, c);
  const abD = cross(a, b, d);
  const cdA = cross(c, d, a);
  const cdB = cross(c, d, b);
  if (((abC > 0 && abD < 0) || (abC < 0 && abD > 0))
      && ((cdA > 0 && cdB < 0) || (cdA < 0 && cdB > 0))) {
    return true;
  }
  return onSegment(a, b, c)
    || onSegment(a, b, d)
    || onSegment(c, d, a)
    || onSegment(c, d, b);
}

function pointInPolygon(point, polygon) {
  let inside = false;
  for (let index = 0, previous = polygon.length - 1;
       index < polygon.length;
       previous = index++) {
    const currentPoint = polygon[index];
    const previousPoint = polygon[previous];
    if (onSegment(previousPoint, currentPoint, point)) {
      return true;
    }
    const intersects = (currentPoint.y > point.y) !== (previousPoint.y > point.y)
      && point.x < (previousPoint.x - currentPoint.x)
      * (point.y - currentPoint.y)
      / (previousPoint.y - currentPoint.y)
      + currentPoint.x;
    if (intersects) inside = !inside;
  }
  return inside;
}

function polygonsIntersect(a, b) {
  if (a.some((point) => pointInPolygon(point, b))
      || b.some((point) => pointInPolygon(point, a))) {
    return true;
  }
  for (let ai = 0; ai < a.length; ai++) {
    const aNext = (ai + 1) % a.length;
    for (let bi = 0; bi < b.length; bi++) {
      const bNext = (bi + 1) % b.length;
      if (segmentsIntersect(a[ai], a[aNext], b[bi], b[bNext])) {
        return true;
      }
    }
  }
  return false;
}

function validate(filename) {
  const data = JSON.parse(readFileSync(resolve(visualizerDirectory, filename), "utf8"));
  const margin = Number(data.settings.safetyMargin || 0);
  const width = Number(data.settings.rWidth) + 2 * margin;
  const height = Number(data.settings.rHeight) + 2 * margin;
  const warnings = [];
  let start = data.startPoint;

  data.lines.forEach((line) => {
    const points = [start, ...line.controlPoints, line.endPoint];
    for (let sample = 0; sample <= 400; sample++) {
      const t = sample / 400;
      const center = curvePoint(points, t);
      const heading = shortestHeading(
        line.endPoint.startDeg,
        line.endPoint.endDeg,
        t);
      const robot = robotCorners(center.x, center.y, heading, width, height);
      const outsideField = robot.some(({ x, y }) =>
        x < 0 || x > fieldSize || y < 0 || y > fieldSize);
      const obstacle = data.shapes.find((shape) =>
        polygonsIntersect(robot, shape.vertices));
      if (outsideField || obstacle) {
        warnings.push({
          line: line.name,
          t: Number(t.toFixed(3)),
          reason: outsideField ? "field boundary" : obstacle.name
        });
        break;
      }
    }
    start = line.endPoint;
  });

  console.log(`\n${filename}`);
  console.log(`Collision envelope: ${width} x ${height} in (includes margin)`);
  if (warnings.length === 0) {
    console.log("No sampled boundary/obstacle intersections.");
  } else {
    warnings.forEach((warning) =>
      console.log(`WARNING ${warning.line} @ t=${warning.t}: ${warning.reason}`));
  }
}

readdirSync(visualizerDirectory)
  .filter((filename) => filename.endsWith(".pp"))
  .sort()
  .forEach(validate);
