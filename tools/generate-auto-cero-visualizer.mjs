import { mkdirSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const projectRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const outputDirectory = resolve(projectRoot, "visualizer");

const settings = {
  xVelocity: 80.3,
  yVelocity: 64,
  aVelocity: Math.PI,
  kFriction: 0.1,
  rWidth: 16.65,
  rHeight: 13.03,
  safetyMargin: 1,
  maxVelocity: 40,
  maxAcceleration: 30,
  maxDeceleration: 30,
  fieldMap: "decode.webp",
  robotImage: "/robot.png",
  theme: "auto",
  showGhostPaths: false,
  showOnionLayers: false,
  onionLayerSpacing: 3,
  onionColor: "#dc2626",
  onionNextPointOnly: false,
  showHeadingArrow: true,
  headingArrowLength: 50,
  headingArrowColor: "#ffffff",
  headingArrowThickness: 2,
  pathOpacity: 1
};

const shapes = [
  {
    id: "triangle-1",
    name: "Red Goal",
    vertices: [
      { x: 141.5, y: 70 },
      { x: 141.5, y: 141.5 },
      { x: 120, y: 141.5 },
      { x: 138, y: 119 },
      { x: 138, y: 70 }
    ],
    color: "#dc2626",
    fillColor: "#ff6b6b"
  },
  {
    id: "triangle-2",
    name: "Blue Goal",
    vertices: [
      { x: 6, y: 119 },
      { x: 25, y: 141.5 },
      { x: 0, y: 141.5 },
      { x: 0, y: 70 },
      { x: 6, y: 70 }
    ],
    color: "#2563eb",
    fillColor: "#60a5fa"
  }
];

const clean = (value) => Math.round(value * 1_000_000) / 1_000_000;
const normalize = (degrees) => ((degrees % 360) + 360) % 360;
const pathColors = [
  "#facc15", "#22c55e", "#3b82f6", "#f97316", "#a855f7", "#14b8a6",
  "#ec4899", "#84cc16", "#06b6d4", "#ef4444", "#8b5cf6"
];
const pose = (x, y, heading) => ({
  x: clean(x),
  y: clean(y),
  heading: clean(normalize(heading))
});

function redRoute() {
  const p00 = pose(123.457, 122.478, -135.2839);
  const p01 = pose(91.8, 89.8, 233.841815);
  const c02Ready = pose(87.68229032258066, 72.26816129032255, 0);
  const p02Ready = pose(85.81290322580645, 59, 0);
  const p02 = pose(131, 59, 0);
  const c03 = pose(99, 60, 0);
  const p03 = pose(84, 83, 227.726311);
  const p04 = pose(99, 83, 0);
  const p05 = pose(125, 83, 0);
  const p06 = pose(84, 83, 227.726311);
  const c07 = pose(86.83790322580646, 42.416129032258056, 0);
  const p07 = pose(83.50161290322582, 36, 0);
  const p08 = pose(132, 36, 0);
  const p09 = pose(84.077, 107.923, 211.06761);
  const park = pose(86, 108.9, 0);
  return {
    alliance: "RED",
    color: "#dc2626",
    start: p00,
    segments: [
      ["01 START -> PRELOAD REAR SCORE", p01, []],
      ["02 SCORE -> ROW 2 READY", p02Ready, [c02Ready]],
      ["03 COLLECT ROW 2", p02, []],
      ["04 ROW 2 -> REAR SCORE", p03, [c03]],
      ["05 SCORE -> ROW 1 READY", p04, []],
      ["06 COLLECT ROW 1", p05, []],
      ["07 ROW 1 -> REAR SCORE", p06, []],
      ["08 SCORE -> ROW 3 READY", p07, [c07]],
      ["09 COLLECT ROW 3", p08, []],
      ["10 ROW 3 -> FINAL REAR SCORE", p09, []],
      ["11 FINAL SCORE -> PARK", park, []]
    ]
  };
}

function blueRoute() {
  const p00 = pose(20.543, 122.478, 315.2839);
  const p01 = pose(52.2, 89.8, -53.841815);
  const c02Ready = pose(56.3177096774193, 72.26816129032255, 180);
  const p02Ready = pose(58.1870967741935, 59, 180);
  const p02 = pose(13, 59, 180);
  const c03 = pose(45, 60, 180);
  const p03 = pose(60, 83, 312.273689);
  const p04 = pose(45, 83, 180);
  const p05 = pose(19, 83, 180);
  const p06 = pose(60, 83, 312.273689);
  const c07 = pose(57.1620967741935, 42.416129032258056, 180);
  const p07 = pose(60.4983870967742, 36, 180);
  const p08 = pose(12, 36, 180);
  const p09 = pose(59.923, 107.923, 328.93239);
  const park = pose(58, 108.9, 180);
  return {
    alliance: "BLUE",
    color: "#2563eb",
    start: p00,
    segments: [
      ["01 START -> PRELOAD REAR SCORE", p01, []],
      ["02 SCORE -> ROW 2 READY", p02Ready, [c02Ready]],
      ["03 COLLECT ROW 2", p02, []],
      ["04 ROW 2 -> REAR SCORE", p03, [c03]],
      ["05 SCORE -> ROW 1 READY", p04, []],
      ["06 COLLECT ROW 1", p05, []],
      ["07 ROW 1 -> REAR SCORE", p06, []],
      ["08 SCORE -> ROW 3 READY", p07, [c07]],
      ["09 COLLECT ROW 3", p08, []],
      ["10 ROW 3 -> FINAL REAR SCORE", p09, []],
      ["11 FINAL SCORE -> PARK", park, []]
    ]
  };
}

function pointForVisualizer(value, startHeading) {
  return {
    x: value.x,
    y: value.y,
    heading: "linear",
    startDeg: startHeading,
    endDeg: value.heading,
    locked: false
  };
}

function buildTrajectory(route) {
  let previous = route.start;
  const lines = route.segments.map(([name, end, controls], index) => {
    const id = `${route.alliance.toLowerCase()}-cero-${String(index + 1).padStart(2, "0")}`;
    const line = {
      id,
      name: `${route.alliance} ${name}`,
      endPoint: pointForVisualizer(end, previous.heading),
      controlPoints: controls.map(({ x, y }) => ({ x, y, locked: false })),
      color: pathColors[index],
      locked: false,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: ""
    };
    previous = end;
    return line;
  });

  const shootingLines = new Set([1, 4, 7, 10]);
  const sequence = [];
  lines.forEach((line, index) => {
    sequence.push({ kind: "path", lineId: line.id });
    if (shootingLines.has(index + 1)) {
      sequence.push({
        kind: "wait",
        id: `${line.id}-shoot`,
        name: `${route.alliance} rear shoot`,
        durationMs: 925,
        locked: false
      });
    }
  });

  return {
    startPoint: {
      x: route.start.x,
      y: route.start.y,
      heading: "constant",
      degrees: route.start.heading,
      locked: false
    },
    lines,
    shapes,
    settings: { ...settings, onionColor: route.color },
    sequence,
    pathChains: [
      {
        id: `${route.alliance.toLowerCase()}-auto-cero`,
        name: `${route.alliance} AutoCero Close Zero Door Rear Shooter`,
        color: route.color,
        lineIds: lines.map((line) => line.id)
      }
    ]
  };
}

mkdirSync(outputDirectory, { recursive: true });
for (const route of [redRoute(), blueRoute()]) {
  writeFileSync(
    resolve(outputDirectory, `${route.alliance}-AutoCero-Close-ZeroDoor-RearShooter.pp`),
    `${JSON.stringify(buildTrajectory(route), null, 2)}\n`,
    "utf8"
  );
}

console.log(`Generated AutoCero Visualizer files in ${outputDirectory}`);
