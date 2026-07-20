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
  // At heading 0 the local X dimension is the robot's front-to-back length.
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
const pathColors = [
  "#facc15", "#22c55e", "#3b82f6", "#f97316", "#a855f7", "#14b8a6",
  "#ec4899", "#84cc16", "#06b6d4", "#ef4444", "#8b5cf6", "#10b981",
  "#0ea5e9"
];
const pose = (x, y, heading) => ({
  x: clean(x),
  y: clean(y),
  heading: clean(heading)
});

function redRoute() {
  const p00 = pose(123.457, 122.478, -135.2839);
  const p01 = pose(91.8, 89.8, 233.841815);
  const p02 = pose(99, 83, 0);
  const p03 = pose(125, 83, 0);
  const c04 = pose(115.20243220665724, 79.23203637064647, 0);
  const p04 = pose(128.16370967741938, 71.54354838709679, -90);
  const p05 = pose(84, 83, 227.726311);
  const c06 = pose(87.68229032258066, 72.26816129032255, 0);
  const p06 = pose(85.81290322580645, 59, 0);
  const p07 = pose(131, 59, 0);
  const c08 = pose(126.66532258064517, 66.64193548387098, 270);
  const p08 = pose(127.7790322580645, 73.82580645161289, 0);
  const c09 = pose(99, 60, 0);
  const p09 = pose(84, 83, 227.726311);
  const c10 = pose(86.83790322580646, 42.416129032258056, 0);
  const p10 = pose(83.50161290322582, 36, 0);
  const p11 = pose(132, 36, 0);
  const p12 = pose(84.077, 107.923, 211.06761);
  const p13 = pose(86, 108.9, 0);

  return {
    alliance: "RED",
    color: "#dc2626",
    start: p00,
    segments: [
      ["01 START -> PRELOAD SCORE", p01, []],
      ["02 SCORE -> ROW 1 READY", p02, []],
      ["03 COLLECT ROW 1", p03, []],
      ["04 PUSH GATE 1", p04, [c04]],
      ["05 GATE 1 -> ROW 1 SCORE", p05, []],
      ["06 SCORE -> ROW 2 READY", p06, [c06]],
      ["07 COLLECT ROW 2", p07, []],
      ["08 PUSH GATE 2", p08, [c08]],
      ["09 GATE 2 -> ROW 2 SCORE", p09, [c09]],
      ["10 SCORE -> ROW 3 READY", p10, [c10]],
      ["11 COLLECT ROW 3", p11, []],
      ["12 ROW 3 -> FINAL SCORE", p12, []],
      ["13 FINAL SCORE -> PARK", p13, []]
    ]
  };
}

function blueRoute() {
  // Explicit values preserve the team's existing blue-only tuning.
  const p00 = pose(20.543, 122.478, 315.2839);
  const p01 = pose(44, 99.8, 322);
  const c02 = pose(50.05535966149506, 96.27080394922427, 180);
  const p02 = pose(45, 81.4, 180);
  const p03 = pose(15.1, 81.4, 180);
  const c04 = pose(39.06772908366534, 69.4183266932271, -90);
  const p04 = pose(11.5, 72, 180);
  const c05 = pose(29.25301204819277, 81.92771084337349, 324.4);
  const p05 = pose(44, 99.8, 322.8);
  const c06 = pose(58.24097984598117, 70.10588235294118, 180);
  const p06 = pose(45, 58.9, 180);
  const p07 = pose(15, 58.9, 180);
  const c08 = pose(38, 52.91444600280506, -90);
  const p08 = pose(11.5, 72, 180);
  const c09 = pose(40, 60.3, 328);
  const p09 = pose(44, 99.8, 323.5);
  const c10 = pose(54.41176470588236, 67.45882352941175, 180);
  const p10 = pose(45, 33.9, 180);
  const p11 = pose(12.7, 33.9, 180);
  const c12 = pose(42, 39, 328);
  const p12 = pose(44, 99.8, 325);
  const p13 = pose(45, 108, 180);

  return {
    alliance: "BLUE",
    color: "#2563eb",
    start: p00,
    segments: [
      ["01 PRELOAD -> REAR SCORE", p01, []],
      ["02 SCORE -> PICKUP 1 READY", p02, [c02]],
      ["03 FRONT COLLECT 1", p03, []],
      ["04 PUSH GATE 1", p04, [c04]],
      ["05 GATE 1 -> REAR SCORE 1", p05, [c05]],
      ["06 SCORE 1 -> PICKUP 2 READY", p06, [c06]],
      ["07 FRONT COLLECT 2", p07, []],
      ["08 PUSH GATE 2", p08, [c08]],
      ["09 GATE 2 -> REAR SCORE 2", p09, [c09]],
      ["10 SCORE 2 -> PICKUP 3 READY", p10, [c10]],
      ["11 FRONT COLLECT 3", p11, []],
      ["12 PICKUP 3 -> REAR SCORE 3", p12, [c12]],
      ["13 REAR SCORE 3 -> PARK", p13, []]
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
    const id = `${route.alliance.toLowerCase()}-${String(index + 1).padStart(2, "0")}`;
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

  const shootingLineNumbers = new Set([1, 5, 9, 12]);
  const sequence = [];
  lines.forEach((line, index) => {
    sequence.push({ kind: "path", lineId: line.id });
    if (shootingLineNumbers.has(index + 1)) {
      sequence.push({
        kind: "wait",
        id: `${line.id}-shoot`,
        name: `${route.alliance} rear shoot`,
        durationMs: 350,
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
    settings,
    sequence,
    pathChains: lines.map((line) => ({
      id: `${line.id}-chain`,
      name: line.name,
      color: line.color,
      lineIds: [line.id]
    }))
  };
}

mkdirSync(outputDirectory, { recursive: true });

const routes = process.argv.includes("--red-only")
  ? [redRoute()]
  : [redRoute(), blueRoute()];

for (const route of routes) {
  const filename = `${route.alliance}-AutoCinco-FrontIntake-RearShooter.pp`;
  writeFileSync(
    resolve(outputDirectory, filename),
    `${JSON.stringify(buildTrajectory(route), null, 2)}\n`,
    "utf8"
  );
}

console.log(`Generated Visualizer files in ${outputDirectory}`);
