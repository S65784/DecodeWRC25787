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
  showGhostPaths: true,
  showOnionLayers: true,
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
const pose = (x, y, heading) => ({
  x: clean(x),
  y: clean(y),
  heading: clean(normalize(heading))
});

function redRoute() {
  const p00 = pose(79.725, 9.431, -90);
  const p01 = pose(91, 14.65060240963856, 250);
  const c02 = pose(103.71084337349397, 4.2409638554216915, -22);
  const p02 = pose(127.42168674698794, 19.66265060240964, -22);
  const c03a = pose(136.72289156626508, 18.313253012048197, 0);
  const c03b = pose(122.60240963855422, 12.530120481927707, 0);
  const p03 = pose(136.09638554216866, 7.518072289156624, 0);
  const c04 = pose(104.09638554216868, 19.46987951807229, 72);
  const p04 = pose(92.5, 15, 243);
  const c05 = pose(103.71084337349397, 4.2409638554216915, -22);
  const p05 = pose(127.42168674698794, 19.66265060240964, -22);
  const c06a = pose(136.72289156626508, 18.313253012048197, 0);
  const c06b = pose(122.60240963855422, 12.530120481927707, 0);
  const p06 = pose(136.09638554216866, 7.518072289156624, 0);
  const c07 = pose(104.09638554216868, 19.46987951807229, 72);
  const p07 = pose(92.5, 15, 245);
  const p08 = pose(105.4, 14.5, 250);

  return {
    alliance: "RED",
    color: "#dc2626",
    start: p00,
    segments: [
      ["01 PRELOAD -> REAR SCORE", p01, []],
      ["02 SCORE -> PICKUP READY 1", p02, [c02]],
      ["03 FRONT COLLECT 1", p03, [c03a, c03b]],
      ["04 PICKUP 1 -> REAR SCORE 1", p04, [c04]],
      ["05 SCORE 1 -> PICKUP READY 2", p05, [c05]],
      ["06 FRONT COLLECT 2", p06, [c06a, c06b]],
      ["07 PICKUP 2 -> REAR SCORE 2", p07, [c07]],
      ["08 REAR SCORE 2 -> PARK", p08, []]
    ]
  };
}

function blueRoute() {
  // Explicit values keep this file independent from the red route.
  const p00 = pose(64.275, 9.431, 270);
  const p01 = pose(53, 14.65060240963856, 290);
  const c02 = pose(40.28915662650603, 4.2409638554216915, 202);
  const p02 = pose(16.57831325301206, 19.66265060240964, 202);
  const c03a = pose(7.27710843373492, 18.313253012048197, 180);
  const c03b = pose(21.39759036144578, 12.530120481927707, 180);
  const p03 = pose(7.90361445783134, 7.518072289156624, 180);
  const c04 = pose(39.90361445783132, 19.46987951807229, 108);
  const p04 = pose(51.5, 15, 297);
  const c05 = pose(40.28915662650603, 4.2409638554216915, 202);
  const p05 = pose(16.57831325301206, 19.66265060240964, 202);
  const c06a = pose(7.27710843373492, 18.313253012048197, 180);
  const c06b = pose(21.39759036144578, 12.530120481927707, 180);
  const p06 = pose(7.90361445783134, 7.518072289156624, 180);
  const c07 = pose(39.90361445783132, 19.46987951807229, 108);
  const p07 = pose(51.5, 15, 295);
  const p08 = pose(38.6, 14.5, 290);

  return {
    alliance: "BLUE",
    color: "#2563eb",
    start: p00,
    segments: [
      ["01 PRELOAD -> REAR SCORE", p01, []],
      ["02 SCORE -> PICKUP READY 1", p02, [c02]],
      ["03 FRONT COLLECT 1", p03, [c03a, c03b]],
      ["04 PICKUP 1 -> REAR SCORE 1", p04, [c04]],
      ["05 SCORE 1 -> PICKUP READY 2", p05, [c05]],
      ["06 FRONT COLLECT 2", p06, [c06a, c06b]],
      ["07 PICKUP 2 -> REAR SCORE 2", p07, [c07]],
      ["08 REAR SCORE 2 -> PARK", p08, []]
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
    const id = `${route.alliance.toLowerCase()}-dos-${String(index + 1).padStart(2, "0")}`;
    const line = {
      id,
      name: `${route.alliance} ${name}`,
      endPoint: pointForVisualizer(end, previous.heading),
      controlPoints: controls.map(({ x, y }) => ({ x, y, locked: false })),
      color: route.color,
      locked: false,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: ""
    };
    previous = end;
    return line;
  });

  const shootingLineNumbers = new Set([1, 4, 7]);
  const sequence = [];
  lines.forEach((line, index) => {
    sequence.push({ kind: "path", lineId: line.id });
    if (shootingLineNumbers.has(index + 1)) {
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
    settings: {
      ...settings,
      onionColor: route.color
    },
    sequence,
    pathChains: [
      {
        id: `${route.alliance.toLowerCase()}-auto-dos`,
        name: `${route.alliance} AutoDos Far 144 Independent Route`,
        color: route.color,
        lineIds: lines.map((line) => line.id)
      }
    ]
  };
}

mkdirSync(outputDirectory, { recursive: true });

for (const route of [redRoute(), blueRoute()]) {
  const filename = `${route.alliance}-AutoDos-Far-144-FrontIntake-RearShooter.pp`;
  writeFileSync(
    resolve(outputDirectory, filename),
    `${JSON.stringify(buildTrajectory(route), null, 2)}\n`,
    "utf8"
  );
}

console.log(`Generated AutoDos Visualizer files in ${outputDirectory}`);
