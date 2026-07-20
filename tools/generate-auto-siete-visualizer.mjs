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
const pose = (x, y, heading) => ({
  x: clean(x),
  y: clean(y),
  heading: clean(normalize(heading))
});
const control = (x, y) => ({ x: clean(x), y: clean(y) });
const pathColors = [
  "#facc15",
  "#22c55e",
  "#3b82f6",
  "#f97316",
  "#a855f7",
  "#14b8a6",
  "#ec4899",
  "#84cc16",
  "#06b6d4",
  "#ef4444",
  "#8b5cf6",
  "#10b981",
  "#eab308",
  "#6366f1",
  "#0ea5e9",
  "#d946ef",
  "#65a30d"
];
const settleAndShoot = (settleMs) => [
  ["settle", "chassis settle", settleMs],
  ["shoot", "rear shoot", 400]
];
const collectDwell = (dwellMs) => [["collect", "pickup dwell", dwellMs]];

function redRoute() {
  return {
    alliance: "RED",
    color: "#dc2626",
    start: pose(79.725, 9.431, -90),
    steps: [
      {
        id: "preload-to-score",
        name: "PRELOAD_TO_SCORE",
        end: pose(89, 12, -106),
        controls: [],
        waits: settleAndShoot(350)
      },
      {
        id: "score-to-pickup-a1",
        name: "SCORE_TO_PICKUP_A1",
        end: pose(132.174, 8.174, 0),
        controls: [control(127.00241903225806, 31.604032387096776)],
        waits: collectDwell(90)
      },
      {
        id: "pickup-a1-to-score-1",
        name: "PICKUP_A1_TO_SCORE_1",
        end: pose(89, 12, -107.5),
        controls: [control(120.857, 17.619)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-1-to-prepickup-b1",
        name: "SCORE_1_TO_PREPICKUP_B1",
        end: pose(109.548, 13.465, 40),
        controls: [],
        waits: []
      },
      {
        id: "prepickup-b1-to-pickup-b1",
        name: "PREPICKUP_B1_TO_PICKUP_B1",
        end: pose(131.08680645161292, 17.493096774193557, 50),
        controls: [control(134.42516129032254, 11.639774193548401)],
        waits: collectDwell(250)
      },
      {
        id: "pickup-b1-to-score-2",
        name: "PICKUP_B1_TO_SCORE_2",
        end: pose(89, 12.5, -108),
        controls: [control(113.695, 25.131)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-2-to-pickup-a2",
        name: "SCORE_2_TO_PICKUP_A2",
        end: pose(132.83041674893587, 11, 270),
        controls: [control(140.61762772930666, 28.464703606145186)],
        waits: collectDwell(200)
      },
      {
        id: "pickup-a2-to-score-3",
        name: "PICKUP_A2_TO_SCORE_3",
        end: pose(89, 12, -109),
        controls: [control(120.857, 17.619)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-3-to-prepickup-b2",
        name: "SCORE_3_TO_PREPICKUP_B2",
        end: pose(109.548, 13.465, 40),
        controls: [],
        waits: []
      },
      {
        id: "prepickup-b2-to-pickup-b2",
        name: "PREPICKUP_B2_TO_PICKUP_B2",
        end: pose(131.08680645161292, 17.493096774193557, 50),
        controls: [control(134.42516129032254, 11.639774193548401)],
        waits: collectDwell(200)
      },
      {
        id: "pickup-b2-to-score-4",
        name: "PICKUP_B2_TO_SCORE_4",
        end: pose(89, 12.5, -109),
        controls: [control(113.695, 25.131)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-4-to-pickup-a3",
        name: "SCORE_4_TO_PICKUP_A3",
        end: pose(132.83041674893587, 11, 270),
        controls: [control(140.38940192285503, 28.464703606145186)],
        waits: collectDwell(150)
      },
      {
        id: "pickup-a3-to-score-5",
        name: "PICKUP_A3_TO_SCORE_5",
        end: pose(89, 12, -109),
        controls: [control(120.857, 17.619)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-5-to-prepickup-b3",
        name: "SCORE_5_TO_PREPICKUP_B3",
        end: pose(109.548, 13.465, 40),
        controls: [],
        waits: []
      },
      {
        id: "prepickup-b3-to-pickup-b3",
        name: "PREPICKUP_B3_TO_PICKUP_B3",
        end: pose(131.08680645161292, 17.493096774193557, 50),
        controls: [control(134.42516129032254, 11.639774193548401)],
        waits: collectDwell(200)
      },
      {
        id: "pickup-b3-to-score-6",
        name: "PICKUP_B3_TO_SCORE_6",
        end: pose(89, 12.5, -109),
        controls: [control(113.695, 25.131)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-6-to-park",
        name: "SCORE_6_TO_PARK",
        end: pose(115.58225806451613, 22.67580645161289, 0),
        controls: [control(98.01774193548384, 24.5290322580645)],
        waits: []
      }
    ]
  };
}

function blueRoute() {
  return {
    alliance: "BLUE",
    color: "#2563eb",
    start: pose(64.275, 9.431, 270),
    steps: [
      {
        id: "preload-to-score",
        name: "PRELOAD_TO_SCORE",
        end: pose(55, 12, 286),
        controls: [],
        waits: settleAndShoot(300)
      },
      {
        id: "score-to-pickup-a1",
        name: "SCORE_TO_PICKUP_A1",
        end: pose(11.826, 8.174, 180),
        controls: [control(16.99758096774194, 31.604032387096776)],
        waits: collectDwell(300)
      },
      {
        id: "pickup-a1-to-score-1",
        name: "PICKUP_A1_TO_SCORE_1",
        end: pose(55, 12, 287.5),
        controls: [control(23.143, 17.619)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-1-to-prepickup-b1",
        name: "SCORE_1_TO_PREPICKUP_B1",
        end: pose(34.452, 13.465, 140),
        controls: [],
        waits: []
      },
      {
        id: "prepickup-b1-to-pickup-b1",
        name: "PREPICKUP_B1_TO_PICKUP_B1",
        end: pose(12.91319354838708, 17.493096774193557, 130),
        controls: [control(9.57483870967746, 11.639774193548401)],
        waits: collectDwell(300)
      },
      {
        id: "pickup-b1-to-score-2",
        name: "PICKUP_B1_TO_SCORE_2",
        end: pose(55, 12.5, 288),
        controls: [control(30.305, 25.131)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-2-to-pickup-a2",
        name: "SCORE_2_TO_PICKUP_A2",
        end: pose(11.16958325106413, 11, -90),
        controls: [control(3.38237227069334, 28.464703606145186)],
        waits: collectDwell(300)
      },
      {
        id: "pickup-a2-to-score-3",
        name: "PICKUP_A2_TO_SCORE_3",
        end: pose(55, 12, 289),
        controls: [control(23.143, 17.619)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-3-to-prepickup-b2",
        name: "SCORE_3_TO_PREPICKUP_B2",
        end: pose(34.452, 13.465, 140),
        controls: [],
        waits: []
      },
      {
        id: "prepickup-b2-to-pickup-b2",
        name: "PREPICKUP_B2_TO_PICKUP_B2",
        end: pose(12.91319354838708, 17.493096774193557, 130),
        controls: [control(9.57483870967746, 11.639774193548401)],
        waits: collectDwell(300)
      },
      {
        id: "pickup-b2-to-score-4",
        name: "PICKUP_B2_TO_SCORE_4",
        end: pose(55, 12.5, 289),
        controls: [control(30.305, 25.131)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-4-to-pickup-a3",
        name: "SCORE_4_TO_PICKUP_A3",
        end: pose(11.16958325106413, 11, -90),
        controls: [control(3.61059807714497, 28.464703606145186)],
        waits: collectDwell(300)
      },
      {
        id: "pickup-a3-to-score-5",
        name: "PICKUP_A3_TO_SCORE_5",
        end: pose(55, 12, 289),
        controls: [control(23.143, 17.619)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-5-to-prepickup-b3",
        name: "SCORE_5_TO_PREPICKUP_B3",
        end: pose(34.452, 13.465, 140),
        controls: [],
        waits: []
      },
      {
        id: "prepickup-b3-to-pickup-b3",
        name: "PREPICKUP_B3_TO_PICKUP_B3",
        end: pose(12.91319354838708, 17.493096774193557, 130),
        controls: [control(9.57483870967746, 11.639774193548401)],
        waits: collectDwell(300)
      },
      {
        id: "pickup-b3-to-score-6",
        name: "PICKUP_B3_TO_SCORE_6",
        end: pose(55, 12.5, 289),
        controls: [control(30.305, 25.131)],
        waits: settleAndShoot(300)
      },
      {
        id: "score-6-to-park",
        name: "SCORE_6_TO_PARK",
        end: pose(28.41774193548387, 22.67580645161289, 180),
        controls: [control(45.98225806451616, 24.5290322580645)],
        waits: []
      }
    ]
  };
}

function buildTrajectory(route) {
  let previous = route.start;
  const lines = route.steps.map((step, index) => {
    const line = {
      id: `${route.alliance.toLowerCase()}-siete-${step.id}`,
      name: `${route.alliance} ${step.name}`,
      endPoint: {
        x: step.end.x,
        y: step.end.y,
        heading: "linear",
        startDeg: previous.heading,
        endDeg: step.end.heading,
        locked: false
      },
      controlPoints: step.controls.map(({ x, y }) => ({ x, y, locked: false })),
      color: pathColors[index % pathColors.length],
      locked: false,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: ""
    };
    previous = step.end;
    return line;
  });

  const sequence = [];
  lines.forEach((line, index) => {
    sequence.push({ kind: "path", lineId: line.id });
    for (const [suffix, name, durationMs] of route.steps[index].waits) {
      sequence.push({
        kind: "wait",
        id: `${line.id}-${suffix}`,
        name: `${route.alliance} ${name}`,
        durationMs,
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
    pathChains: lines.map((line) => ({
      id: `${line.id}-chain`,
      name: line.name,
      color: line.color,
      lineIds: [line.id]
    }))
  };
}

mkdirSync(outputDirectory, { recursive: true });
for (const route of [redRoute(), blueRoute()]) {
  writeFileSync(
    resolve(outputDirectory, `${route.alliance}-AutoSiete-Far-SixCycle-RearShooter.pp`),
    `${JSON.stringify(buildTrajectory(route), null, 2)}\n`,
    "utf8"
  );
}

console.log(`Generated standalone semantic AutoSiete files in ${outputDirectory}`);
