import fs from "node:fs";
import path from "node:path";

const projectRoot = path.resolve(import.meta.dirname, "..");
const autoRoot = path.join(
  projectRoot,
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Auto",
);
const configFile = path.join(
  projectRoot,
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode/config/DecodeConfig.java",
);
const visualizerRoot = path.join(projectRoot, "visualizer");

// RedAutoOcho currently defines 20 segments. Colors must not repeat within one .pp file.
const colors = [
  "#facc15",
  "#22c55e",
  "#3b82f6",
  "#f97316",
  "#a855f7",
  "#14b8a6",
  "#ef4444",
  "#84cc16",
  "#06b6d4",
  "#8b5cf6",
  "#10b981",
  "#eab308",
  "#6366f1",
  "#0ea5e9",
  "#d946ef",
  "#65a30d",
  "#fb7185",
  "#7c3aed",
  "#0891b2",
  "#f59e0b",
];

function parseNumericConstants(source, visibility) {
  const values = new Map();
  const pattern = new RegExp(
    `${visibility} static final (?:double|long)\\s+(\\w+)\\s*=\\s*([^;]+);`,
    "g",
  );
  for (const match of source.matchAll(pattern)) {
    const expression = match[2].trim().replaceAll("_", "");
    const number = Number(expression);
    if (Number.isFinite(number)) {
      values.set(match[1], number);
    }
  }
  return values;
}

function numericExpression(expression, context) {
  const normalized = expression.trim().replaceAll("_", "");
  if (!/^[0-9+\-*/().\s]+$/.test(normalized)) {
    throw new Error(`Unsupported numeric expression ${expression} in ${context}`);
  }
  const result = Function(`"use strict"; return (${normalized});`)();
  if (!Number.isFinite(result)) {
    throw new Error(`Non-finite numeric expression ${expression} in ${context}`);
  }
  return result;
}

const decodeConfigSource = fs.readFileSync(configFile, "utf8");
const decodeValues = parseNumericConstants(decodeConfigSource, "public");

function parseJava(relativeFile) {
  const source = fs.readFileSync(path.join(autoRoot, relativeFile), "utf8");
  const poses = new Map();
  const posePattern =
    /^\s*private final Pose\s+(\w+)\s*=\s*pose\(\s*([^,]+),\s*([^,]+),\s*([^)]+)\);/gm;
  for (const match of source.matchAll(posePattern)) {
    poses.set(match[1], {
      x: numericExpression(match[2], relativeFile),
      y: numericExpression(match[3], relativeFile),
      heading: numericExpression(match[4], relativeFile),
    });
  }

  const newPosePattern =
    /^\s*private final Pose\s+(\w+)\s*=\s*new Pose\(\s*([^,]+),\s*([^,]+),\s*Math\.toRadians\(\s*([^)]+)\s*\)\s*\);/gm;
  for (const match of source.matchAll(newPosePattern)) {
    poses.set(match[1], {
      x: numericExpression(match[2], relativeFile),
      y: numericExpression(match[3], relativeFile),
      heading: numericExpression(match[4], relativeFile),
    });
  }

  const constants = parseNumericConstants(source, "private");
  const aliasPattern =
    /private static final (?:double|long)\s+(\w+)\s*=\s*DecodeConfig\.(\w+);/g;
  for (const match of source.matchAll(aliasPattern)) {
    if (!decodeValues.has(match[2])) {
      throw new Error(`Unknown DecodeConfig value ${match[2]} in ${relativeFile}`);
    }
    constants.set(match[1], decodeValues.get(match[2]));
  }

  return { relativeFile, source, poses, constants };
}

function p(java, name) {
  const value = java.poses.get(name);
  if (!value) {
    throw new Error(`Missing pose ${name} in ${java.relativeFile}`);
  }
  return value;
}

function value(java, name) {
  if (java.constants.has(name)) return java.constants.get(name);
  if (decodeValues.has(name)) return decodeValues.get(name);
  throw new Error(`Missing numeric value ${name} in ${java.relativeFile}`);
}

function segment(name, start, end, controls = []) {
  return { name, start, end, controls };
}

function wait(name, durationMs) {
  return { kind: "wait", name, durationMs };
}

function routeCero(java, alliance) {
  const red = alliance === "RED";
  const segments = [
    segment("01 START -> PRELOAD SCORE", p(java, "p00Start"), p(java, "p01PreloadScore")),
    segment(
      "02 COLLECT ROW 2",
      p(java, "p01PreloadScore"),
      p(java, "p02RowTwoPickup"),
      red
        ? [p(java, "c02RowTwoPickup"), p(java, "cc02RowTwoPickup")]
        : [p(java, "c02RowTwoPickup")],
    ),
    segment(
      "03 ROW 2 -> SCORE",
      p(java, "p02RowTwoPickup"),
      p(java, "p03RowTwoScore"),
      [p(java, "c03RowTwoScore")],
    ),
    segment(
      "04 SCORE -> ROW 1 READY",
      p(java, "p03RowTwoScore"),
      p(java, "p04RowOneReady"),
    ),
    segment(
      "05 COLLECT ROW 1",
      p(java, "p04RowOneReady"),
      p(java, "p05RowOnePickup"),
    ),
    segment(
      "06 ROW 1 -> SCORE",
      p(java, "p05RowOnePickup"),
      p(java, "p06RowOneScore"),
    ),
    segment(
      "07 SCORE -> ROW 3 READY",
      p(java, "p06RowOneScore"),
      p(java, "p07RowThreeReady"),
      red ? [p(java, "c07RowThreeReady")] : [],
    ),
    segment(
      "08 COLLECT ROW 3",
      p(java, "p07RowThreeReady"),
      p(java, "p08RowThreePickup"),
    ),
    segment(
      "09 ROW 3 -> FINAL SCORE",
      p(java, "p08RowThreePickup"),
      p(java, "p09FinalScore"),
    ),
  ];
  if (red) {
    segments.push(
      segment(
        "10 FINAL SCORE -> PARK",
        p(java, "p09FinalScore"),
        p(java, "parkPose"),
      ),
    );
  }

  const fireMs = value(java, "AUTO_NEAR_FIRE_DURATION_MS");
  const sequence = [];
  segments.forEach((_, index) => {
    sequence.push({ kind: "path", index });
    if ([0, 2, 5, 8].includes(index)) {
      sequence.push(wait(`${String(index + 1).padStart(2, "0")} FIRE`, fireMs));
    }
  });
  return { start: p(java, "p00Start"), segments, sequence };
}

function routeCinco(java) {
  const segments = [
    segment("01 START -> PRELOAD SCORE", p(java, "p00Start"), p(java, "p01PreloadScore")),
    segment(
      "02 SCORE -> ROW 1 READY",
      p(java, "p01PreloadScore"),
      p(java, "p02RowOneReady"),
    ),
    segment(
      "03 COLLECT ROW 1",
      p(java, "p02RowOneReady"),
      p(java, "p03RowOnePickup"),
    ),
    segment(
      "04 PUSH GATE 1",
      p(java, "p03RowOnePickup"),
      p(java, "p04GateOne"),
      [p(java, "c04GateOne")],
    ),
    segment(
      "05 GATE 1 -> ROW 1 SCORE",
      p(java, "p04GateOne"),
      p(java, "p05RowOneScore"),
    ),
    segment(
      "06 SCORE -> ROW 2 READY",
      p(java, "p05RowOneScore"),
      p(java, "p06RowTwoReady"),
      [p(java, "c06RowTwoReady")],
    ),
    segment(
      "07 COLLECT ROW 2",
      p(java, "p06RowTwoReady"),
      p(java, "p07RowTwoPickup"),
    ),
    segment(
      "08 PUSH GATE 2",
      p(java, "p07RowTwoPickup"),
      p(java, "p08GateTwo"),
      [p(java, "c08GateTwo")],
    ),
    segment(
      "09 GATE 2 -> ROW 2 SCORE",
      p(java, "p08GateTwo"),
      p(java, "p09RowTwoScore"),
      [p(java, "c09RowTwoScore")],
    ),
    segment(
      "10 SCORE -> ROW 3 READY",
      p(java, "p09RowTwoScore"),
      p(java, "p10RowThreeReady"),
      [p(java, "c10RowThreeReady")],
    ),
    segment(
      "11 COLLECT ROW 3",
      p(java, "p10RowThreeReady"),
      p(java, "p11RowThreePickup"),
    ),
    segment(
      "12 ROW 3 -> FINAL SCORE",
      p(java, "p11RowThreePickup"),
      p(java, "p12FinalScore"),
    ),
    segment("13 FINAL SCORE -> PARK", p(java, "p12FinalScore"), p(java, "p13Park")),
  ];

  const fireMs = value(java, "AUTO_NEAR_FIRE_DURATION_MS");
  const sequence = [];
  segments.forEach((_, index) => {
    sequence.push({ kind: "path", index });
    if ([0, 4, 8, 11].includes(index)) {
      sequence.push(wait(`${String(index + 1).padStart(2, "0")} FIRE`, fireMs));
    }
  });
  return { start: p(java, "p00Start"), segments, sequence };
}

function routeDos(java) {
  const segments = [
    segment("01 START -> PRELOAD SCORE", p(java, "p00Start"), p(java, "p01PreloadScore")),
    segment(
      "02 SCORE -> PICKUP READY 1",
      p(java, "p01PreloadScore"),
      p(java, "p02PickupReady"),
      [p(java, "c02PickupReady")],
    ),
    segment(
      "03 COLLECT 1",
      p(java, "p02PickupReady"),
      p(java, "p03Pickup"),
      [p(java, "c03CollectOne"), p(java, "c03CollectTwo")],
    ),
    segment(
      "04 PICKUP 1 -> SCORE 1",
      p(java, "p03Pickup"),
      p(java, "p04ScoreOne"),
      [p(java, "c04ScoreOne")],
    ),
    segment(
      "05 SCORE 1 -> PICKUP READY 2",
      p(java, "p04ScoreOne"),
      p(java, "p05PickupReady"),
      [p(java, "c05PickupReady")],
    ),
    segment(
      "06 COLLECT 2",
      p(java, "p05PickupReady"),
      p(java, "p06Pickup"),
      [p(java, "c06CollectOne"), p(java, "c06CollectTwo")],
    ),
    segment(
      "07 PICKUP 2 -> SCORE 2",
      p(java, "p06Pickup"),
      p(java, "p07ScoreTwo"),
      [p(java, "c07ScoreTwo")],
    ),
    segment("08 SCORE 2 -> PARK", p(java, "p07ScoreTwo"), p(java, "p08Park")),
  ];

  const fireMs = value(java, "AUTO_FIRE_DURATION_MS");
  const sequence = [];
  segments.forEach((_, index) => {
    sequence.push({ kind: "path", index });
    if ([0, 3, 6].includes(index)) {
      sequence.push(wait(`${String(index + 1).padStart(2, "0")} FIRE`, fireMs));
    }
  });
  return { start: p(java, "p00Start"), segments, sequence };
}

function routeSieteBlue(java) {
  const names = ["A1", "B1", "A2", "B2", "A3", "B3"];
  const segments = [
    segment("01 PRELOAD -> SCORE", p(java, "startPose"), p(java, "preloadScorePose")),
  ];
  let score = p(java, "preloadScorePose");
  names.forEach((cycle, index) => {
    const pickup = p(java, `pickup${cycle}Pose`);
    const nextScore = p(java, `scoreAfter${cycle}Pose`);
    const outboundName =
      index === 0
        ? "02 SCORE -> PICKUP A1"
        : `${String(index * 2 + 2).padStart(2, "0")} SCORE ${index} -> PICKUP ${cycle}`;
    segments.push(
      segment(
        outboundName,
        score,
        pickup,
        [p(java, `controlScoreTo${cycle}`)],
      ),
      segment(
        `${String(index * 2 + 3).padStart(2, "0")} PICKUP ${cycle} -> SCORE ${index + 1}`,
        pickup,
        nextScore,
        [p(java, `control${cycle}ToScore`)],
      ),
    );
    score = nextScore;
  });
  segments.push(
    segment("14 SCORE 6 -> PARK", score, p(java, "parkPose"), [
      p(java, "controlScoreToPark"),
    ]),
  );

  const settleMs = value(java, "CHASSIS_SETTLE_MS");
  const dwellMs = value(java, "PICKUP_DWELL_MS");
  const fireMs = value(java, "AUTO_FIRE_DURATION_MS");
  const sequence = [
    { kind: "path", index: 0 },
    wait("01 CHASSIS SETTLE", settleMs),
    wait("01 FIRE PRELOAD", fireMs),
  ];
  for (let cycle = 0; cycle < 6; cycle++) {
    const outbound = cycle * 2 + 1;
    const inbound = outbound + 1;
    sequence.push(
      { kind: "path", index: outbound },
      wait(`${String(outbound + 1).padStart(2, "0")} PICKUP DWELL`, dwellMs),
      { kind: "path", index: inbound },
      wait(`${String(inbound + 1).padStart(2, "0")} CHASSIS SETTLE`, settleMs),
      wait(`${String(inbound + 1).padStart(2, "0")} FIRE`, fireMs),
    );
  }
  sequence.push({ kind: "path", index: 13 });
  return { start: p(java, "startPose"), segments, sequence };
}

function routeSieteRed(java) {
  const segments = [
    segment("01 PRELOAD -> SCORE", p(java, "startPose"), p(java, "preloadScorePose")),
    segment(
      "02 SCORE -> PICKUP A1",
      p(java, "preloadScorePose"),
      p(java, "pickupA1Pose"),
      [p(java, "control2ScoreToA1")],
    ),
    segment(
      "03 PICKUP A1 -> SCORE 1",
      p(java, "pickupA1Pose"),
      p(java, "scoreAfterA1Pose"),
      [p(java, "controlA1ToScore")],
    ),
    segment(
      "04 SCORE 1 -> PREPICKUP B1",
      p(java, "scoreAfterA1Pose"),
      p(java, "prePickupB1Pose"),
    ),
    segment(
      "05 PREPICKUP B1 -> PICKUP B1",
      p(java, "prePickupB1Pose"),
      p(java, "pickupB1Pose"),
      [p(java, "controlPrePickupB1ToPickupB1")],
    ),
    segment(
      "06 PICKUP B1 -> SCORE 2",
      p(java, "pickupB1Pose"),
      p(java, "scoreAfterB1Pose"),
      [p(java, "controlB1ToScore")],
    ),
    segment(
      "07 SCORE 2 -> PICKUP A2",
      p(java, "scoreAfterB1Pose"),
      p(java, "pickupA2Pose"),
      [p(java, "controlScoreToA2"), p(java, "control2ScoreToA2")],
    ),
    segment(
      "08 PICKUP A2 -> SCORE 3",
      p(java, "pickupA2Pose"),
      p(java, "scoreAfterA2Pose"),
      [p(java, "controlA2ToScore")],
    ),
    segment(
      "09 SCORE 3 -> PREPICKUP B2",
      p(java, "scoreAfterA2Pose"),
      p(java, "prePickupB2Pose"),
    ),
    segment(
      "10 PREPICKUP B2 -> PICKUP B2",
      p(java, "prePickupB2Pose"),
      p(java, "pickupB2Pose"),
      [p(java, "controlPrePickupB2ToPickupB2")],
    ),
    segment(
      "11 PICKUP B2 -> SCORE 4",
      p(java, "pickupB2Pose"),
      p(java, "scoreAfterB2Pose"),
      [p(java, "controlB2ToScore")],
    ),
    segment(
      "12 SCORE 4 -> PICKUP A3",
      p(java, "scoreAfterB2Pose"),
      p(java, "pickupA3Pose"),
      [p(java, "controlScoreToA3"), p(java, "control2ScoreToA3")],
    ),
    segment(
      "13 PICKUP A3 -> SCORE 5",
      p(java, "pickupA3Pose"),
      p(java, "scoreAfterA3Pose"),
      [p(java, "controlA3ToScore")],
    ),
    segment(
      "14 SCORE 5 -> PREPICKUP B3",
      p(java, "scoreAfterA3Pose"),
      p(java, "prePickupB3Pose"),
    ),
    segment(
      "15 PREPICKUP B3 -> PICKUP B3",
      p(java, "prePickupB3Pose"),
      p(java, "pickupB3Pose"),
      [p(java, "controlPrePickupB3ToPickupB3")],
    ),
    segment(
      "16 PICKUP B3 -> SCORE 6",
      p(java, "pickupB3Pose"),
      p(java, "scoreAfterB3Pose"),
      [p(java, "controlB3ToScore")],
    ),
    segment(
      "17 SCORE 6 -> PARK",
      p(java, "scoreAfterB3Pose"),
      p(java, "parkPose"),
      [p(java, "controlScoreToPark")],
    ),
  ];

  const fireMs = value(java, "REAR_SHOOT_MS");
  const sequence = [
    { kind: "path", index: 0 },
    wait("01 PRELOAD SETTLE", value(java, "PRELOAD_CHASSIS_SETTLE_MS")),
    wait("01 FIRE PRELOAD", fireMs),
    { kind: "path", index: 1 },
    wait("02 PICKUP A1 DWELL", value(java, "A1_PICKUP_DWELL_MS")),
    { kind: "path", index: 2 },
    wait("03 A1 SCORE SETTLE", value(java, "A1_CHASSIS_SETTLE_MS")),
    wait("03 FIRE SCORE 1", fireMs),
    { kind: "path", index: 3 },
    { kind: "path", index: 4 },
    wait("05 PICKUP B1 DWELL", value(java, "B1_PICKUP_DWELL_MS")),
    { kind: "path", index: 5 },
    wait("06 B1 SCORE SETTLE", value(java, "B1_CHASSIS_SETTLE_MS")),
    wait("06 FIRE SCORE 2", fireMs),
    { kind: "path", index: 6 },
    wait("07 PICKUP A2 DWELL", value(java, "A2_PICKUP_DWELL_MS")),
    { kind: "path", index: 7 },
    wait("08 A2 SCORE SETTLE", value(java, "A2_CHASSIS_SETTLE_MS")),
    wait("08 FIRE SCORE 3", fireMs),
    { kind: "path", index: 8 },
    { kind: "path", index: 9 },
    wait("10 PICKUP B2 DWELL", value(java, "B2_PICKUP_DWELL_MS")),
    { kind: "path", index: 10 },
    wait("11 B2 SCORE SETTLE", value(java, "B2_CHASSIS_SETTLE_MS")),
    wait("11 FIRE SCORE 4", fireMs),
    { kind: "path", index: 11 },
    wait("12 PICKUP A3 DWELL", value(java, "A3_PICKUP_DWELL_MS")),
    { kind: "path", index: 12 },
    wait("13 A3 SCORE SETTLE", value(java, "A3_CHASSIS_SETTLE_MS")),
    wait("13 FIRE SCORE 5", fireMs),
    { kind: "path", index: 13 },
    { kind: "path", index: 14 },
    wait("15 PICKUP B3 DWELL", value(java, "B3_PICKUP_DWELL_MS")),
    { kind: "path", index: 15 },
    wait("16 B3 SCORE SETTLE", value(java, "B3_CHASSIS_SETTLE_MS")),
    wait("16 FIRE SCORE 6", fireMs),
    { kind: "path", index: 16 },
  ];
  return { start: p(java, "startPose"), segments, sequence };
}

function routeOchoRed(java) {
  const segments = [
    segment("01 PRELOAD -> SCORE", p(java, "startPose"), p(java, "preloadScorePose")),
    segment(
      "02 SCORE -> PICKUP A1",
      p(java, "preloadScorePose"),
      p(java, "pickupA1Pose"),
      [p(java, "control2ScoreToA1")],
    ),
    segment(
      "03 PICKUP A1 -> SCORE 1",
      p(java, "pickupA1Pose"),
      p(java, "scoreAfterA1Pose"),
      [p(java, "controlA1ToScore")],
    ),
    segment(
      "04 SCORE 1 -> FINAL ROW READY",
      p(java, "scoreAfterA1Pose"),
      p(java, "p07RowThreeReady"),
    ),
    segment(
      "05 COLLECT FINAL ROW",
      p(java, "p07RowThreeReady"),
      p(java, "p08RowThreePickup"),
    ),
    segment(
      "06 FINAL ROW -> SCORE",
      p(java, "p08RowThreePickup"),
      p(java, "scoreAfterA1Pose"),
    ),
    segment(
      "07 SCORE -> PREPICKUP B1",
      p(java, "scoreAfterA1Pose"),
      p(java, "prePickupB1Pose"),
    ),
    segment(
      "08 PREPICKUP B1 -> PICKUP B1",
      p(java, "prePickupB1Pose"),
      p(java, "pickupB1Pose"),
      [p(java, "controlPrePickupB1ToPickupB1")],
    ),
    segment(
      "09 PICKUP B1 -> SCORE 2",
      p(java, "pickupB1Pose"),
      p(java, "scoreAfterB1Pose"),
      [p(java, "controlB1ToScore")],
    ),
    segment(
      "10 SCORE 2 -> PICKUP A2",
      p(java, "scoreAfterB1Pose"),
      p(java, "pickupA2Pose"),
    ),
    segment(
      "11 PICKUP A2 -> SCORE 3",
      p(java, "pickupA2Pose"),
      p(java, "scoreAfterA2Pose"),
      [p(java, "controlA2ToScore")],
    ),
    segment(
      "12 SCORE 3 -> PREPICKUP B2",
      p(java, "scoreAfterA2Pose"),
      p(java, "prePickupB2Pose"),
    ),
    segment(
      "13 PREPICKUP B2 -> PICKUP B2",
      p(java, "prePickupB2Pose"),
      p(java, "pickupB2Pose"),
      [p(java, "controlPrePickupB2ToPickupB2")],
    ),
    segment(
      "14 PICKUP B2 -> SCORE 4",
      p(java, "pickupB2Pose"),
      p(java, "scoreAfterB2Pose"),
      [p(java, "controlB2ToScore")],
    ),
    segment(
      "15 SCORE 4 -> PICKUP A3",
      p(java, "scoreAfterB2Pose"),
      p(java, "pickupA3Pose"),
    ),
    segment(
      "16 PICKUP A3 -> SCORE 5",
      p(java, "pickupA3Pose"),
      p(java, "scoreAfterA3Pose"),
      [p(java, "controlA3ToScore")],
    ),
    segment(
      "17 SCORE 5 -> PREPICKUP B3",
      p(java, "scoreAfterA3Pose"),
      p(java, "prePickupB3Pose"),
    ),
    segment(
      "18 PREPICKUP B3 -> PICKUP B3",
      p(java, "prePickupB3Pose"),
      p(java, "pickupB3Pose"),
      [p(java, "controlPrePickupB3ToPickupB3")],
    ),
    segment(
      "19 PICKUP B3 -> SCORE 6",
      p(java, "pickupB3Pose"),
      p(java, "scoreAfterB3Pose"),
      [p(java, "controlB3ToScore")],
    ),
    segment(
      "20 SCORE 6 -> PARK",
      p(java, "scoreAfterB3Pose"),
      p(java, "parkPose"),
      [p(java, "controlScoreToPark")],
    ),
  ];

  const fireMs = value(java, "REAR_SHOOT_MS");
  const sequence = [
    { kind: "path", index: 0 },
    wait("01 PRELOAD SETTLE", value(java, "PRELOAD_CHASSIS_SETTLE_MS")),
    wait("01 FIRE PRELOAD", fireMs),
    { kind: "path", index: 1 },
    wait("02 PICKUP A1 DWELL", value(java, "A1_PICKUP_DWELL_MS")),
    { kind: "path", index: 2 },
    wait("03 A1 SCORE SETTLE", value(java, "A1_CHASSIS_SETTLE_MS")),
    wait("03 FIRE SCORE 1", fireMs),
    { kind: "path", index: 3 },
    { kind: "path", index: 4 },
    wait("05 FINAL ROW PICKUP DWELL", value(java, "FINAL_ROW_PICKUP_DWELL_MS")),
    { kind: "path", index: 5 },
    wait("06 FINAL ROW SCORE SETTLE", value(java, "FINAL_ROW_CHASSIS_SETTLE_MS")),
    wait("06 FIRE FINAL ROW", fireMs),
    { kind: "path", index: 6 },
    { kind: "path", index: 7 },
    wait("08 PICKUP B1 DWELL", value(java, "B1_PICKUP_DWELL_MS")),
    { kind: "path", index: 8 },
    wait("09 B1 SCORE SETTLE", value(java, "B1_CHASSIS_SETTLE_MS")),
    wait("09 FIRE SCORE 2", fireMs),
    // Java state 13 jumps to 19, so A2 paths 10/11 are not executed.
    { kind: "path", index: 11 },
    { kind: "path", index: 12 },
    wait("13 PICKUP B2 DWELL", value(java, "B2_PICKUP_DWELL_MS")),
    { kind: "path", index: 13 },
    wait("14 B2 SCORE SETTLE", value(java, "B2_CHASSIS_SETTLE_MS")),
    wait("14 FIRE SCORE 4", fireMs),
    // Java state 24 jumps to 30, so A3 paths 15/16 are not executed.
    { kind: "path", index: 16 },
    { kind: "path", index: 17 },
    wait("18 PICKUP B3 DWELL", value(java, "B3_PICKUP_DWELL_MS")),
    { kind: "path", index: 18 },
    wait("19 B3 SCORE SETTLE", value(java, "B3_CHASSIS_SETTLE_MS")),
    wait("19 FIRE SCORE 6", fireMs),
    { kind: "path", index: 19 },
  ];
  return { start: p(java, "startPose"), segments, sequence };
}

function slug(text) {
  return text
    .toLowerCase()
    .replaceAll(/[^a-z0-9]+/g, "-")
    .replaceAll(/^-|-$/g, "");
}

function splitTopLevel(text) {
  const parts = [];
  let start = 0;
  let depth = 0;
  let quoted = false;
  for (let index = 0; index < text.length; index++) {
    const character = text[index];
    if (character === '"' && text[index - 1] !== "\\") {
      quoted = !quoted;
    } else if (!quoted && character === "(") {
      depth++;
    } else if (!quoted && character === ")") {
      depth--;
    } else if (!quoted && character === "," && depth === 0) {
      parts.push(text.slice(start, index).trim());
      start = index + 1;
    }
  }
  parts.push(text.slice(start).trim());
  return parts;
}

function extractCall(text) {
  const match = text.trim().match(/^(\w+)\s*\(([\s\S]*)\)$/);
  if (!match) throw new Error(`Cannot parse path expression: ${text}`);
  return { helper: match[1], args: splitTopLevel(match[2]) };
}

function parseJavaStringExpression(expression) {
  const parts = expression
    .split("+")
    .map((part) => part.trim())
    .filter(Boolean);
  if (parts.length === 0 || parts.some((part) => !/^"(?:[^"\\]|\\.)*"$/.test(part))) {
    throw new Error(`Cannot parse Java string expression: ${expression}`);
  }
  return parts.map((part) => JSON.parse(part)).join("");
}

function readArguments(body, callAt, marker, java) {
  const argumentsStart = callAt + marker.length;
  let depth = 1;
  let quoted = false;
  let cursor = argumentsStart;
  for (; cursor < body.length && depth > 0; cursor++) {
    const character = body[cursor];
    if (character === '"' && body[cursor - 1] !== "\\") {
      quoted = !quoted;
    } else if (!quoted && character === "(") {
      depth++;
    } else if (!quoted && character === ")") {
      depth--;
    }
  }
  if (depth !== 0) throw new Error(`Unclosed ${marker} in ${java.relativeFile}`);
  return {
    args: splitTopLevel(body.slice(argumentsStart, cursor - 1)),
    cursor,
  };
}

function segmentFromPathCall(java, name, pathCall, targetName) {
  const poseArgs = pathCall.args.filter((argument) => argument !== "follower");
  const startName = poseArgs[0];
  const endName = poseArgs.at(-1);
  if (endName !== targetName) {
    throw new Error(
      `${java.relativeFile} ${name}: path ends at ${endName}, target is ${targetName}`,
    );
  }
  const controlNames = poseArgs.slice(1, -1);
  const lineHelpers = new Set([
    "line",
    "linearChain",
    "tangentChain",
    "noDecelerationLinearChain",
  ]);
  const curveHelpers = new Set([
    "curve",
    "curveChain",
    "noDecelerationCurveChain",
  ]);
  if (lineHelpers.has(pathCall.helper) && controlNames.length !== 0) {
    throw new Error(`${java.relativeFile} ${name}: line helper has controls`);
  }
  if (curveHelpers.has(pathCall.helper) && controlNames.length === 0) {
    throw new Error(`${java.relativeFile} ${name}: curve helper has no controls`);
  }
  if (!lineHelpers.has(pathCall.helper) && !curveHelpers.has(pathCall.helper)) {
    throw new Error(`Unknown path helper ${pathCall.helper} in ${java.relativeFile}`);
  }
  return segment(
    name,
    p(java, startName),
    p(java, endName),
    controlNames.map((controlName) => p(java, controlName)),
  );
}

function extractSequenceSegments(java) {
  const methodStart = java.source.indexOf("private void buildSequence()");
  const methodEnd = java.source.indexOf("private void finishAuto()", methodStart);
  if (methodStart < 0 || methodEnd < 0) return null;

  const body = java.source.slice(methodStart, methodEnd);
  const segments = [];
  let searchAt = 0;
  while (true) {
    const followAt = body.indexOf(".follow(", searchAt);
    if (followAt < 0) break;
    const result = readArguments(body, followAt, ".follow(", java);
    if (result.args.length !== 4) {
      throw new Error(`Expected four follow() arguments in ${java.relativeFile}`);
    }
    segments.push(
      segmentFromPathCall(
        java,
        parseJavaStringExpression(result.args[0]),
        extractCall(result.args[1]),
        result.args[2],
      ),
    );
    searchAt = result.cursor;
  }
  return segments;
}

function extractMethodBody(java, methodName) {
  const signature = new RegExp(`\\bvoid\\s+${methodName}\\s*\\(\\s*\\)\\s*\\{`);
  const match = signature.exec(java.source);
  if (!match) return null;

  const bodyStart = match.index + match[0].length;
  let depth = 1;
  let quoted = false;
  let cursor = bodyStart;
  for (; cursor < java.source.length && depth > 0; cursor++) {
    const character = java.source[cursor];
    if (character === '"' && java.source[cursor - 1] !== "\\\\") {
      quoted = !quoted;
    } else if (!quoted && character === "{") {
      depth++;
    } else if (!quoted && character === "}") {
      depth--;
    }
  }
  if (depth !== 0) {
    throw new Error(`Unclosed ${methodName}() in ${java.relativeFile}`);
  }
  return java.source.slice(bodyStart, cursor - 1);
}

function extractPathStateSegments(java) {
  const pathsBody = extractMethodBody(java, "buildPaths");
  const actionBody = extractMethodBody(java, "autonomousPathUpdate");
  if (pathsBody === null || actionBody === null) {
    throw new Error(
      `Missing buildSequence() or buildPaths()/autonomousPathUpdate() in ${java.relativeFile}`,
    );
  }
  const paths = new Map();
  const assignmentPattern = /(\w+)\s*=\s*(\w+)\s*\(/g;
  for (const match of pathsBody.matchAll(assignmentPattern)) {
    const callAt = match.index + match[0].lastIndexOf(`${match[2]}(`);
    const result = readArguments(pathsBody, callAt, `${match[2]}(`, java);
    paths.set(
      match[1],
      extractCall(`${match[2]}(${pathsBody.slice(
        callAt + match[2].length + 1,
        result.cursor - 1,
      )})`),
    );
  }

  const segments = [];
  let searchAt = 0;
  while (true) {
    const followAt = actionBody.indexOf("follow(", searchAt);
    if (followAt < 0) break;
    if (actionBody[followAt - 1] === ".") {
      searchAt = followAt + "follow(".length;
      continue;
    }
    const result = readArguments(actionBody, followAt, "follow(", java);
    if (result.args.length !== 4) {
      throw new Error(`Expected four follow() arguments in ${java.relativeFile}`);
    }
    const pathField = result.args[0];
    const pathCall = paths.get(pathField);
    if (!pathCall) {
      throw new Error(`Missing buildPaths() assignment for ${pathField}`);
    }
    segments.push(
      segmentFromPathCall(
        java,
        parseJavaStringExpression(result.args[1]),
        pathCall,
        result.args[2],
      ),
    );
    searchAt = result.cursor;
  }
  return segments;
}

function extractJavaSegments(java) {
  return extractSequenceSegments(java) ?? extractPathStateSegments(java);
}

function assertRouteMatchesJava(java, route) {
  const actual = extractJavaSegments(java);
  const compact = (segments) =>
    JSON.stringify(
      segments.map((entry) => ({
        name: entry.name,
        start: entry.start,
        end: entry.end,
        controls: entry.controls,
      })),
    );
  if (compact(actual) !== compact(route.segments)) {
    throw new Error(
      `Generated route does not match Java paths/actions in ${java.relativeFile}`,
    );
  }
}

function buildPp(definition) {
  const settingsSource = fs.existsSync(definition.output)
    ? definition.output
    : path.join(visualizerRoot, definition.template);
  const previous = JSON.parse(fs.readFileSync(settingsSource, "utf8"));
  const prefix = `${definition.alliance.toLowerCase()}-${definition.kind.toLowerCase()}`;
  if (definition.route.segments.length > colors.length) {
    throw new Error(`Not enough unique colors for ${definition.output}`);
  }

  const lineIds = definition.route.segments.map(
    (segmentValue, index) =>
      `${prefix}-${String(index + 1).padStart(2, "0")}-${slug(segmentValue.name)}`,
  );
  const lines = definition.route.segments.map((segmentValue, index) => ({
    id: lineIds[index],
    name: `${definition.alliance} ${segmentValue.name}`,
    endPoint: {
      x: segmentValue.end.x,
      y: segmentValue.end.y,
      heading: "linear",
      startDeg: segmentValue.start.heading,
      endDeg: segmentValue.end.heading,
      locked: false,
    },
    controlPoints: segmentValue.controls.map((control) => ({
      x: control.x,
      y: control.y,
      locked: false,
    })),
    color: colors[index],
    locked: false,
    waitBeforeMs: 0,
    waitAfterMs: 0,
    waitBeforeName: "",
    waitAfterName: "",
  }));

  let waitIndex = 0;
  const sequence = definition.route.sequence.map((entry) => {
    if (entry.kind === "path") {
      return { kind: "path", lineId: lineIds[entry.index] };
    }
    waitIndex++;
    return {
      kind: "wait",
      id: `${prefix}-wait-${String(waitIndex).padStart(2, "0")}-${slug(entry.name)}`,
      name: `${definition.alliance} ${entry.name}`,
      durationMs: entry.durationMs,
      locked: false,
    };
  });

  const pathChains = lines.map((line, index) => ({
    id: `${line.id}-chain`,
    name: line.name,
    color: colors[index],
    lineIds: [line.id],
  }));

  const output = {
    startPoint: {
      x: definition.route.start.x,
      y: definition.route.start.y,
      heading: "constant",
      degrees: definition.route.start.heading,
      locked: false,
    },
    lines,
    shapes: previous.shapes ?? [],
    settings: {
      ...previous.settings,
      showGhostPaths: false,
      showOnionLayers: false,
      onionColor: definition.alliance === "RED" ? "#dc2626" : "#2563eb",
    },
    sequence,
    pathChains,
  };
  fs.writeFileSync(definition.output, `${JSON.stringify(output, null, 2)}\n`);
}

const definitions = [
  {
    alliance: "RED",
    kind: "Cero",
    java: "Red/RedAutoCero.java",
    output: "RED-AutoCero-Close-ZeroDoor-RearShooter.pp",
    route: routeCero,
  },
  {
    alliance: "BLUE",
    kind: "Cero",
    java: "Blue/BlueAutoCero.java",
    output: "BLUE-AutoCero-Close-ZeroDoor-RearShooter.pp",
    route: routeCero,
  },
  {
    alliance: "RED",
    kind: "Cinco",
    java: "Red/RedAutoCinco.java",
    output: "RED-AutoCinco-FrontIntake-RearShooter.pp",
    route: routeCinco,
  },
  {
    alliance: "BLUE",
    kind: "Cinco",
    java: "Blue/BlueAutoCinco.java",
    output: "BLUE-AutoCinco-FrontIntake-RearShooter.pp",
    route: routeCinco,
  },
  {
    alliance: "RED",
    kind: "Dos",
    java: "Red/RedAutoDos.java",
    output: "RED-AutoDos-Far-144-FrontIntake-RearShooter.pp",
    route: routeDos,
  },
  {
    alliance: "BLUE",
    kind: "Dos",
    java: "Blue/BlueAutoDos.java",
    output: "BLUE-AutoDos-Far-144-FrontIntake-RearShooter.pp",
    route: routeDos,
  },
  {
    alliance: "RED",
    kind: "Siete",
    java: "Red/RedAutoSiete.java",
    output: "RED-AutoSiete-Far-SixCycle-RearShooter.pp",
    route: routeSieteRed,
  },
  {
    alliance: "BLUE",
    kind: "Siete",
    java: "Blue/BlueAutoSiete.java",
    output: "BLUE-AutoSiete-Far-SixCycle-RearShooter.pp",
    route: routeSieteBlue,
  },
  {
    alliance: "RED",
    kind: "Ocho",
    java: "Red/RedAutoOcho.java",
    output: "RED-AutoOcho-Far-FinalRow-RearShooter.pp",
    template: "RED-AutoSiete-Far-SixCycle-RearShooter.pp",
    route: routeOchoRed,
  },
];

const requestedClasses = new Set(process.argv.slice(2));
const selectedDefinitions = requestedClasses.size === 0
  ? definitions
  : definitions.filter((definition) =>
      requestedClasses.has(path.basename(definition.java, ".java")),
    );
if (selectedDefinitions.length !== requestedClasses.size) {
  const known = definitions.map((definition) => path.basename(definition.java, ".java"));
  throw new Error(`Unknown autonomous class. Available: ${known.join(", ")}`);
}

for (const definition of selectedDefinitions) {
  const java = parseJava(definition.java);
  const output = path.join(visualizerRoot, definition.output);
  const route =
    definition.route === routeCero
      ? definition.route(java, definition.alliance)
      : definition.route(java);
  assertRouteMatchesJava(java, route);
  buildPp({ ...definition, output, route });
}
