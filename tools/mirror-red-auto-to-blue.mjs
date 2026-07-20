import { readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const projectRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const autoRoot = resolve(
  projectRoot,
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Auto"
);

function parseFixed(value) {
  const match = value.match(/^(-?)(\d+)(?:\.(\d+))?$/);
  if (!match) {
    throw new Error(`Unsupported number: ${value}`);
  }
  const fraction = match[3] ?? "";
  const magnitude = BigInt(`${match[2]}${fraction}`);
  return {
    scaled: match[1] === "-" ? -magnitude : magnitude,
    scale: fraction.length
  };
}

function formatFixed(scaled, scale) {
  const negative = scaled < 0n;
  let digits = (negative ? -scaled : scaled).toString().padStart(scale + 1, "0");
  let value;
  if (scale === 0) {
    value = digits;
  } else {
    const splitAt = digits.length - scale;
    value = `${digits.slice(0, splitAt)}.${digits.slice(splitAt)}`;
    value = value.replace(/\.?0+$/, "");
  }
  if (value === "0") {
    return value;
  }
  return negative ? `-${value}` : value;
}

function subtractFixed(leftValue, rightValue) {
  const left = parseFixed(leftValue);
  const right = parseFixed(rightValue);
  const scale = Math.max(left.scale, right.scale);
  const leftScaled = left.scaled * 10n ** BigInt(scale - left.scale);
  const rightScaled = right.scaled * 10n ** BigInt(scale - right.scale);
  return formatFixed(leftScaled - rightScaled, scale);
}

function mirrorSource(redSource, redClassName) {
  const blueClassName = redClassName.replace(/^Red/, "Blue");
  let blueSource = redSource
    .replace(
      "package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;",
      "package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;"
    )
    .replaceAll("RedAuto", "BlueAuto")
    .replaceAll("Alliance.RED", "Alliance.BLUE")
    .replaceAll('"红 ', '"蓝 ');

  blueSource = blueSource.replace(
    /pose\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)/g,
    (_, x, y, heading) =>
      `pose(${subtractFixed("144", x)}, ${y}, ${subtractFixed("180", heading)})`
  );

  if (blueSource.includes(redClassName) || blueSource.includes("Alliance.RED")) {
    throw new Error(`Incomplete program-name/alliance conversion for ${redClassName}`);
  }
  return { blueClassName, blueSource };
}

const redClassNames = process.argv.slice(2);
if (redClassNames.length === 0) {
  throw new Error(
    "Usage: node tools/mirror-red-auto-to-blue.mjs RedAutoCinco [RedAutoSiete ...]"
  );
}

for (const redClassName of redClassNames) {
  if (!/^RedAuto[A-Za-z0-9_]+$/.test(redClassName)) {
    throw new Error(`Invalid red autonomous class name: ${redClassName}`);
  }
  const redFile = resolve(autoRoot, "Red", `${redClassName}.java`);
  const redSource = readFileSync(redFile, "utf8");
  const { blueClassName, blueSource } = mirrorSource(redSource, redClassName);
  const blueFile = resolve(autoRoot, "Blue", `${blueClassName}.java`);
  writeFileSync(blueFile, blueSource, "utf8");
  console.log(`Mirrored ${redClassName} -> ${blueClassName}`);
}
