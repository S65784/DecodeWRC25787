import { readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

// Blue -> Red mirror. This is the exact reverse of mirror-red-auto-to-blue.mjs.
// The field mirror math is symmetric (Red X = 144 - Blue X, Red heading = 180 - Blue
// heading, Y unchanged), so only the direction of the name/alliance replacements
// differs. Use this when an autonomous was authored on the blue side first.

const projectRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const autoRoot = resolve(
  projectRoot,
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Auto"
);

// A coordinate may be a decimal literal or +/- arithmetic between literals, so
// pose(144-127, 72, 180) mirrors instead of being copied through unchanged.
const TERM = String.raw`-?\d+(?:\.\d+)?(?:\s*[-+]\s*\d+(?:\.\d+)?)*`;

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

/**
 * Exactly evaluates a literal arithmetic expression such as "144-127" or "12.5 + 3".
 * Only + and - between decimal literals are supported; that is all the autonomous
 * coordinates ever use, and anything else must fail loudly rather than be copied
 * to the red side unmirrored.
 */
function evaluateFixed(expression) {
  const compact = expression.replace(/\s+/g, "");
  if (!/^[-+]?\d+(?:\.\d+)?(?:[-+]\d+(?:\.\d+)?)*$/.test(compact)) {
    throw new Error(`Unsupported numeric expression: ${expression}`);
  }
  const terms = compact.match(/[-+]?\d+(?:\.\d+)?/g).map((term) =>
    parseFixed(term.startsWith("+") ? term.slice(1) : term)
  );
  const scale = terms.reduce((widest, term) => Math.max(widest, term.scale), 0);
  const scaled = terms.reduce(
    (sum, term) => sum + term.scaled * 10n ** BigInt(scale - term.scale),
    0n
  );
  return { scaled, scale };
}

function subtractFixed(leftValue, rightExpression) {
  const left = parseFixed(leftValue);
  const right = evaluateFixed(rightExpression);
  const scale = Math.max(left.scale, right.scale);
  const leftScaled = left.scaled * 10n ** BigInt(scale - left.scale);
  const rightScaled = right.scaled * 10n ** BigInt(scale - right.scale);
  return formatFixed(leftScaled - rightScaled, scale);
}

/**
 * Fails if any pose survived mirroring untouched.
 *
 * <p>Without this, an unmatched form such as new Pose(...) or pose(144-127, ...) is
 * silently copied to the red file with blue-side coordinates, which puts red gate
 * pushes in blue territory.</p>
 *
 * <p>X and heading must be plain literals after mirroring because both were rewritten.
 * Y is passed through verbatim so the blue file's own formatting survives, and it is
 * only required to be a literal or +/- arithmetic.</p>
 */
function assertEveryPoseMirrored(redSource, redClassName) {
  const finder = /\bnew\s+Pose\s*\(|\bpose\s*\(/g;
  const plainNumber = /^\s*-?\d+(?:\.\d+)?\s*$/;
  const literalExpression = new RegExp(String.raw`^\s*${TERM}\s*$`);
  let match;
  while ((match = finder.exec(redSource)) !== null) {
    const open = redSource.indexOf("(", match.index);
    let depth = 0;
    let close = open;
    for (; close < redSource.length; close++) {
      if (redSource[close] === "(") depth++;
      else if (redSource[close] === ")" && --depth === 0) break;
    }
    const args = redSource.slice(open + 1, close).split(",");
    const mirrored =
      match[0].trimStart().startsWith("pose") &&
      args.length === 3 &&
      plainNumber.test(args[0]) &&
      literalExpression.test(args[1]) &&
      plainNumber.test(args[2]);
    if (!mirrored) {
      const line = redSource.slice(0, match.index).split("\n").length;
      const call = redSource.slice(match.index, close + 1).replace(/\s+/g, " ");
      throw new Error(
        `${redClassName}.java:${line} was not mirrored: ${call}\n` +
          "  Every pose in a blue autonomous must be pose(x, y, headingDegrees) using " +
          "decimal literals or +/- arithmetic.\n" +
          "  new Pose(...) and Math.toRadians(...) are not mirrored by this script."
      );
    }
    finder.lastIndex = close + 1;
  }
}

function mirrorSource(blueSource, blueClassName) {
  const redClassName = blueClassName.replace(/^Blue/, "Red");
  let redSource = blueSource
    .replace(
      "package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;",
      "package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;"
    )
    .replaceAll("BlueAuto", "RedAuto")
    .replaceAll("Alliance.BLUE", "Alliance.RED")
    .replaceAll('"蓝 ', '"红 ');

  const poseCall = new RegExp(
    String.raw`pose\(\s*(${TERM})\s*,\s*(${TERM})\s*,\s*(${TERM})\s*\)`,
    "g"
  );
  // Y is never mirrored, so it is passed through exactly as the blue file wrote it.
  redSource = redSource.replace(
    poseCall,
    (_, x, y, heading) =>
      `pose(${subtractFixed("144", x)}, ${y}, ${subtractFixed("180", heading)})`
  );

  if (redSource.includes(blueClassName) || redSource.includes("Alliance.BLUE")) {
    throw new Error(`Incomplete program-name/alliance conversion for ${blueClassName}`);
  }
  assertEveryPoseMirrored(redSource, redClassName);
  return { redClassName, redSource };
}

const blueClassNames = process.argv.slice(2);
if (blueClassNames.length === 0) {
  throw new Error(
    "Usage: node tools/mirror-blue-auto-to-red.mjs BlueAutoTres [BlueAutoOne ...]"
  );
}

for (const blueClassName of blueClassNames) {
  if (!/^BlueAuto[A-Za-z0-9_]+$/.test(blueClassName)) {
    throw new Error(`Invalid blue autonomous class name: ${blueClassName}`);
  }
  const blueFile = resolve(autoRoot, "Blue", `${blueClassName}.java`);
  const blueSource = readFileSync(blueFile, "utf8");
  const { redClassName, redSource } = mirrorSource(blueSource, blueClassName);
  const redFile = resolve(autoRoot, "Red", `${redClassName}.java`);
  writeFileSync(redFile, redSource, "utf8");
  console.log(`Mirrored ${blueClassName} -> ${redClassName}`);
}
