import { readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const projectRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const autoRoot = resolve(
  projectRoot,
  "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Auto"
);

// A coordinate may be a decimal literal or +/- arithmetic between literals, so
// pose(140-34, 52.9, 270) mirrors instead of being copied through unchanged.
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
 * Exactly evaluates a literal arithmetic expression such as "140-34" or "12.5 + 3".
 * Only + and - between decimal literals are supported; that is all the autonomous
 * coordinates ever use, and anything else must fail loudly rather than be copied
 * to the blue side unmirrored.
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
 * <p>Without this, an unmatched form such as new Pose(...) or pose(140-34, ...) is
 * silently copied to the blue file with red-side coordinates, which puts blue gate
 * pushes in red territory.</p>
 *
 * <p>X and heading must be plain literals after mirroring because both were rewritten.
 * Y is passed through verbatim so the red file's own formatting survives, and it is
 * only required to be a literal or +/- arithmetic.</p>
 */
function assertEveryPoseMirrored(blueSource, blueClassName) {
  const finder = /\bnew\s+Pose\s*\(|\bpose\s*\(/g;
  const plainNumber = /^\s*-?\d+(?:\.\d+)?\s*$/;
  const literalExpression = new RegExp(String.raw`^\s*${TERM}\s*$`);
  let match;
  while ((match = finder.exec(blueSource)) !== null) {
    const open = blueSource.indexOf("(", match.index);
    let depth = 0;
    let close = open;
    for (; close < blueSource.length; close++) {
      if (blueSource[close] === "(") depth++;
      else if (blueSource[close] === ")" && --depth === 0) break;
    }
    const args = blueSource.slice(open + 1, close).split(",");
    const mirrored =
      match[0].trimStart().startsWith("pose") &&
      args.length === 3 &&
      plainNumber.test(args[0]) &&
      literalExpression.test(args[1]) &&
      plainNumber.test(args[2]);
    if (!mirrored) {
      const line = blueSource.slice(0, match.index).split("\n").length;
      const call = blueSource.slice(match.index, close + 1).replace(/\s+/g, " ");
      throw new Error(
        `${blueClassName}.java:${line} was not mirrored: ${call}\n` +
          "  Every pose in a red autonomous must be pose(x, y, headingDegrees) using " +
          "decimal literals or +/- arithmetic.\n" +
          "  new Pose(...) and Math.toRadians(...) are not mirrored by this script."
      );
    }
    finder.lastIndex = close + 1;
  }
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

  const poseCall = new RegExp(
    String.raw`pose\(\s*(${TERM})\s*,\s*(${TERM})\s*,\s*(${TERM})\s*\)`,
    "g"
  );
  // Y is never mirrored, so it is passed through exactly as the red file wrote it.
  blueSource = blueSource.replace(
    poseCall,
    (_, x, y, heading) =>
      `pose(${subtractFixed("144", x)}, ${y}, ${subtractFixed("180", heading)})`
  );

  if (blueSource.includes(redClassName) || blueSource.includes("Alliance.RED")) {
    throw new Error(`Incomplete program-name/alliance conversion for ${redClassName}`);
  }
  assertEveryPoseMirrored(blueSource, blueClassName);
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
