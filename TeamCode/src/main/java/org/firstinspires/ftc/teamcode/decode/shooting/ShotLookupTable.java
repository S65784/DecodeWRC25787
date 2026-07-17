package org.firstinspires.ftc.teamcode.decode.shooting;

import java.util.Arrays;
import java.util.Comparator;

/**
 * Piecewise-linear distance lookup for shooter velocity and pitch.
 *
 * <p>The table is copied and sorted in the constructor, so callers cannot accidentally modify it
 * while the OpMode is running.</p>
 */
public final class ShotLookupTable {
    private static final int DISTANCE = 0;
    private static final int VELOCITY = 1;
    private static final int PITCH = 2;

    private final double[][] rows;

    public ShotLookupTable(double[][] sourceRows) {
        if (sourceRows == null || sourceRows.length == 0) {
            throw new IllegalArgumentException("Shot table must contain at least one row");
        }

        rows = new double[sourceRows.length][3];
        for (int i = 0; i < sourceRows.length; i++) {
            if (sourceRows[i] == null || sourceRows[i].length != 3) {
                throw new IllegalArgumentException(
                        "Every shot table row must be {distance, velocity, pitch}");
            }
            rows[i] = Arrays.copyOf(sourceRows[i], 3);
        }

        Arrays.sort(rows, Comparator.comparingDouble(row -> row[DISTANCE]));
        for (int i = 0; i < rows.length; i++) {
            if (!Double.isFinite(rows[i][DISTANCE])
                    || !Double.isFinite(rows[i][VELOCITY])
                    || !Double.isFinite(rows[i][PITCH])) {
                throw new IllegalArgumentException("Shot table values must be finite");
            }
            if (i > 0 && rows[i][DISTANCE] <= rows[i - 1][DISTANCE]) {
                throw new IllegalArgumentException("Shot table distances must be unique");
            }
        }
    }

    public ShotSolution get(double distanceInches) {
        if (!Double.isFinite(distanceInches)) {
            throw new IllegalArgumentException("Distance must be finite");
        }

        if (distanceInches <= rows[0][DISTANCE]) {
            return solution(distanceInches, rows[0]);
        }

        int last = rows.length - 1;
        if (distanceInches >= rows[last][DISTANCE]) {
            return solution(distanceInches, rows[last]);
        }

        for (int upper = 1; upper < rows.length; upper++) {
            if (distanceInches <= rows[upper][DISTANCE]) {
                double[] low = rows[upper - 1];
                double[] high = rows[upper];
                double fraction = (distanceInches - low[DISTANCE])
                        / (high[DISTANCE] - low[DISTANCE]);
                return new ShotSolution(
                        distanceInches,
                        lerp(low[VELOCITY], high[VELOCITY], fraction),
                        lerp(low[PITCH], high[PITCH], fraction));
            }
        }

        // Unreachable because both table boundaries were handled above.
        return solution(distanceInches, rows[last]);
    }

    private static ShotSolution solution(double requestedDistance, double[] row) {
        return new ShotSolution(requestedDistance, row[VELOCITY], row[PITCH]);
    }

    private static double lerp(double low, double high, double fraction) {
        return low + (high - low) * fraction;
    }
}
