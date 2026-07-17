package org.firstinspires.ftc.teamcode.decode.shooting;

/** Immutable result of a distance lookup. */
public final class ShotSolution {
    private final double distanceInches;
    private final double velocityTicksPerSecond;
    private final double pitchServoPosition;

    public ShotSolution(
            double distanceInches,
            double velocityTicksPerSecond,
            double pitchServoPosition) {
        this.distanceInches = distanceInches;
        this.velocityTicksPerSecond = velocityTicksPerSecond;
        this.pitchServoPosition = pitchServoPosition;
    }

    public double distanceInches() {
        return distanceInches;
    }

    public double velocityTicksPerSecond() {
        return velocityTicksPerSecond;
    }

    public double pitchServoPosition() {
        return pitchServoPosition;
    }
}
