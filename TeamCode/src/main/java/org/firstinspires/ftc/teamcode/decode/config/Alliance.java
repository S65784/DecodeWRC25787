package org.firstinspires.ftc.teamcode.decode.config;

import com.pedropathing.geometry.Pose;

/**
 * Pedro coordinates: field is 0..144 inches, +X right, +Y away from the audience.
 */
public enum Alliance {
    RED(
            // TODO: Confirm the preferred horizontal aim point inside the red goal.
            new Pose(134, 138),
            // TODO: Replace reset poses with taped, repeatable field locations.
            new Pose(94, 108, Math.toRadians(0)),
            new Pose(100, 99.8, Math.toRadians(37.2)),
            new Pose(94, 108, Math.toRadians(0))
    ),
    BLUE(
            // TODO: Confirm the preferred horizontal aim point inside the blue goal.
            new Pose(10, 138),
            new Pose(50, 108, Math.toRadians(180)),
            new Pose(44, 99.8, Math.toRadians(142.8)),
            new Pose(50, 108, Math.toRadians(180))
    );

    private final Pose goal;
    private final Pose resetPointOne;
    private final Pose resetPointTwo;
    private final Pose teleOpFallbackPose;

    Alliance(Pose goal, Pose resetPointOne, Pose resetPointTwo, Pose teleOpFallbackPose) {
        this.goal = goal;
        this.resetPointOne = resetPointOne;
        this.resetPointTwo = resetPointTwo;
        this.teleOpFallbackPose = teleOpFallbackPose;
    }

    public Pose goal() {
        return goal;
    }

    public Pose resetPointOne() {
        return resetPointOne;
    }

    public Pose resetPointTwo() {
        return resetPointTwo;
    }

    public Pose teleOpFallbackPose() {
        return teleOpFallbackPose;
    }
}
