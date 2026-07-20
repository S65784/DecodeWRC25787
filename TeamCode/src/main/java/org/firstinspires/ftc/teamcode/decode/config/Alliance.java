package org.firstinspires.ftc.teamcode.decode.config;

import com.pedropathing.geometry.Pose;

/**
 * Pedro coordinates: field is 0..144 inches, +X right, +Y away from the audience.
 */
public enum Alliance {
    RED(
            // TODO: Confirm the preferred horizontal aim point inside the red goal.
            //new Pose(134, 138),
            //127.87787182587668, 135.71825876662638更近上端
//            new Pose(129, 131.78),
            new Pose(130, 135),
            // TODO: Replace reset poses with taped, repeatable field locations.
            new Pose(12.999, 7.328, Math.toRadians(180)),//humanzone
            new Pose(144-16.61, 77.92, Math.toRadians(0)),//neargate
            new Pose(94, 108, Math.toRadians(0))
    ),
    BLUE(
            // TODO: Confirm the preferred horizontal aim point inside the blue goal.
            new Pose(144-128, 133),
            new Pose(144-12.999, 7.328, Math.toRadians(0)),
            new Pose(16.61, 77.92, Math.toRadians(180)),
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
