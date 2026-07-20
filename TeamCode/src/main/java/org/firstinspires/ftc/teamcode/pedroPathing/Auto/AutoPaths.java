package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/** Path construction helpers with no autonomous coordinates or settings. */
public final class AutoPaths {
    private AutoPaths() {
    }

    public static Path line(Pose start, Pose end) {
        Path path = new Path(new BezierLine(start, end));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        return path;
    }

    public static Path curve(Pose start, Pose control, Pose end) {
        Path path = new Path(new BezierCurve(start, control, end));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        return path;
    }

    public static Path curve(Pose start, Pose controlOne, Pose controlTwo, Pose end) {
        Path path = new Path(new BezierCurve(start, controlOne, controlTwo, end));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        return path;
    }

    public static PathChain linearChain(Follower follower, Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public static PathChain tangentChain(Follower follower, Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain noDecelerationLinearChain(
            Follower follower,
            Pose start,
            Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .setNoDeceleration()
                .build();
    }

    public static PathChain noDecelerationCurveChain(
            Follower follower,
            Pose start,
            Pose control,
            Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .setNoDeceleration()
                .build();
    }

    public static PathChain curveChain(
            Follower follower,
            Pose start,
            Pose control,
            Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
