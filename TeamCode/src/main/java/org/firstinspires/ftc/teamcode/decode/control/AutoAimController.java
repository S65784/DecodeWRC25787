package org.firstinspires.ftc.teamcode.decode.control;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.util.MathUtil;

/**
 * Calculates the chassis heading that points a rear-mounted shooter toward a field target.
 *
 * <p>This controller only produces a turn value; driver translation remains available.</p>
 */
public final class AutoAimController {
    private final Pose target;
    private boolean active;
    private boolean firstUpdate = true;
    private long lastUpdateNanos;
    private double previousError;
    private double farOffsetRadians;
    private double targetHeading;
    private double headingError;
    private double distance;

    public AutoAimController(Pose target) {
        this.target = target;
    }

    public void start() {
        active = true;
        firstUpdate = true;
    }

    public void cancel() {
        active = false;
        firstUpdate = true;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isOnTarget() {
        return active && Math.abs(headingError) <= DecodeConfig.AIM_TOLERANCE_RAD;
    }

    public double update(Pose robotPose) {
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        distance = Math.hypot(dx, dy);

        double goalBearing = Math.atan2(dy, dx);
        double appliedOffset = distance >= DecodeConfig.FAR_SHOT_DISTANCE_INCHES
                ? farOffsetRadians
                : 0;
        targetHeading = MathUtil.normalizeRadians(
                goalBearing
                        - DecodeConfig.SHOOTER_DIRECTION_FROM_ROBOT_FORWARD_RAD
                        + appliedOffset);
        headingError = MathUtil.normalizeRadians(targetHeading - robotPose.getHeading());

        if (!active) {
            return 0;
        }

        long now = System.nanoTime();
        double derivative = 0;
        if (!firstUpdate) {
            double dt = (now - lastUpdateNanos) / 1e9;
            if (dt > 1e-4) {
                derivative = (headingError - previousError) / dt;
            }
        }
        firstUpdate = false;
        lastUpdateNanos = now;
        previousError = headingError;

        if (Math.abs(headingError) <= DecodeConfig.AIM_TOLERANCE_RAD) {
            return 0;
        }

        double output = DecodeConfig.AIM_KP * headingError + DecodeConfig.AIM_KD * derivative;
        output = MathUtil.clamp(output, -DecodeConfig.AIM_MAX_TURN, DecodeConfig.AIM_MAX_TURN);

        if (Math.abs(output) < DecodeConfig.AIM_MIN_TURN) {
            output = Math.copySign(DecodeConfig.AIM_MIN_TURN, headingError);
        }
        return output;
    }

    public double distanceToTarget(Pose robotPose) {
        return Math.hypot(
                target.getX() - robotPose.getX(),
                target.getY() - robotPose.getY());
    }

    public void setFarOffsetRadians(double offsetRadians) {
        farOffsetRadians = MathUtil.clamp(
                offsetRadians,
                -DecodeConfig.FAR_OFFSET_LIMIT_RAD,
                DecodeConfig.FAR_OFFSET_LIMIT_RAD);
    }

    public double getFarOffsetRadians() {
        return farOffsetRadians;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getDistance() {
        return distance;
    }
}
