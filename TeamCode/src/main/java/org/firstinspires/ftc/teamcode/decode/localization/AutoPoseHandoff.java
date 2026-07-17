package org.firstinspires.ftc.teamcode.decode.localization;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;

import java.util.Map;

/**
 * Transfers the final autonomous Pedro pose to TeleOp through the SDK 11 blackboard.
 *
 * <p>The blackboard survives OpMode changes, but intentionally does not survive a Robot
 * Controller restart or a new app download.</p>
 */
public final class AutoPoseHandoff {
    private static final String PREFIX = "decode.pose.";
    private static final String X = PREFIX + "x";
    private static final String Y = PREFIX + "y";
    private static final String HEADING = PREFIX + "heading";
    private static final String ALLIANCE = PREFIX + "alliance";
    private static final String VALID = PREFIX + "valid";

    private AutoPoseHandoff() {
    }

    public static void save(
            Map<String, Object> blackboard,
            Pose pose,
            Alliance alliance) {
        blackboard.put(X, pose.getX());
        blackboard.put(Y, pose.getY());
        blackboard.put(HEADING, pose.getHeading());
        blackboard.put(ALLIANCE, alliance.name());
        blackboard.put(VALID, true);
    }

    public static Pose loadOrFallback(
            Map<String, Object> blackboard,
            Alliance expectedAlliance,
            Pose fallback) {
        try {
            if (!Boolean.TRUE.equals(blackboard.get(VALID))
                    || !expectedAlliance.name().equals(blackboard.get(ALLIANCE))) {
                return fallback;
            }
            return new Pose(
                    ((Number) blackboard.get(X)).doubleValue(),
                    ((Number) blackboard.get(Y)).doubleValue(),
                    ((Number) blackboard.get(HEADING)).doubleValue());
        } catch (RuntimeException invalidData) {
            return fallback;
        }
    }

    public static boolean hasPoseFor(
            Map<String, Object> blackboard,
            Alliance expectedAlliance) {
        return Boolean.TRUE.equals(blackboard.get(VALID))
                && expectedAlliance.name().equals(blackboard.get(ALLIANCE));
    }

    public static void invalidate(Map<String, Object> blackboard) {
        blackboard.put(VALID, false);
    }
}
