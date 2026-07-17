package org.firstinspires.ftc.teamcode.decode.util;

import com.qualcomm.robotcore.util.Range;

public final class MathUtil {
    private MathUtil() {
    }

    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public static double clamp(double value, double min, double max) {
        return Range.clip(value, min, max);
    }
}
