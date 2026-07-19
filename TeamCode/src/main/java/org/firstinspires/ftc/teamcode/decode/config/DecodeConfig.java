package org.firstinspires.ftc.teamcode.decode.config;

/**
 * All robot-specific values that still need testing are kept in this one file.
 *
 * <p>Values marked TODO are safe placeholders, not competition-ready calibration.</p>
 */
public final class DecodeConfig {
    private DecodeConfig() {
    }

    // Hardware Map names.
    public static final String INTAKE_MOTOR = "Intake";
    public static final String LEFT_SHOOTER_MOTOR = "ShooterL";
    public static final String RIGHT_SHOOTER_MOTOR = "ShooterR";
    public static final String PITCH_SERVO = "rs";
    public static final String GATE_SERVO = "hao";

    // Gate and single pitch servo.
    public static final double GATE_CLOSED_POSITION = 0.63;
    public static final double GATE_OPEN_POSITION = 1.00;
    public static final double PITCH_MIN_POSITION = 0.20; // Most vertical.
    public static final double PITCH_MAX_POSITION = 0.52;

    // The single Intake motor handles collecting, ejecting, and feeding.
    public static final double INTAKE_POWER = -0.80;
    public static final double OUTTAKE_POWER = 0.40;
    public static final double SHOOT_FEED_POWER = -1.00;

    // External velocity PIDF: ShooterR encoder is the feedback source and the same power is sent
    // to ShooterL/ShooterR. Tune these with Motor PIDF Tuner (Panels).
    public static final double SHOOTER_VELOCITY_TOLERANCE_TICKS_PER_SECOND = 50;
    public static final long DEFAULT_FIRE_DURATION_MS = 500;
    public static final double SHOOTER_KP = 1;
    public static final double SHOOTER_KI = 0;
    public static final double SHOOTER_KD = 0;
    public static final double SHOOTER_KF = 0.0004;
    public static final double SHOOTER_INTEGRAL_LIMIT = 0.25;

    // Autonomous sequencing. These preserve the old AutoCinco feed time and pickup speeds while
    // keeping every wait non-blocking. A shot that never reaches speed is skipped at the timeout.
    public static final long AUTO_FIRE_DURATION_MS = 925;
    public static final long AUTO_SHOT_TIMEOUT_MS = 4_000;
    public static final long AUTO_PATH_TIMEOUT_MS = 5_000;
    public static final double AUTO_PICKUP_ONE_MAX_POWER = 0.70;
    public static final double AUTO_PICKUP_TWO_MAX_POWER = 0.40;
    public static final double AUTO_PICKUP_THREE_MAX_POWER = 0.45;
    // Keep the original AutoCinco shot until Alliance goals and SHOT_TABLE are measured.
    // The old 2900 RPM target with a 28 tick/rev encoder equals about 1353 ticks/second.
    public static final boolean AUTO_USE_DISTANCE_SHOT_TABLE = false;
    public static final double AUTO_FIXED_SHOT_VELOCITY_TICKS_PER_SECOND = 1353;
    public static final double AUTO_FIXED_PITCH_POSITION = 0.355;

    // Shooter points toward the back of the chassis.
    public static final double SHOOTER_DIRECTION_FROM_ROBOT_FORWARD_RAD = Math.PI;

    // Chassis heading controller. Output is Pedro's normalized turn input [-1, 1].
    // TODO: Tune AIM_KP and AIM_KD with the robot on carpet.
    public static final double AIM_KP = 1.55;
    public static final double AIM_KD = 0.08;
    public static final double AIM_MAX_TURN = 0.70;
    public static final double AIM_MIN_TURN = 0.07;
    public static final double AIM_TOLERANCE_RAD = Math.toRadians(1.5);
    public static final double DRIVER_TURN_CANCEL_DEADBAND = 0.10;

    public static final double SLOW_DRIVE_SCALE = 0.40;
    public static final double FAR_SHOT_DISTANCE_INCHES = 80;
    public static final double FAR_OFFSET_STEP_RAD = Math.toRadians(0.25);
    public static final double FAR_OFFSET_LIMIT_RAD = Math.toRadians(10);

    /**
     * TODO: Replace these four placeholder rows with measured data.
     *
     * <p>Columns are distance (inch), flywheel velocity (encoder ticks/second), and rs pitch
     * servo position. Values between rows are linearly interpolated; values outside the table are
     * clamped to the nearest row.</p>
     */
    public static final double[][] SHOT_TABLE = {
            // Servo placeholders are the old mirrored rs values, clipped to the new safe range.
//            {36, 1260, 0.520},
//            {60, 1353, 0.355},
//            {84, 1680, 0.330},
//            {108, 2007, 0.200}
            {38.98, 1435, 0.2},
            {62.99, 1410, 0.325},
            {51.18, 1435, 0.3},
            {83.07, 1785, 0.49}
    };
}
