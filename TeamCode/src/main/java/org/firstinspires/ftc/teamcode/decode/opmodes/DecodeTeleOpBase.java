package org.firstinspires.ftc.teamcode.decode.opmodes;

import android.content.Context;
import android.content.SharedPreferences;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.control.AutoAimController;
import org.firstinspires.ftc.teamcode.decode.localization.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.decode.shooting.ShotLookupTable;
import org.firstinspires.ftc.teamcode.decode.shooting.ShotSolution;
import org.firstinspires.ftc.teamcode.decode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.decode.util.MathUtil;
import org.firstinspires.ftc.teamcode.decode.util.RisingEdge;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Shared implementation for the fixed-alliance Red and Blue TeleOps.
 *
 * <p>Driver 1 retains translation while auto aim controls rotation. Manual right-stick rotation
 * cancels auto aim immediately.</p>
 */
public abstract class DecodeTeleOpBase extends OpMode {
    protected enum TurnControl {
        RIGHT_STICK(1.0),
        RIGHT_STICK_80(0.8);

        final double scale;

        TurnControl(double scale) {
            this.scale = scale;
        }
    }

    private static final String AIM_PREFERENCES = "decode_aim_preferences";
    private static final String FAR_OFFSET_KEY_PREFIX = "far_offset_";
    private static final String FAR_VELOCITY_OFFSET_KEY_PREFIX = "far_velocity_offset_";
    private static final double FAR_ADJUSTMENT_MIN_DISTANCE_INCHES = 90;
    private static final double FAR_VELOCITY_STEP_TICKS_PER_SECOND = 50;
    private static final double FAR_VELOCITY_OFFSET_LIMIT_TICKS_PER_SECOND = 500;

    private final Alliance alliance;
    private final TurnControl turnControl;
    private final ShotLookupTable shotTable = new ShotLookupTable(DecodeConfig.SHOT_TABLE);

    private final RisingEdge aimPressed = new RisingEdge();
    private final RisingEdge cancelAimPressed = new RisingEdge();
    private final RisingEdge resetOnePressed = new RisingEdge();
    private final RisingEdge resetTwoPressed = new RisingEdge();
    private final RisingEdge headingResetPressed = new RisingEdge();
    private final RisingEdge shooterTogglePressed = new RisingEdge();
    private final RisingEdge firePressed = new RisingEdge();
    private final RisingEdge offsetLeftPressed = new RisingEdge();
    private final RisingEdge offsetRightPressed = new RisingEdge();
    private final RisingEdge offsetResetPressed = new RisingEdge();
    private final RisingEdge velocityUpPressed = new RisingEdge();
    private final RisingEdge velocityDownPressed = new RisingEdge();

    private Follower follower;
    private IMU imu;
    private AutoAimController autoAim;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SharedPreferences preferences;
    private boolean initializedFromAuto;
    private double farVelocityOffsetTicksPerSecond;
    private String lastNotice = "";

    protected DecodeTeleOpBase(Alliance alliance, TurnControl turnControl) {
        this.alliance = alliance;
        this.turnControl = turnControl;
    }

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        follower = Constants.createFollower(hardwareMap);
        initializedFromAuto = AutoPoseHandoff.hasPoseFor(blackboard, alliance);
        Pose initialPose = AutoPoseHandoff.loadOrFallback(
                blackboard,
                alliance,
                alliance.teleOpFallbackPose());
        follower.setStartingPose(initialPose);
        follower.update();

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, intake);
        autoAim = new AutoAimController(alliance.goal());
        autoAim.setFarOffsetMinDistanceInches(FAR_ADJUSTMENT_MIN_DISTANCE_INCHES);
        PanelsConfigurables.INSTANCE.refreshClass(autoAim);

        preferences = hardwareMap.appContext.getSharedPreferences(
                AIM_PREFERENCES,
                Context.MODE_PRIVATE);
        autoAim.setFarOffsetRadians(
                preferences.getFloat(farOffsetPreferenceKey(), 0));
        farVelocityOffsetTicksPerSecond =
                preferences.getFloat(farVelocityOffsetPreferenceKey(), 0);

        telemetry.addLine("DECODE TeleOp initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting pose source", initializedFromAuto ? "AUTO blackboard" : "fallback TODO");
        telemetry.addData("Starting pose", formatPose(initialPose));
        telemetry.addData("Turn control", turnControl);
        telemetry.addLine("Gamepad2 pose reset: tap A / Y");
        telemetry.addLine(
                "Heading reset: point away from your Driver Station, then tap gamepad1 dpad-left");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // Pedro updates Pinpoint and applies the previous loop's drive vector.
        follower.update();
        Pose pose = follower.getPose();

        if (resetOnePressed.update(gamepad2.a)) {
            follower.setPose(alliance.resetPointOne());
            autoAim.cancel();
            pose = follower.getPose();
            lastNotice = "Pose reset to point 1";
        }
        if (resetTwoPressed.update(gamepad2.y)) {
            follower.setPose(alliance.resetPointTwo());
            autoAim.cancel();
            pose = follower.getPose();
            lastNotice = "Pose reset to point 2";
        }
        if (headingResetPressed.update(gamepad1.dpad_left)) {
            // IMU yaw becomes the driver's local zero. Pedro remains in the shared field frame:
            // away from the red Driver Station is 0, away from blue is 180 degrees.
            imu.resetYaw();
            double allianceForwardHeading = alliance == Alliance.BLUE ? Math.PI : 0;
            follower.setPose(new Pose(pose.getX(), pose.getY(), allianceForwardHeading));
            autoAim.cancel();
            pose = follower.getPose();
            lastNotice = alliance == Alliance.BLUE
                    ? "IMU local zero; Pedro heading reset to 180 deg"
                    : "IMU local zero; Pedro heading reset to 0 deg";
        }

        // Right stick pushed right must rotate the team's physical chassis clockwise.
        double manualTurn = -gamepad1.right_stick_x * turnControl.scale;
        if (Math.abs(manualTurn) > DecodeConfig.DRIVER_TURN_CANCEL_DEADBAND) {
            autoAim.cancel();
        }
        if (aimPressed.update(gamepad1.a)) {
            autoAim.start();
            lastNotice = "Auto aim active";
        }
        if (cancelAimPressed.update(gamepad1.b)) {
            autoAim.cancel();
            lastNotice = "Auto aim cancelled";
        }

        double distance = autoAim.distanceToTarget(pose);
        updateFarAdjustments(distance);

        ShotSolution tableSolution = shotTable.get(distance);
        ShotSolution solution = applyFarVelocityOffset(tableSolution);
        shooter.setSolution(solution);

        if (shooterTogglePressed.update(gamepad1.x)) {
            shooter.toggleEnabled();
        }
        if (firePressed.update(gamepad1.y)) {
            // requestFire also starts the flywheels and queues until both encoders are ready.
            shooter.requestFire();
            lastNotice = "Fire queued";
        }

        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            intake.setManualMode(IntakeSubsystem.Mode.INTAKE);
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            intake.setManualMode(IntakeSubsystem.Mode.OUTTAKE);
        } else {
            intake.setManualMode(IntakeSubsystem.Mode.IDLE);
        }

        double aimTurn = autoAim.update(pose);
        double turn = autoAim.isActive() ? aimTurn : manualTurn;

        // The blue Driver Station faces the field from the opposite end of the red driver.
        // Rotate only the field-centric translation command by 180 degrees for blue.
        double allianceTranslationSign = alliance == Alliance.BLUE ? -1 : 1;

        // false = field-centric in PedroPathing 2.1.2.
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * allianceTranslationSign,
                -gamepad1.left_stick_x * allianceTranslationSign,
                turn,
                false);

        shooter.update();
        intake.update();
        updateTelemetry(pose, solution);
    }

    private void updateFarAdjustments(double distance) {
        boolean farShot = distance >= FAR_ADJUSTMENT_MIN_DISTANCE_INCHES;
        double offset = autoAim.getFarOffsetRadians();
        boolean aimOffsetChanged = false;
        boolean velocityOffsetChanged = false;

        if (offsetLeftPressed.update(gamepad2.dpad_left)) {
            if (farShot) {
                offset -= DecodeConfig.FAR_OFFSET_STEP_RAD;
                aimOffsetChanged = true;
            } else {
                lastNotice = "Far offset ignored: distance is below 90 in";
            }
        }
        if (offsetRightPressed.update(gamepad2.dpad_right)) {
            if (farShot) {
                offset += DecodeConfig.FAR_OFFSET_STEP_RAD;
                aimOffsetChanged = true;
            } else {
                lastNotice = "Far offset ignored: distance is below 90 in";
            }
        }
        if (offsetResetPressed.update(gamepad2.right_stick_button)) {
            offset = 0;
            aimOffsetChanged = true;
        }

        if (velocityUpPressed.update(gamepad2.dpad_up)) {
            if (farShot) {
                farVelocityOffsetTicksPerSecond += FAR_VELOCITY_STEP_TICKS_PER_SECOND;
                velocityOffsetChanged = true;
            } else {
                lastNotice = "Far velocity ignored: distance is below 90 in";
            }
        }
        if (velocityDownPressed.update(gamepad2.dpad_down)) {
            if (farShot) {
                farVelocityOffsetTicksPerSecond -= FAR_VELOCITY_STEP_TICKS_PER_SECOND;
                velocityOffsetChanged = true;
            } else {
                lastNotice = "Far velocity ignored: distance is below 90 in";
            }
        }

        if (aimOffsetChanged) {
            autoAim.setFarOffsetRadians(MathUtil.clamp(
                    offset,
                    -DecodeConfig.FAR_OFFSET_LIMIT_RAD,
                    DecodeConfig.FAR_OFFSET_LIMIT_RAD));
            preferences.edit()
                    .putFloat(
                            farOffsetPreferenceKey(),
                            (float) autoAim.getFarOffsetRadians())
                    .apply();
            lastNotice = "Far offset saved";
        }
        if (velocityOffsetChanged) {
            farVelocityOffsetTicksPerSecond = MathUtil.clamp(
                    farVelocityOffsetTicksPerSecond,
                    -FAR_VELOCITY_OFFSET_LIMIT_TICKS_PER_SECOND,
                    FAR_VELOCITY_OFFSET_LIMIT_TICKS_PER_SECOND);
            preferences.edit()
                    .putFloat(
                            farVelocityOffsetPreferenceKey(),
                            (float) farVelocityOffsetTicksPerSecond)
                    .apply();
            lastNotice = String.format(
                    "Far velocity saved: %+.0f ticks/s",
                    farVelocityOffsetTicksPerSecond);
        }
    }

    private ShotSolution applyFarVelocityOffset(ShotSolution tableSolution) {
        boolean farShot =
                tableSolution.distanceInches() >= FAR_ADJUSTMENT_MIN_DISTANCE_INCHES;
        double velocity = tableSolution.velocityTicksPerSecond();
        if (farShot) {
            velocity = Math.max(0, velocity + farVelocityOffsetTicksPerSecond);
        }
        return new ShotSolution(
                tableSolution.distanceInches(),
                velocity,
                tableSolution.pitchServoPosition());
    }

    private void updateTelemetry(Pose pose, ShotSolution solution) {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turn control", turnControl);
        telemetry.addData("Pose", formatPose(pose));
        telemetry.addData("Control Hub IMU heading", "%.1f deg",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Distance to goal", "%.1f in", solution.distanceInches());
        telemetry.addData("Aim", "%s / error %.1f deg",
                autoAim.isActive() ? (autoAim.isOnTarget() ? "LOCKED" : "TURNING") : "manual",
                Math.toDegrees(autoAim.getHeadingError()));
        telemetry.addData("Target chassis heading", "%.1f deg",
                Math.toDegrees(autoAim.getTargetHeading()));
        telemetry.addData("Saved far offset", "%+.2f deg",
                Math.toDegrees(autoAim.getFarOffsetRadians()));
        telemetry.addData("Saved far velocity", "%+.0f ticks/s",
                farVelocityOffsetTicksPerSecond);
        telemetry.addData("Flywheel target", "%.0f ticks/s",
                solution.velocityTicksPerSecond());
        telemetry.addData("Flywheel actual ShooterR", "%.0f ticks/s",
                shooter.getRightVelocity());
        telemetry.addData("Shared flywheel power", "%.3f", shooter.getAppliedPower());
        telemetry.addData("Shooter", "%s, speed=%s, fire=%s",
                shooter.isEnabled() ? "ON" : "OFF",
                shooter.isAtSpeed() ? "READY" : "NOT READY",
                shooter.getFireState());
        telemetry.addData("Pitch servo", "%.3f", solution.pitchServoPosition());
        telemetry.addData("Notice", lastNotice);
        telemetry.addLine(String.format(
                "G1: left stick drive, right stick turn x%.1f, A aim, B cancel",
                turnControl.scale));
        telemetry.addLine("G1: RB intake, LB outtake, X flywheel, Y fire");
        telemetry.addLine("G1: dpad-left heading reset (face away from own Driver Station)");
        telemetry.addLine("G2: A/Y pose reset point 1/2");
        telemetry.addLine("G2: dpad L/R far aim trim (>=90 in), right-stick-click reset");
        telemetry.addLine("G2: dpad U/D far shooter velocity +/-50 ticks/s (>=90 in)");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            AutoPoseHandoff.save(blackboard, follower.getPose(), alliance);
            follower.setTeleOpDrive(0, 0, 0, true);
            follower.update();
        }
        if (shooter != null) {
            shooter.stop();
        }
        if (intake != null) {
            intake.stop();
        }
    }

    private String farOffsetPreferenceKey() {
        return FAR_OFFSET_KEY_PREFIX + alliance.name();
    }

    private String farVelocityOffsetPreferenceKey() {
        return FAR_VELOCITY_OFFSET_KEY_PREFIX + alliance.name();
    }

    private static String formatPose(Pose pose) {
        return String.format(
                "(%.1f, %.1f, %.1f deg)",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
    }
}
