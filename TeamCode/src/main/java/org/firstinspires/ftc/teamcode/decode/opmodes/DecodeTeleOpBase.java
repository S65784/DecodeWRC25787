package org.firstinspires.ftc.teamcode.decode.opmodes;

import android.content.Context;
import android.content.SharedPreferences;

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
 * <p>Driver 1 retains translation while auto aim controls rotation. Moving the right stick cancels
 * auto aim immediately.</p>
 */
public abstract class DecodeTeleOpBase extends OpMode {
    private static final String AIM_PREFERENCES = "decode_aim_preferences";
    private static final String FAR_OFFSET_KEY_PREFIX = "far_offset_";

    private final Alliance alliance;
    private final ShotLookupTable shotTable = new ShotLookupTable(DecodeConfig.SHOT_TABLE);

    private final RisingEdge aimPressed = new RisingEdge();
    private final RisingEdge cancelAimPressed = new RisingEdge();
    private final RisingEdge resetOnePressed = new RisingEdge();
    private final RisingEdge resetTwoPressed = new RisingEdge();
    private final RisingEdge shooterTogglePressed = new RisingEdge();
    private final RisingEdge firePressed = new RisingEdge();
    private final RisingEdge offsetLeftPressed = new RisingEdge();
    private final RisingEdge offsetRightPressed = new RisingEdge();
    private final RisingEdge offsetResetPressed = new RisingEdge();

    private Follower follower;
    private IMU imu;
    private AutoAimController autoAim;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SharedPreferences preferences;
    private boolean initializedFromAuto;
    private String lastNotice = "";

    protected DecodeTeleOpBase(Alliance alliance) {
        this.alliance = alliance;
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

        preferences = hardwareMap.appContext.getSharedPreferences(
                AIM_PREFERENCES,
                Context.MODE_PRIVATE);
        autoAim.setFarOffsetRadians(
                preferences.getFloat(farOffsetPreferenceKey(), 0));

        telemetry.addLine("DECODE TeleOp initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting pose source", initializedFromAuto ? "AUTO blackboard" : "fallback TODO");
        telemetry.addData("Starting pose", formatPose(initialPose));
        telemetry.addLine("Reset pose: hold START and tap X / Y");
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

        if (resetOnePressed.update(gamepad1.start && gamepad1.x)) {
            follower.setPose(alliance.resetPointOne());
            autoAim.cancel();
            pose = follower.getPose();
            lastNotice = "Pose reset to point 1";
        }
        if (resetTwoPressed.update(gamepad1.start && gamepad1.y)) {
            follower.setPose(alliance.resetPointTwo());
            autoAim.cancel();
            pose = follower.getPose();
            lastNotice = "Pose reset to point 2";
        }

        double manualTurn = gamepad1.right_trigger - gamepad1.left_trigger;
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
        updateFarOffset(distance);

        ShotSolution solution = shotTable.get(distance);
        shooter.setSolution(solution);

        if (shooterTogglePressed.update(gamepad2.y)) {
            shooter.toggleEnabled();
        }
        if (firePressed.update(gamepad2.a)) {
            // requestFire also starts the flywheels and queues until both encoders are ready.
            shooter.requestFire();
            lastNotice = "Fire queued";
        }

        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            intake.setManualMode(IntakeSubsystem.Mode.INTAKE);
        } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            intake.setManualMode(IntakeSubsystem.Mode.OUTTAKE);
        } else {
            intake.setManualMode(IntakeSubsystem.Mode.IDLE);
        }

        double aimTurn = autoAim.update(pose);
        double turn = autoAim.isActive() ? aimTurn : manualTurn;
        double driveScale = gamepad1.left_bumper ? DecodeConfig.SLOW_DRIVE_SCALE : 1;

        // false = field-centric in PedroPathing 2.1.2.
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * driveScale,
                gamepad1.left_stick_x * driveScale,
                turn * driveScale,
                false);

        shooter.update();
        intake.update();
        updateTelemetry(pose, solution);
    }

    private void updateFarOffset(double distance) {
        boolean farShot = distance >= DecodeConfig.FAR_SHOT_DISTANCE_INCHES;
        double offset = autoAim.getFarOffsetRadians();
        boolean changed = false;

        if (offsetLeftPressed.update(gamepad2.dpad_left)) {
            if (farShot) {
                offset -= DecodeConfig.FAR_OFFSET_STEP_RAD;
                changed = true;
            } else {
                lastNotice = "Far offset ignored: robot is in near range";
            }
        }
        if (offsetRightPressed.update(gamepad2.dpad_right)) {
            if (farShot) {
                offset += DecodeConfig.FAR_OFFSET_STEP_RAD;
                changed = true;
            } else {
                lastNotice = "Far offset ignored: robot is in near range";
            }
        }
        if (offsetResetPressed.update(gamepad2.left_stick_button)) {
            offset = 0;
            changed = true;
        }

        if (changed) {
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
    }

    private void updateTelemetry(Pose pose, ShotSolution solution) {
        telemetry.addData("Alliance", alliance);
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
        telemetry.addData("Flywheel target", "%.0f ticks/s",
                solution.velocityTicksPerSecond());
        telemetry.addData("Flywheel actual ShooterL", "%.0f ticks/s",
                shooter.getLeftVelocity());
        telemetry.addData("Shared flywheel power", "%.3f", shooter.getAppliedPower());
        telemetry.addData("Shooter", "%s, speed=%s, fire=%s",
                shooter.isEnabled() ? "ON" : "OFF",
                shooter.isAtSpeed() ? "READY" : "NOT READY",
                shooter.getFireState());
        telemetry.addData("Pitch servo", "%.3f", solution.pitchServoPosition());
        telemetry.addData("Notice", lastNotice);
        telemetry.addLine("G1: left stick drive, RT/LT turn, A aim, B cancel");
        telemetry.addLine("G1: LB slow, START+X/Y reset");
        telemetry.addLine("G2: Y flywheel, A fire, RB intake, LB outtake");
        telemetry.addLine("G2: dpad L/R far trim, left-stick-click reset trim");
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

    private static String formatPose(Pose pose) {
        return String.format(
                "(%.1f, %.1f, %.1f deg)",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
    }
}
