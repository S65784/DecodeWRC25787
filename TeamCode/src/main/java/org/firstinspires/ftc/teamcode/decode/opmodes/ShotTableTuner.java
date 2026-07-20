package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.shooting.ShotSolution;
import org.firstinspires.ftc.teamcode.decode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.decode.util.RisingEdge;

/**
 * On-robot calibration utility for producing rows for DecodeConfig.SHOT_TABLE.
 *
 * <p>Measure the robot-to-goal distance with a tape measure, select the same distance here, then
 * adjust velocity and pitch until shots are repeatable.</p>
 */
@Configurable
@TeleOp(name = "DECODE Shot Table Tuner", group = "Calibration")
public final class ShotTableTuner extends OpMode {
    /** Live test-only copy; changing it in Panels does not modify DecodeConfig. */
    public static double TEST_FIRE_DURATION_MS = DecodeConfig.DEFAULT_FIRE_DURATION_MS;

    private final RisingEdge velocityUp = new RisingEdge();
    private final RisingEdge velocityDown = new RisingEdge();
    private final RisingEdge pitchUp = new RisingEdge();
    private final RisingEdge pitchDown = new RisingEdge();
    private final RisingEdge distanceUp = new RisingEdge();
    private final RisingEdge distanceDown = new RisingEdge();
    private final RisingEdge toggleShooter = new RisingEdge();
    private final RisingEdge fire = new RisingEdge();
    private final RisingEdge markRow = new RisingEdge();

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    private double testDistanceInches = 36;
    private double velocityTicksPerSecond = 1260;
    private double pitchPosition = 0.52;
    private String markedRow = "Press START to mark a row";

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, intake);
        TEST_FIRE_DURATION_MS = DecodeConfig.DEFAULT_FIRE_DURATION_MS;
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetry.addLine("Place robot at a measured distance before firing.");
        telemetry.addLine("TEST_FIRE_DURATION_MS is live-tunable in Panels and test-only.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (velocityUp.update(gamepad1.dpad_up)) {
            velocityTicksPerSecond += 25;
        }
        if (velocityDown.update(gamepad1.dpad_down)) {
            velocityTicksPerSecond = Math.max(0, velocityTicksPerSecond - 25);
        }
        if (pitchUp.update(gamepad1.dpad_right)) {
            pitchPosition = Range.clip(
                    pitchPosition + 0.005,
                    DecodeConfig.PITCH_MIN_POSITION,
                    DecodeConfig.PITCH_MAX_POSITION);
        }
        if (pitchDown.update(gamepad1.dpad_left)) {
            pitchPosition = Range.clip(
                    pitchPosition - 0.005,
                    DecodeConfig.PITCH_MIN_POSITION,
                    DecodeConfig.PITCH_MAX_POSITION);
        }
        if (distanceUp.update(gamepad1.b)) {
            testDistanceInches += 6;
        }
        if (distanceDown.update(gamepad1.x)) {
            testDistanceInches = Math.max(0, testDistanceInches - 6);
        }

        shooter.setSolution(new ShotSolution(
                testDistanceInches,
                velocityTicksPerSecond,
                pitchPosition));

        if (toggleShooter.update(gamepad1.y)) {
            shooter.toggleEnabled();
        }
        if (fire.update(gamepad1.a)) {
            long testFireDurationMs = Math.max(1, Math.round(TEST_FIRE_DURATION_MS));
            shooter.requestFire(testFireDurationMs);
        }
        if (markRow.update(gamepad1.start)) {
            markedRow = String.format(
                    "{%.1f, %.0f, %.3f},",
                    testDistanceInches,
                    velocityTicksPerSecond,
                    pitchPosition);
        }

        // Match competition TeleOp intake controls. Shooter feed remains the highest-priority
        // IntakeSubsystem command, so a queued shot overrides these buttons while FEEDING.
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            intake.setManualMode(IntakeSubsystem.Mode.INTAKE);
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            intake.setManualMode(IntakeSubsystem.Mode.OUTTAKE);
        } else {
            intake.setManualMode(IntakeSubsystem.Mode.IDLE);
        }

        shooter.update();
        intake.update();

        telemetry.addData("Distance label", "%.1f in", testDistanceInches);
        telemetry.addData("Target velocity", "%.0f ticks/s", velocityTicksPerSecond);
        telemetry.addData("Pitch position", "%.3f", pitchPosition);
        telemetry.addData("Actual ShooterR", "%.0f ticks/s", shooter.getRightVelocity());
        telemetry.addData("Shared flywheel power", "%.3f", shooter.getAppliedPower());
        telemetry.addData("Speed interlock", shooter.isAtSpeed() ? "READY" : "NOT READY");
        telemetry.addData("Fire state", shooter.getFireState());
        telemetry.addData(
                "Test fire duration",
                "%d ms (Panels; not saved)",
                Math.max(1, Math.round(TEST_FIRE_DURATION_MS)));
        telemetry.addData("COPY THIS ROW", markedRow);
        telemetry.addLine("Dpad up/down: velocity +/-25 ticks/s");
        telemetry.addLine("Dpad right/left: pitch +/-0.005");
        telemetry.addLine("B/X: distance label +/-6 in");
        telemetry.addLine("Y: flywheel on/off, A: queued fire, START: mark row");
        telemetry.addLine("RB: intake -0.8, LB: outtake +0.4, release: stop");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
    }
}
