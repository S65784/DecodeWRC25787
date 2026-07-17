package org.firstinspires.ftc.teamcode.decode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;

/**
 * Owns the single Intake motor.
 *
 * <p>Shooter feed has highest priority, followed by a timed command, followed by manual control.
 * Timed commands are non-blocking; call {@link #update()} once per OpMode loop.</p>
 */
public final class IntakeSubsystem {
    public enum Mode {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    private final DcMotor intake;
    private Mode manualMode = Mode.IDLE;
    private Mode timedMode = Mode.IDLE;
    private long timedCommandEndsNanos;
    private boolean shootFeedOverride;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, DecodeConfig.INTAKE_MOTOR);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void setManualMode(Mode mode) {
        manualMode = mode;
    }

    /** Starts an intake/outtake action without blocking the driver loop. */
    public void runFor(Mode mode, long durationMs) {
        if (mode == Mode.IDLE || durationMs <= 0) {
            cancelTimedCommand();
            return;
        }
        timedMode = mode;
        timedCommandEndsNanos = System.nanoTime() + durationMs * 1_000_000L;
    }

    public void cancelTimedCommand() {
        timedMode = Mode.IDLE;
        timedCommandEndsNanos = 0;
    }

    public boolean isTimedCommandActive() {
        return timedMode != Mode.IDLE && System.nanoTime() < timedCommandEndsNanos;
    }

    public void beginShootFeed() {
        shootFeedOverride = true;
    }

    public void endShootFeed() {
        shootFeedOverride = false;
    }

    public boolean isShootFeeding() {
        return shootFeedOverride;
    }

    public void update() {
        if (shootFeedOverride) {
            intake.setPower(DecodeConfig.SHOOT_FEED_POWER);
            return;
        }

        if (timedMode != Mode.IDLE) {
            if (System.nanoTime() < timedCommandEndsNanos) {
                applyMode(timedMode);
                return;
            }
            cancelTimedCommand();
        }

        applyMode(manualMode);
    }

    public void stop() {
        shootFeedOverride = false;
        manualMode = Mode.IDLE;
        cancelTimedCommand();
        intake.setPower(0);
    }

    private void applyMode(Mode mode) {
        switch (mode) {
            case INTAKE:
                intake.setPower(DecodeConfig.INTAKE_POWER);
                break;
            case OUTTAKE:
                intake.setPower(DecodeConfig.OUTTAKE_POWER);
                break;
            case IDLE:
            default:
                intake.setPower(0);
                break;
        }
    }
}
