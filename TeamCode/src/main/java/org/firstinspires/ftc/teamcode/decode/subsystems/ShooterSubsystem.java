package org.firstinspires.ftc.teamcode.decode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.control.SharedFlywheelController;
import org.firstinspires.ftc.teamcode.decode.shooting.ShotSolution;

/**
 * Dual-flywheel shooter with one feedback encoder, automatic speed interlock, pitch control,
 * and gate timing.
 *
 * <p>A fire request waits until the ShooterL encoder is within the configured velocity tolerance.
 * Once ready, the gate opens and the single Intake motor feeds immediately.</p>
 */
public final class ShooterSubsystem {
    public enum FireState {
        IDLE,
        WAITING_FOR_SPEED,
        FEEDING
    }

    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final SharedFlywheelController flywheelController;
    private final Servo pitch;
    private final Servo gate;
    private final IntakeSubsystem intake;

    private FireState fireState = FireState.IDLE;
    private ShotSolution solution = new ShotSolution(0, 0, 0.5);
    private boolean enabled;
    private long feedEndsNanos;
    private long requestedFeedDurationMs = DecodeConfig.DEFAULT_FIRE_DURATION_MS;

    public ShooterSubsystem(HardwareMap hardwareMap, IntakeSubsystem intake) {
        this.intake = intake;
        leftShooter = hardwareMap.get(DcMotorEx.class, DecodeConfig.LEFT_SHOOTER_MOTOR);
        rightShooter = hardwareMap.get(DcMotorEx.class, DecodeConfig.RIGHT_SHOOTER_MOTOR);
        pitch = hardwareMap.get(Servo.class, DecodeConfig.PITCH_SERVO);
        gate = hardwareMap.get(Servo.class, DecodeConfig.GATE_SERVO);

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        configureFlywheel(leftShooter);
        configureFlywheel(rightShooter);
        flywheelController = new SharedFlywheelController(leftShooter, rightShooter);
        flywheelController.setGains(
                DecodeConfig.SHOOTER_KP,
                DecodeConfig.SHOOTER_KI,
                DecodeConfig.SHOOTER_KD,
                DecodeConfig.SHOOTER_KF,
                DecodeConfig.SHOOTER_INTEGRAL_LIMIT);

        closeGate();
        setPitch(solution.pitchServoPosition());
    }

    private static void configureFlywheel(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // External shared-power PIDF is used, so do not also enable the hub's velocity loop.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSolution(ShotSolution solution) {
        this.solution = solution;
        setPitch(solution.pitchServoPosition());
    }

    public ShotSolution getSolution() {
        return solution;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            cancelFire();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void toggleEnabled() {
        setEnabled(!enabled);
    }

    public void requestFire() {
        requestFire(DecodeConfig.DEFAULT_FIRE_DURATION_MS);
    }

    /**
     * Queues a non-blocking shot. If flywheels are not ready, the request remains waiting.
     */
    public void requestFire(long feedDurationMs) {
        requestedFeedDurationMs = Math.max(1, feedDurationMs);
        enabled = true;
        if (fireState == FireState.IDLE) {
            fireState = FireState.WAITING_FOR_SPEED;
        }
    }

    public void cancelFire() {
        fireState = FireState.IDLE;
        closeGate();
        intake.endShootFeed();
    }

    public void update() {
        double targetVelocity = enabled ? solution.velocityTicksPerSecond() : 0;
        flywheelController.setTargetVelocity(targetVelocity);
        flywheelController.update();

        switch (fireState) {
            case WAITING_FOR_SPEED:
                closeGate();
                intake.endShootFeed();
                if (isAtSpeed()) {
                    gate.setPosition(DecodeConfig.GATE_OPEN_POSITION);
                    intake.beginShootFeed();
                    feedEndsNanos =
                            System.nanoTime() + requestedFeedDurationMs * 1_000_000L;
                    fireState = FireState.FEEDING;
                }
                break;
            case FEEDING:
                if (System.nanoTime() >= feedEndsNanos) {
                    closeGate();
                    intake.endShootFeed();
                    fireState = FireState.IDLE;
                }
                break;
            case IDLE:
            default:
                closeGate();
                intake.endShootFeed();
                break;
        }
    }

    public boolean isAtSpeed() {
        if (!enabled || solution.velocityTicksPerSecond() <= 0) {
            return false;
        }
        double target = solution.velocityTicksPerSecond();
        double tolerance = DecodeConfig.SHOOTER_VELOCITY_TOLERANCE_TICKS_PER_SECOND;
        return Math.abs(getLeftVelocity() - target) <= tolerance;
    }

    public double getLeftVelocity() {
        return flywheelController.getMeasuredVelocity();
    }

    /** Diagnostic only: ShooterR has no feedback encoder and is not used by the interlock. */
    public double getRightVelocity() {
        return Math.abs(rightShooter.getVelocity());
    }

    public double getAppliedPower() {
        return flywheelController.getAppliedPower();
    }

    public FireState getFireState() {
        return fireState;
    }

    public void stop() {
        enabled = false;
        cancelFire();
        flywheelController.stop();
    }

    private void setPitch(double position) {
        pitch.setPosition(Range.clip(
                position,
                DecodeConfig.PITCH_MIN_POSITION,
                DecodeConfig.PITCH_MAX_POSITION));
    }

    private void closeGate() {
        gate.setPosition(DecodeConfig.GATE_CLOSED_POSITION);
    }
}
