package org.firstinspires.ftc.teamcode.decode.control;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Uses one encoder as the velocity feedback source and sends one shared power command to two
 * flywheel motors.
 *
 * <p>This keeps both motors on the same command, but it cannot measure or correct a speed
 * difference on the unmonitored motor.</p>
 */
public final class SharedFlywheelController {
    private final DcMotorEx feedbackMotor;
    private final DcMotorEx followerMotor;

    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double integralLimit;

    private double targetVelocity;
    private double measuredVelocity;
    private double error;
    private double appliedPower;
    private double integral;
    private double previousError;
    private long previousTimeNanos;
    private boolean firstUpdate = true;

    public SharedFlywheelController(DcMotorEx feedbackMotor, DcMotorEx followerMotor) {
        this.feedbackMotor = feedbackMotor;
        this.followerMotor = followerMotor;
    }

    public void setGains(
            double kP,
            double kI,
            double kD,
            double kF,
            double integralLimit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralLimit = Math.max(0, integralLimit);
    }

    public void setTargetVelocity(double targetVelocity) {
        double clippedTarget = Math.max(0, targetVelocity);
        if (clippedTarget == 0 && this.targetVelocity != 0) {
            resetState();
        }
        this.targetVelocity = clippedTarget;
    }

    public void update() {
        measuredVelocity = Math.abs(feedbackMotor.getVelocity());

        if (targetVelocity <= 0) {
            appliedPower = 0;
            feedbackMotor.setPower(0);
            followerMotor.setPower(0);
            return;
        }

        long now = System.nanoTime();
        error = targetVelocity - measuredVelocity;
        double derivative = 0;

        if (!firstUpdate) {
            double dt = (now - previousTimeNanos) / 1e9;
            if (dt > 1e-4 && dt < 0.25) {
                if (kI == 0) {
                    integral = 0;
                } else {
                    integral += error * dt;
                    if (integralLimit > 0) {
                        double maxIntegral = integralLimit / Math.abs(kI);
                        integral = Range.clip(integral, -maxIntegral, maxIntegral);
                    }
                }
                derivative = (error - previousError) / dt;
            }
        }

        firstUpdate = false;
        previousTimeNanos = now;
        previousError = error;

        appliedPower = Range.clip(
                kF * targetVelocity
                        + kP * error
                        + kI * integral
                        + kD * derivative,
                0,
                1);
        feedbackMotor.setPower(appliedPower);
        followerMotor.setPower(appliedPower);
    }

    public void stop() {
        targetVelocity = 0;
        resetState();
        feedbackMotor.setPower(0);
        followerMotor.setPower(0);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getMeasuredVelocity() {
        return measuredVelocity;
    }

    public double getError() {
        return error;
    }

    public double getAppliedPower() {
        return appliedPower;
    }

    private void resetState() {
        integral = 0;
        previousError = 0;
        previousTimeNanos = 0;
        firstUpdate = true;
        error = 0;
    }
}
