package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.shooting.ShotSolution;
import org.firstinspires.ftc.teamcode.decode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decode.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Non-blocking declarative autonomous executor.
 *
 * <p>This class contains no route coordinates. Each OpMode owns its poses and declares its complete
 * action order in one {@code buildSequence()} method.</p>
 */
public final class AutoSequence {
    private interface Step {
        String name();

        Pose target();

        void start();

        boolean isComplete();
    }

    private abstract static class NamedStep implements Step {
        private final String name;
        private final Pose target;

        NamedStep(String name, Pose target) {
            this.name = name;
            this.target = target;
        }

        @Override
        public String name() {
            return name;
        }

        @Override
        public Pose target() {
            return target;
        }
    }

    private final Follower follower;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final Alliance alliance;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final List<Step> steps = new ArrayList<>();

    private int stepIndex;
    private boolean running;
    private boolean stepStarted;
    private boolean finished;
    private String activeStep = "INIT";
    private String notice = "Ready";
    private Pose activeTarget;
    private ShotSolution shotSolution;

    public AutoSequence(
            Follower follower,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            Alliance alliance) {
        this.follower = follower;
        this.intake = intake;
        this.shooter = shooter;
        this.alliance = alliance;
    }

    public AutoSequence prepareShot(
            String name,
            Pose scoringPose,
            double velocity,
            double pitch) {
        return action(name, () -> prepareShotNow(scoringPose, velocity, pitch));
    }

    public AutoSequence follow(
            String name,
            Path path,
            Pose target,
            boolean holdEnd) {
        steps.add(new NamedStep(name, target) {
            @Override
            public void start() {
                follower.followPath(path, holdEnd);
            }

            @Override
            public boolean isComplete() {
                return pathFinishedOrTimedOut();
            }
        });
        return this;
    }

    public AutoSequence follow(
            String name,
            PathChain path,
            Pose target,
            boolean holdEnd) {
        steps.add(new NamedStep(name, target) {
            @Override
            public void start() {
                follower.followPath(path, holdEnd);
            }

            @Override
            public boolean isComplete() {
                return pathFinishedOrTimedOut();
            }
        });
        return this;
    }

    public AutoSequence waitMillis(String name, long durationMs) {
        steps.add(new NamedStep(name, null) {
            @Override
            public void start() {
            }

            @Override
            public boolean isComplete() {
                return stepTimer.milliseconds() >= durationMs;
            }
        });
        return this;
    }

    public AutoSequence startIntake(String name, double followerMaxPower) {
        return action(name, () -> {
            follower.setMaxPower(followerMaxPower);
            intake.setManualMode(IntakeSubsystem.Mode.INTAKE);
        });
    }

    public AutoSequence stopIntake(String name) {
        return action(name, () -> {
            intake.setManualMode(IntakeSubsystem.Mode.IDLE);
            follower.setMaxPower(1);
        });
    }

    public AutoSequence setShooterEnabled(String name, boolean enabled) {
        return action(name, () -> shooter.setEnabled(enabled));
    }

    public AutoSequence fire(
            String name,
            double velocity,
            double pitch,
            long fireDurationMs) {
        steps.add(new NamedStep(name, null) {
            @Override
            public void start() {
                intake.setManualMode(IntakeSubsystem.Mode.IDLE);
                follower.setMaxPower(1);
                prepareShotNow(follower.getPose(), velocity, pitch);
                shooter.requestFire(fireDurationMs);
            }

            @Override
            public boolean isComplete() {
                if (shooter.getFireState() == ShooterSubsystem.FireState.IDLE) {
                    return true;
                }
                if (shooter.getFireState() == ShooterSubsystem.FireState.WAITING_FOR_SPEED
                        && stepTimer.milliseconds() >= DecodeConfig.AUTO_SHOT_TIMEOUT_MS) {
                    shooter.cancelFire();
                    notice = "Shot skipped: speed timeout";
                    return true;
                }
                return false;
            }
        });
        return this;
    }

    public AutoSequence action(String name, Runnable action) {
        steps.add(new NamedStep(name, null) {
            @Override
            public void start() {
                action.run();
            }

            @Override
            public boolean isComplete() {
                return true;
            }
        });
        return this;
    }

    public void prepareShotNow(Pose scoringPose, double velocity, double pitch) {
        double distance = Math.hypot(
                alliance.goal().getX() - scoringPose.getX(),
                alliance.goal().getY() - scoringPose.getY());
        shotSolution = new ShotSolution(distance, velocity, pitch);
        shooter.setSolution(shotSolution);
        shooter.setEnabled(true);
    }

    public void start() {
        stepIndex = 0;
        stepStarted = false;
        finished = steps.isEmpty();
        running = !finished;
        notice = finished ? "No steps" : "Running";
    }

    public void update() {
        if (!running || finished) {
            return;
        }

        int instantStepGuard = 0;
        while (stepIndex < steps.size() && instantStepGuard <= steps.size()) {
            Step step = steps.get(stepIndex);
            if (!stepStarted) {
                activeStep = step.name();
                activeTarget = step.target();
                stepTimer.reset();
                step.start();
                stepStarted = true;
            }

            if (!step.isComplete()) {
                return;
            }

            stepIndex++;
            stepStarted = false;
            instantStepGuard++;
        }

        if (stepIndex >= steps.size()) {
            finished = true;
            running = false;
            notice = "Done";
        }
    }

    public void stop() {
        running = false;
        follower.breakFollowing();
        shooter.stop();
        intake.stop();
    }

    public boolean isFinished() {
        return finished;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Sequence step", "%d / %d", Math.min(stepIndex + 1, steps.size()), steps.size());
        telemetry.addData("Active step", activeStep);
        telemetry.addData(
                "Target pose",
                activeTarget == null ? "none" : formatPose(activeTarget));
        telemetry.addData("Pose", formatPose(follower.getPose()));
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Shooter state", shooter.getFireState());
        if (shotSolution != null) {
            telemetry.addData("Shot distance (in)", "%.1f", shotSolution.distanceInches());
            telemetry.addData(
                    "Target / actual (ticks/s)",
                    "%.0f / %.0f",
                    shotSolution.velocityTicksPerSecond(),
                    shooter.getRightVelocity());
            telemetry.addData("Pitch rs", "%.4f", shotSolution.pitchServoPosition());
        }
        telemetry.addData("Shooter power", "%.3f", shooter.getAppliedPower());
        telemetry.addData("Sequence finished", finished);
        telemetry.addData("Notice", notice);
    }

    private boolean pathFinishedOrTimedOut() {
        if (!follower.isBusy()) {
            return true;
        }
        if (stepTimer.milliseconds() >= DecodeConfig.AUTO_PATH_TIMEOUT_MS) {
            follower.breakFollowing();
            notice = "Path timeout: " + activeStep;
            return true;
        }
        return false;
    }

    private static String formatPose(Pose pose) {
        return String.format(
                "(%.1f, %.1f, %.1f deg)",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
    }
}
