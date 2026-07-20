package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.localization.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.decode.shooting.ShotSolution;
import org.firstinspires.ftc.teamcode.decode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Shared runtime plumbing for the team's editable buildPaths() + pathState autonomous layout.
 *
 * <p>Route geometry and action order deliberately remain in each autonomous OpMode.</p>
 */
public abstract class PathStateAutoBase extends OpMode {
    protected Follower follower;
    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;
    protected int pathState;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private ShotSolution shotSolution;
    private boolean poseSaved;
    private String activePath = "INIT";
    private String notice = "Ready";
    private Pose activeTargetPose;

    protected abstract Alliance alliance();

    protected abstract String autoName();

    protected abstract Pose startingPose();

    protected abstract void buildPaths();

    protected abstract void autonomousPathUpdate();

    protected abstract void prepareInitialShot();

    @Override
    public final void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose());
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, intake);
        buildPaths();
        prepareInitialShot();
        setPathState(0);

        telemetry.addData("Auto", autoName());
        telemetry.addData("Start pose", formatPose(startingPose()));
        telemetry.addLine("Edit poses, buildPaths(), then autonomousPathUpdate().");
        telemetry.update();
    }

    @Override
    public final void start() {
        setPathState(0);
    }

    @Override
    public final void loop() {
        follower.update();
        autonomousPathUpdate();
        shooter.update();
        intake.update();
        updateTelemetry();
    }

    @Override
    public final void stop() {
        finishAuto();
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    protected final void setPathState(int nextState) {
        pathState = nextState;
        stateTimer.reset();
    }

    protected final void prepareShotForPose(Pose scoringPose, ShotParameters parameters) {
        double distance = Math.hypot(
                alliance().goal().getX() - scoringPose.getX(),
                alliance().goal().getY() - scoringPose.getY());
        shotSolution = new ShotSolution(
                distance,
                parameters.velocityTicksPerSecond,
                parameters.pitchServoPosition);
        shooter.setSolution(shotSolution);
        shooter.setEnabled(true);
    }

    protected final void requestShot(ShotParameters parameters, long fireDurationMs) {
        stopIntake();
        prepareShotForPose(follower.getPose(), parameters);
        shooter.requestFire(fireDurationMs);
        activePath = activePath + " / FIRE";
    }

    protected final void startIntake(double maxPower) {
        follower.setMaxPower(maxPower);
        intake.setManualMode(IntakeSubsystem.Mode.INTAKE);
    }

    protected final void stopIntake() {
        intake.setManualMode(IntakeSubsystem.Mode.IDLE);
        follower.setMaxPower(1);
    }

    protected final void follow(Path path, String name, Pose target, boolean holdEnd) {
        activePath = name;
        activeTargetPose = target;
        follower.followPath(path, holdEnd);
    }

    protected final void follow(PathChain path, String name, Pose target, boolean holdEnd) {
        activePath = name;
        activeTargetPose = target;
        follower.followPath(path, holdEnd);
    }

    protected final boolean pathFinishedOrTimedOut() {
        if (!follower.isBusy()) {
            return true;
        }
        if (stateTimer.milliseconds() >= DecodeConfig.AUTO_PATH_TIMEOUT_MS) {
            follower.breakFollowing();
            notice = "Path timeout at state " + pathState;
            return true;
        }
        return false;
    }

    protected final boolean waitFinished(long durationMs) {
        return stateTimer.milliseconds() >= durationMs;
    }

    protected final boolean shotFinishedOrTimedOut() {
        if (shooter.getFireState() == ShooterSubsystem.FireState.IDLE) {
            return true;
        }
        if (shooter.getFireState() == ShooterSubsystem.FireState.WAITING_FOR_SPEED
                && stateTimer.milliseconds() >= DecodeConfig.AUTO_SHOT_TIMEOUT_MS) {
            shooter.cancelFire();
            notice = "Shot skipped: speed timeout";
            return true;
        }
        return false;
    }

    protected final void finishAuto() {
        if (poseSaved || follower == null) {
            return;
        }
        AutoPoseHandoff.save(blackboard, follower.getPose(), alliance());
        poseSaved = true;
        shooter.stop();
        intake.stop();
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Path state", pathState);
        telemetry.addData("Active path", activePath);
        telemetry.addData(
                "Target pose",
                activeTargetPose == null ? "none" : formatPose(activeTargetPose));
        telemetry.addData("Pose", formatPose(follower.getPose()));
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Shooter state", shooter.getFireState());
        if (shotSolution != null) {
            telemetry.addData(
                    "Target / actual (ticks/s)",
                    "%.0f / %.0f",
                    shotSolution.velocityTicksPerSecond(),
                    shooter.getRightVelocity());
            telemetry.addData("Pitch rs", "%.4f", shotSolution.pitchServoPosition());
        }
        telemetry.addData("Pose saved", poseSaved);
        telemetry.addData("Notice", notice);
        telemetry.update();
    }

    protected static String formatPose(Pose pose) {
        return String.format(
                "(%.1f, %.1f, %.1f deg)",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
    }

    protected static Pose pose(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    protected static ShotParameters shot(double velocity, double pitch) {
        return new ShotParameters(velocity, pitch);
    }

    protected static final class ShotParameters {
        public final double velocityTicksPerSecond;
        public final double pitchServoPosition;

        private ShotParameters(double velocityTicksPerSecond, double pitchServoPosition) {
            this.velocityTicksPerSecond = velocityTicksPerSecond;
            this.pitchServoPosition = pitchServoPosition;
        }
    }
}
