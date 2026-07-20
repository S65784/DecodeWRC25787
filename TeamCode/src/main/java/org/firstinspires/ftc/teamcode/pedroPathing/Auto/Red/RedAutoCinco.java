package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;

import static org.firstinspires.ftc.teamcode.pedroPathing.Auto.AutoPaths.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Auto.PathStateAutoBase;

/**
 * RedAutoCero route reordered to Row 1, Row 2, Row 3, with two side-gate pushes.
 *
 * <p>All non-gate coordinates and mechanism parameters come from RedAutoCero.
 * Only c04/p04 and c08/p08 come from the tuned RedAutoCinco gate paths.</p>
 */
@Autonomous(name = "红 AutoCinco（近推门2）", group = "Competition")
public final class RedAutoCinco extends PathStateAutoBase {
    private final Pose p00Start = pose(123.457, 122.478, -135.2839);
    private final Pose p01PreloadScore = pose(91.8, 89.8, 233.841815);
    private final ShotParameters shot01Preload = shot(1369, 0.2474);

    // Row 1 (y = 83) runs first.
    private final Pose p02RowOneReady = pose(90, 83.000, 0);
    private final Pose p03RowOnePickup = pose(125.000, 83.000, 0);

    // Gate 1 coordinates remain unchanged.
    private final Pose c04GateOne = pose(115.20243220665724, 79.23203637064647, 0);
    private final Pose p04GateOne = pose(128.16370967741938, 71.54354838709679, -90);

    private final Pose p05RowOneScore = pose(84.000, 83.000, 227.726311);
    private final ShotParameters shot05RowOne = shot(1408, 0.2782);

    // Row 2 (y = 59) runs second.
    private final Pose c06RowTwoReady = pose(87.68229032258066, 72.26816129032255, 0);
    private final Pose p06RowTwoReady = pose(85.81290322580645, 59.000, 0);
    private final Pose p07RowTwoPickup = pose(131.000, 59.000, 0);

    // Gate 2 coordinates remain unchanged.
    private final Pose c08GateTwo = pose(120.95967783870968, 62.99032209677419, 270);
    private final Pose p08GateTwo = pose(127.09435458064517, 75.19516083870967, -90);

    private final Pose c09RowTwoScore = pose(99.000, 60.000, 0);
    private final Pose p09RowTwoScore = pose(84.000, 83.000, 227.726311);
    private final ShotParameters shot09RowTwo = shot(1408, 0.2782);

    // Row 3: no gate push.
    private final Pose c10RowThreeReady = pose(86.83790322580646, 42.416129032258056, 0);
    private final Pose p10RowThreeReady = pose(83.50161290322582, 36.000, 0);
    private final Pose p11RowThreePickup = pose(132.000, 36.000, 0);
    private final Pose p12FinalScore = pose(84.077, 107.923, 211.067610);
    private final ShotParameters shot12Final = shot(1348, 0.2300);
    private final Pose p13Park = pose(86, 108.9, 0);

    private static final double ROW_TWO_MAX_POWER = 0.85;

    private Path preload;
    private PathChain rowOneReady;
    private PathChain rowOnePickup;
    private PathChain gateOne;
    private PathChain rowOneScore;
    private Path rowTwoReady;
    private Path rowTwoPickup;
    private PathChain gateTwo;
    private PathChain rowTwoScore;
    private PathChain rowThreeReady;
    private PathChain rowThreePickup;
    private PathChain finalScore;
    private PathChain park;

    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected String autoName() { return "RedAutoCinco"; }
    @Override protected Pose startingPose() { return p00Start; }
    @Override protected void prepareInitialShot() {
        prepareShotForPose(p01PreloadScore, shot01Preload);
    }

    @Override
    protected void buildPaths() {
        preload = line(p00Start, p01PreloadScore);

        rowOneReady = noDecelerationLinearChain(
                follower, p01PreloadScore, p02RowOneReady);
        rowOnePickup = tangentChain(follower, p02RowOneReady, p03RowOnePickup);
        gateOne = curveChain(follower, p03RowOnePickup, c04GateOne, p04GateOne);
        rowOneScore = linearChain(follower, p04GateOne, p05RowOneScore);

        rowTwoReady = curve(p05RowOneScore, c06RowTwoReady, p06RowTwoReady);
        rowTwoPickup = line(p06RowTwoReady, p07RowTwoPickup);
        gateTwo = curveChain(follower, p07RowTwoPickup, c08GateTwo, p08GateTwo);
        rowTwoScore = curveChain(
                follower, p08GateTwo, c09RowTwoScore, p09RowTwoScore);

        rowThreeReady = noDecelerationCurveChain(
                follower, p09RowTwoScore, c10RowThreeReady, p10RowThreeReady);
        rowThreePickup = tangentChain(follower, p10RowThreeReady, p11RowThreePickup);
        finalScore = linearChain(follower, p11RowThreePickup, p12FinalScore);
        park = linearChain(follower, p12FinalScore, p13Park);
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                prepareShotForPose(p01PreloadScore, shot01Preload);
                follow(preload, "01 START -> PRELOAD SCORE", p01PreloadScore, true);
                setPathState(1);
                break;
            case 1:
                if (pathFinishedOrTimedOut()) {
                    requestShot(shot01Preload, DecodeConfig.AUTO_NEAR_FIRE_DURATION_MS);
                    setPathState(2);
                }
                break;
            case 2:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(rowOneReady, "02 SCORE -> ROW 1 READY", p02RowOneReady, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathFinishedOrTimedOut()) {
                    follow(rowOnePickup, "03 COLLECT ROW 1", p03RowOnePickup, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    follow(gateOne, "04 PUSH GATE 1", p04GateOne, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathFinishedOrTimedOut()) {
                    prepareShotForPose(p05RowOneScore, shot05RowOne);
                    follow(rowOneScore, "05 GATE 1 -> ROW 1 SCORE", p05RowOneScore, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathFinishedOrTimedOut()) {
                    requestShot(shot05RowOne, DecodeConfig.AUTO_NEAR_FIRE_DURATION_MS);
                    setPathState(7);
                }
                break;
            case 7:
                if (shotFinishedOrTimedOut()) {
                    startIntake(ROW_TWO_MAX_POWER);
                    follow(rowTwoReady, "06 SCORE -> ROW 2 READY", p06RowTwoReady, false);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathFinishedOrTimedOut()) {
                    follow(rowTwoPickup, "07 COLLECT ROW 2", p07RowTwoPickup, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    follow(gateTwo, "08 PUSH GATE 2", p08GateTwo, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathFinishedOrTimedOut()) {
                    prepareShotForPose(p09RowTwoScore, shot09RowTwo);
                    follow(rowTwoScore, "09 GATE 2 -> ROW 2 SCORE", p09RowTwoScore, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathFinishedOrTimedOut()) {
                    requestShot(shot09RowTwo, DecodeConfig.AUTO_NEAR_FIRE_DURATION_MS);
                    setPathState(12);
                }
                break;
            case 12:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(rowThreeReady, "10 SCORE -> ROW 3 READY", p10RowThreeReady, false);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathFinishedOrTimedOut()) {
                    follow(rowThreePickup, "11 COLLECT ROW 3", p11RowThreePickup, false);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    prepareShotForPose(p12FinalScore, shot12Final);
                    follow(finalScore, "12 ROW 3 -> FINAL SCORE", p12FinalScore, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathFinishedOrTimedOut()) {
                    requestShot(shot12Final, DecodeConfig.AUTO_NEAR_FIRE_DURATION_MS);
                    setPathState(16);
                }
                break;
            case 16:
                if (shotFinishedOrTimedOut()) {
                    shooter.setEnabled(false);
                    follow(park, "13 FINAL SCORE -> PARK", p13Park, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathFinishedOrTimedOut()) {
                    finishAuto();
                    setPathState(-1);
                }
                break;
            default:
                break;
        }
    }
}
