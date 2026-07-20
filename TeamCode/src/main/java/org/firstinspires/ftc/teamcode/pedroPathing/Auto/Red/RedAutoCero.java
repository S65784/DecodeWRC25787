package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;

import static org.firstinspires.ftc.teamcode.pedroPathing.Auto.AutoPaths.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Auto.PathStateAutoBase;

/** Red close-side autonomous. Geometry and actions are fully editable in this file. */
@Autonomous(name = "红 AutoCero 近端不开门", group = "Competition")
public final class RedAutoCero extends PathStateAutoBase {
    private final Pose p00Start = pose(123.457, 122.478, -135.2839);
    private final Pose p01PreloadScore = pose(91.8, 89.8, 233.841815);
    private final ShotParameters shot01Preload = shot(1369, 0.2474);

    private final Pose c12RowTwoReady = pose(87.68229032258066, 72.26816129032255, 0);
    private final Pose p12RowTwoReady = pose(85.81290322580645, 59.000, 0);
    private final Pose c02RowTwoPickup = pose(81.5201935483871, 55.60767741935483, 0);
    private final Pose cc02RowTwoPickup = pose(81.71653225806452, 57.41008064516127, 0);
    private final Pose p02RowTwoPickup = pose(131.000, 59.000, 0);
    private final Pose c03RowTwoScore = pose(99.000, 60.000, 0);
    private final Pose p03RowTwoScore = pose(84.000, 83.000, 227.726311);
    private final ShotParameters shot03RowTwo = shot(1408, 0.2782);

    private final Pose p04RowOneReady = pose(90, 83.000, 0);
    private final Pose p05RowOnePickup = pose(125.000, 83.000, 0);
    private final Pose p06RowOneScore = pose(84.000, 83.000, 227.726311);
    private final ShotParameters shot06RowOne = shot(1408, 0.2782);

    private final Pose c07RowThreeReady = pose(89, 42.416129032258056, 0);
    private final Pose p07RowThreeReady = pose(89, 36.000, 0);
    private final Pose p08RowThreePickup = pose(132.000, 36.000, 0);
    private final Pose p09FinalScore = pose(84.077, 107.923, 211.067610);
    private final ShotParameters shot09Final = shot(1348, 0.2300);
    private final Pose parkPose = pose(86, 108.9, 0);

    private static final double ROW_TWO_MAX_POWER = 0.85;

    private Path preload;
    private Path rowTwoReady;
    private Path rowTwoPickup;
    private Path rowTwoScore;
    private PathChain rowOneReady;
    private PathChain collectRowOne;
    private PathChain rowOneScore;
    private PathChain rowThreeReady;
    private PathChain collectRowThree;
    private PathChain finalScore;
    private PathChain scoreToPark;

    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected String autoName() { return "RedAutoCero"; }
    @Override protected Pose startingPose() { return p00Start; }
    @Override protected void prepareInitialShot() {
        prepareShotForPose(p01PreloadScore, shot01Preload);
    }

    @Override
    protected void buildPaths() {
        preload = line(p00Start, p01PreloadScore);
        rowTwoReady = curve(p01PreloadScore, c12RowTwoReady, p12RowTwoReady);
        rowTwoPickup = line(p12RowTwoReady, p02RowTwoPickup);
        rowTwoScore = curve(p02RowTwoPickup, c03RowTwoScore, p03RowTwoScore);
        rowOneReady = noDecelerationLinearChain(follower, p03RowTwoScore, p04RowOneReady);
        collectRowOne = tangentChain(follower, p04RowOneReady, p05RowOnePickup);
        rowOneScore = linearChain(follower, p05RowOnePickup, p06RowOneScore);
        rowThreeReady = noDecelerationCurveChain(
                follower, p06RowOneScore, c07RowThreeReady, p07RowThreeReady);
        collectRowThree = tangentChain(follower, p07RowThreeReady, p08RowThreePickup);
        finalScore = linearChain(follower, p08RowThreePickup, p09FinalScore);
        scoreToPark = linearChain(follower, p09FinalScore, parkPose);
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
                    setPathState(120);
                }
                break;
            case 120:
                if (shotFinishedOrTimedOut()) {
                    startIntake(ROW_TWO_MAX_POWER);
                    follow(rowTwoReady, "02 SCORE -> ROW 2 READY", p12RowTwoReady, false);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathFinishedOrTimedOut()) {
                    follow(rowTwoPickup, "03 COLLECT ROW 2", p02RowTwoPickup, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    prepareShotForPose(p03RowTwoScore, shot03RowTwo);
                    follow(rowTwoScore, "04 ROW 2 -> SCORE", p03RowTwoScore, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathFinishedOrTimedOut()) {
                    requestShot(shot03RowTwo, DecodeConfig.AUTO_NEAR_FIRE_DURATION_MS);
                    setPathState(5);
                }
                break;
            case 5:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(rowOneReady, "05 SCORE -> ROW 1 READY", p04RowOneReady, false);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathFinishedOrTimedOut()) {
                    follow(collectRowOne, "06 COLLECT ROW 1", p05RowOnePickup, false);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    prepareShotForPose(p06RowOneScore, shot06RowOne);
                    follow(rowOneScore, "07 ROW 1 -> SCORE", p06RowOneScore, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathFinishedOrTimedOut()) {
                    requestShot(shot06RowOne, DecodeConfig.AUTO_NEAR_FIRE_DURATION_MS);
                    setPathState(9);
                }
                break;
            case 9:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(rowThreeReady, "08 SCORE -> ROW 3 READY", p07RowThreeReady, false);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathFinishedOrTimedOut()) {
                    follow(collectRowThree, "09 COLLECT ROW 3", p08RowThreePickup, false);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    prepareShotForPose(p09FinalScore, shot09Final);
                    follow(finalScore, "10 ROW 3 -> FINAL SCORE", p09FinalScore, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathFinishedOrTimedOut()) {
                    requestShot(shot09Final, DecodeConfig.AUTO_NEAR_FIRE_DURATION_MS);
                    setPathState(13);
                }
                break;
            case 13:
                if (shotFinishedOrTimedOut()) {
                    shooter.setEnabled(false);
                    follow(scoreToPark, "11 FINAL SCORE -> PARK", parkPose, true);
                    setPathState(14);
                }
                break;
            case 14:
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
