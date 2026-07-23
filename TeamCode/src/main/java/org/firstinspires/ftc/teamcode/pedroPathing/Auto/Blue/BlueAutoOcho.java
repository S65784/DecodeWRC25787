package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;

import static org.firstinspires.ftc.teamcode.pedroPathing.Auto.AutoPaths.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Auto.PathStateAutoBase;

@Autonomous(name = "蓝 AutoOcho 远端 + 最后一组", group = "Competition")
public final class BlueAutoOcho extends PathStateAutoBase {

    //skipa2a3
    private static final ShotParameters SHOT = shot(1840, 0.52);
    private static final long REAR_SHOOT_MS = DecodeConfig.AUTO_FIRE_DURATION_MS;

    private final Pose startPose = pose(64.275, 9.431, 270);
    private final Pose preloadScorePose = pose(55, 12.000, 286);
    private static final long PRELOAD_CHASSIS_SETTLE_MS = 400;

    private final Pose pickupA1Pose = pose(8.1744, 11.1409, 180);
    private final Pose controlScoreToA1 = pose(18.139, 39.135, 180);
    private final Pose control2ScoreToA1 = pose(3.9719, 2.06603, 180);
    private final Pose scoreAfterA1Pose = pose(55, 12.000, 288.2);//-107.5
    private final Pose controlA1ToScore = pose(23.143, 17.619, 180);
    private static final long A1_PICKUP_DWELL_MS = 300;
    private static final long A1_CHASSIS_SETTLE_MS = 400;

    // Extra final-row pickup inserted after A1 is scored and before cycle B1.
    private final Pose p07RowThreeReady = pose(52.70967741935485, 36.000, 180);
    private final Pose p08RowThreePickup = pose(12, 36.000, 180);
    private static final long FINAL_ROW_PICKUP_DWELL_MS = 175;
    private static final long FINAL_ROW_CHASSIS_SETTLE_MS = 300;

    private final Pose prePickupB1Pose = pose(34.452, 13.465, 140);
    private final Pose pickupB1Pose = pose(7.664, 46.2495, 110);
    private final Pose controlPrePickupB1ToPickupB1 =
            pose(9.57483870967746, 11.639774193548401, 140);
    private final Pose scoreAfterB1Pose = pose(55, 12.500, 288);//108
    private final Pose controlB1ToScore = pose(30.305, 25.131, 180);
    private static final long B1_PICKUP_DWELL_MS = 250;
    private static final long B1_CHASSIS_SETTLE_MS = 300;

    private final Pose pickupA2Pose = pose(11.16958325106413, 11, 180);
//    private final Pose controlScoreToA2 = pose(20.98882388359655, 26.94817134808066, 180);
//    private final Pose control2ScoreToA2 = pose(3.38237227069334, 28.464703606145186, 180);
    private final Pose scoreAfterA2Pose = pose(55, 12.000, 288);
    private final Pose controlA2ToScore = pose(23.143, 17.619, 180);
    private static final long A2_PICKUP_DWELL_MS = 200;
    private static final long A2_CHASSIS_SETTLE_MS = 300;

    private final Pose prePickupB2Pose = pose(34.452, 13.465, 140);
    private final Pose pickupB2Pose = pose(10.4028, 46.706, 130);
    private final Pose controlPrePickupB2ToPickupB2 =
            pose(9.57483870967746, 11.639774193548401, 140);
    private final Pose scoreAfterB2Pose = pose(55, 12.500, 289);
    private final Pose controlB2ToScore = pose(30.305, 25.131, 180);
    private static final long B2_PICKUP_DWELL_MS = 200;
    private static final long B2_CHASSIS_SETTLE_MS = 300;

    private final Pose pickupA3Pose = pose(14, 11, 180);
    private final Pose controlScoreToA3 = pose(20.98882388359655, 26.94817134808066, 180);
    private final Pose control2ScoreToA3 = pose(3.61059807714497, 28.464703606145186, 180);
    private final Pose scoreAfterA3Pose = pose(55, 12.000, 289);
    private final Pose controlA3ToScore = pose(23.143, 17.619, 180);
    private static final long A3_PICKUP_DWELL_MS = 150;
    private static final long A3_CHASSIS_SETTLE_MS = 300;

    private final Pose prePickupB3Pose = pose(34.452, 13.465, 140);
    private final Pose pickupB3Pose = pose(7.8923, 42.3697, 110);
    private final Pose controlPrePickupB3ToPickupB3 =
            pose(9.57483870967746, 11.639774193548401, 140);
    private final Pose scoreAfterB3Pose = pose(55, 12.500, 289);
    private final Pose controlB3ToScore = pose(30.305, 25.131, 180);
    private static final long B3_PICKUP_DWELL_MS = 200;
    private static final long B3_CHASSIS_SETTLE_MS = 300;

    private final Pose parkPose = pose(28.41774193548387, 22.67580645161289, 180);
    private final Pose controlScoreToPark = pose(45.98225806451616, 24.5290322580645, 180);

    private Path preload;
    private Path scoreToA1;
    private Path a1ToScore;
    private Path scoreToFinalRowReady;
    private Path collectFinalRow;
    private Path finalRowToScore;
    private Path scoreToPreB1;
    private Path preB1ToB1;
    private Path b1ToScore;
    private Path scoreToA2;
    private Path a2ToScore;
    private Path scoreToPreB2;
    private Path preB2ToB2;
    private Path b2ToScore;
    private Path scoreToA3;
    private Path a3ToScore;
    private Path scoreToPreB3;
    private Path preB3ToB3;
    private Path b3ToScore;
    private Path scoreToPark;

    private static final int STATE_FINAL_ROW_READY = 80;
    private static final int STATE_FINAL_ROW_PICKUP = 81;
    private static final int STATE_FINAL_ROW_DWELL = 82;
    private static final int STATE_FINAL_ROW_RETURN = 83;
    private static final int STATE_FINAL_ROW_SETTLE = 84;
    private static final int STATE_FINAL_ROW_FIRE = 85;

    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected String autoName() { return "BlueAutoOcho"; }
    @Override protected Pose startingPose() { return startPose; }
    @Override protected void prepareInitialShot() {
        prepareShotForPose(preloadScorePose, SHOT);
    }

    @Override
    protected void buildPaths() {
        preload = line(startPose, preloadScorePose);
        scoreToA1 = curve(preloadScorePose, controlScoreToA1,control2ScoreToA1, pickupA1Pose);
        a1ToScore = curve(pickupA1Pose, controlA1ToScore, scoreAfterA1Pose);
        scoreToFinalRowReady = line(scoreAfterA1Pose, p07RowThreeReady);
        collectFinalRow = line(p07RowThreeReady, p08RowThreePickup);
        finalRowToScore = line(p08RowThreePickup, scoreAfterA1Pose);
        scoreToPreB1 = line(scoreAfterA1Pose, prePickupB1Pose);
        preB1ToB1 = curve(
                prePickupB1Pose, controlPrePickupB1ToPickupB1, pickupB1Pose);
        b1ToScore = curve(pickupB1Pose, controlB1ToScore, scoreAfterB1Pose);
        scoreToA2 = line(
                scoreAfterB1Pose, pickupA2Pose);
        a2ToScore = curve(pickupA2Pose, controlA2ToScore, scoreAfterA2Pose);
        scoreToPreB2 = line(scoreAfterA2Pose, prePickupB2Pose);
        preB2ToB2 = curve(
                prePickupB2Pose, controlPrePickupB2ToPickupB2, pickupB2Pose);
        b2ToScore = curve(pickupB2Pose, controlB2ToScore, scoreAfterB2Pose);
        scoreToA3 = line(
                scoreAfterB2Pose, pickupA3Pose);
        a3ToScore = curve(pickupA3Pose, controlA3ToScore, scoreAfterA3Pose);
        scoreToPreB3 = line(scoreAfterA3Pose, prePickupB3Pose);
        preB3ToB3 = curve(
                prePickupB3Pose, controlPrePickupB3ToPickupB3, pickupB3Pose);
        b3ToScore = curve(pickupB3Pose, controlB3ToScore, scoreAfterB3Pose);
        scoreToPark = curve(scoreAfterB3Pose, controlScoreToPark, parkPose);
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follow(preload, "01 PRELOAD -> SCORE", preloadScorePose, false);
                setPathState(1);
                break;
            case 1:
                if (pathFinishedOrTimedOut()) setPathState(2);
                break;
            case 2:
                if (waitFinished(PRELOAD_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(3);
                }
                break;
            case 3:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToA1, "02 SCORE -> PICKUP A1", pickupA1Pose, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathFinishedOrTimedOut()) setPathState(5);
                break;
            case 5:
                if (waitFinished(A1_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterA1Pose, SHOT);
                    follow(a1ToScore, "03 PICKUP A1 -> SCORE 1", scoreAfterA1Pose, false);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    setPathState(7);
                }
                break;
            case 7:
                if (waitFinished(A1_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(8);
                }
                break;
            case 8:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(
                            scoreToFinalRowReady,
                            "04 SCORE 1 -> FINAL ROW READY",
                            p07RowThreeReady,
                            false);
                    setPathState(STATE_FINAL_ROW_READY);
                }
                break;
            case STATE_FINAL_ROW_READY:
                if (pathFinishedOrTimedOut()) {
                    follow(
                            collectFinalRow,
                            "05 COLLECT FINAL ROW",
                            p08RowThreePickup,
                            false);
                    setPathState(STATE_FINAL_ROW_PICKUP);
                }
                break;
            case STATE_FINAL_ROW_PICKUP:
                if (pathFinishedOrTimedOut()) {
                    setPathState(STATE_FINAL_ROW_DWELL);
                }
                break;
            case STATE_FINAL_ROW_DWELL:
                if (waitFinished(FINAL_ROW_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterA1Pose, SHOT);
                    follow(
                            finalRowToScore,
                            "06 FINAL ROW -> SCORE",
                            scoreAfterA1Pose,
                            false);
                    setPathState(STATE_FINAL_ROW_RETURN);
                }
                break;
            case STATE_FINAL_ROW_RETURN:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    setPathState(STATE_FINAL_ROW_SETTLE);
                }
                break;
            case STATE_FINAL_ROW_SETTLE:
                if (waitFinished(FINAL_ROW_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(STATE_FINAL_ROW_FIRE);
                }
                break;
            case STATE_FINAL_ROW_FIRE:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToPreB1, "07 SCORE -> PREPICKUP B1", prePickupB1Pose, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathFinishedOrTimedOut()) {
                    follow(preB1ToB1, "08 PREPICKUP B1 -> PICKUP B1", pickupB1Pose, false);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathFinishedOrTimedOut()) setPathState(11);
                break;
            case 11:
                if (waitFinished(B1_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterB1Pose, SHOT);
                    follow(b1ToScore, "09 PICKUP B1 -> SCORE 2", scoreAfterB1Pose, false);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    setPathState(13);
                }
                break;
            case 13:
                if (waitFinished(B1_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(19);
                }
                break;


                //
            case 14:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToA2, "10 SCORE 2 -> PICKUP A2", pickupA2Pose, false);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathFinishedOrTimedOut()) setPathState(16);
                break;
            case 16:
                if (waitFinished(A2_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterA2Pose, SHOT);
                    follow(a2ToScore, "11 PICKUP A2 -> SCORE 3", scoreAfterA2Pose, false);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    setPathState(18);
                }
                break;
            case 18:
                if (waitFinished(A2_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(19);
                }
                break;
                //


            case 19:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToPreB2, "12 SCORE 3 -> PREPICKUP B2", prePickupB2Pose, false);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathFinishedOrTimedOut()) {
                    follow(preB2ToB2, "13 PREPICKUP B2 -> PICKUP B2", pickupB2Pose, false);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathFinishedOrTimedOut()) setPathState(22);
                break;
            case 22:
                if (waitFinished(B2_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterB2Pose, SHOT);
                    follow(b2ToScore, "14 PICKUP B2 -> SCORE 4", scoreAfterB2Pose, false);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    setPathState(24);
                }
                break;
            case 24:
                if (waitFinished(B2_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(25);
                }
                break;



            case 25:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToA3, "15 SCORE 4 -> PICKUP A3", pickupA3Pose, false);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathFinishedOrTimedOut()) setPathState(27);
                break;
            case 27:
                if (waitFinished(A3_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterA3Pose, SHOT);
                    follow(a3ToScore, "16 PICKUP A3 -> SCORE 5", scoreAfterA3Pose, false);
                    setPathState(28);
                }
                break;
            case 28:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    setPathState(29);
                }
                break;
            case 29:
                if (waitFinished(A3_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(36);
                }
                break;


            //
            case 30:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToPreB3, "17 SCORE 5 -> PREPICKUP B3", prePickupB3Pose, false);
                    setPathState(31);
                }
                break;
            case 31:
                if (pathFinishedOrTimedOut()) {
                    follow(preB3ToB3, "18 PREPICKUP B3 -> PICKUP B3", pickupB3Pose, false);
                    setPathState(32);
                }
                break;
            case 32:
                if (pathFinishedOrTimedOut()) setPathState(33);
                break;
            case 33:
                if (waitFinished(B3_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterB3Pose, SHOT);
                    follow(b3ToScore, "19 PICKUP B3 -> SCORE 6", scoreAfterB3Pose, false);
                    setPathState(34);
                }
                break;
            case 34:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    setPathState(35);
                }
                break;
            case 35:
                if (waitFinished(B3_CHASSIS_SETTLE_MS)) {
                    requestShot(SHOT, REAR_SHOOT_MS);
                    setPathState(36);
                }
                break;
                //


            case 36:
                if (shotFinishedOrTimedOut()) {
                    shooter.setEnabled(false);
                    follow(scoreToPark, "20 SCORE 6 -> PARK", parkPose, false);
                    setPathState(37);
                }
                break;
            case 37:
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
