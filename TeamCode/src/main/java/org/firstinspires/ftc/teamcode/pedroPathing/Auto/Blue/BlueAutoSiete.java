package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;

import static org.firstinspires.ftc.teamcode.pedroPathing.Auto.AutoPaths.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Auto.PathStateAutoBase;

@Autonomous(name = "蓝 AutoSiete 远端多次吸球", group = "Competition")
public final class BlueAutoSiete extends PathStateAutoBase {
    private static final ShotParameters SHOT = shot(1860, 0.52);
    private static final long REAR_SHOOT_MS = DecodeConfig.AUTO_FIRE_DURATION_MS;

    private final Pose startPose = pose(64.275, 9.431, 270);
    private final Pose preloadScorePose = pose(55, 12.000, 286);
    private static final long PRELOAD_CHASSIS_SETTLE_MS = 350;

    private final Pose pickupA1Pose = pose(11.826, 8.174, 180);
    private final Pose controlScoreToA1 = pose(29.43064516129031, 32.40806451612903, 180);
    private final Pose control2ScoreToA1 = pose(16.99758096774194, 31.604032387096776, 180);
    private final Pose scoreAfterA1Pose = pose(55, 12.000, 288);
    private final Pose controlA1ToScore = pose(23.143, 17.619, 180);
    private static final long A1_PICKUP_DWELL_MS = 90;
    private static final long A1_CHASSIS_SETTLE_MS = 300;

    private final Pose prePickupB1Pose = pose(34.452, 13.465, 140);
    private final Pose pickupB1Pose = pose(12.91319354838708, 17.493096774193557, 130);
    private final Pose controlPrePickupB1ToPickupB1 =
            pose(9.57483870967746, 11.639774193548401, 140);
    private final Pose scoreAfterB1Pose = pose(55, 12.500, 288);
    private final Pose controlB1ToScore = pose(30.305, 25.131, 180);
    private static final long B1_PICKUP_DWELL_MS = 250;
    private static final long B1_CHASSIS_SETTLE_MS = 300;

    private final Pose pickupA2Pose = pose(144-133.5, 11, 180);
    private final Pose control2ScoreToA2 = pose(3.38237227069334, 28.464703606145186, 180);
    private final Pose scoreAfterA2Pose = pose(55, 12.000, 289);
    private final Pose controlA2ToScore = pose(23.143, 17.619, 180);
    private static final long A2_PICKUP_DWELL_MS = 200;
    private static final long A2_CHASSIS_SETTLE_MS = 300;

    private final Pose prePickupB2Pose = pose(34.452, 13.465, 140);
    private final Pose pickupB2Pose = pose(12.91319354838708, 17.493096774193557, 130);
    private final Pose controlPrePickupB2ToPickupB2 =
            pose(9.57483870967746, 11.639774193548401, 140);
    private final Pose scoreAfterB2Pose = pose(55, 12.500, 289);
    private final Pose controlB2ToScore = pose(30.305, 25.131, 180);
    private static final long B2_PICKUP_DWELL_MS = 200;
    private static final long B2_CHASSIS_SETTLE_MS = 300;

    private final Pose pickupA3Pose = pose(144-133.5, 11, 180);
    private final Pose control2ScoreToA3 = pose(3.61059807714497, 28.464703606145186, 180);
    private final Pose scoreAfterA3Pose = pose(55, 12.000, 289);
    private final Pose controlA3ToScore = pose(23.143, 17.619, 180);
    private static final long A3_PICKUP_DWELL_MS = 150;
    private static final long A3_CHASSIS_SETTLE_MS = 300;

    private final Pose prePickupB3Pose = pose(34.452, 13.465, 140);
    private final Pose pickupB3Pose= pose(12.91319354838708, 17.493096774193557, 130);
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

    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected String autoName() { return "BlueAutoSiete"; }
    @Override protected Pose startingPose() { return startPose; }
    @Override protected void prepareInitialShot() {
        prepareShotForPose(preloadScorePose, SHOT);
    }

    @Override
    protected void buildPaths() {
        preload = line(startPose, preloadScorePose);
        scoreToA1 = curve(preloadScorePose, control2ScoreToA1, pickupA1Pose);
        a1ToScore = curve(pickupA1Pose, controlA1ToScore, scoreAfterA1Pose);
        scoreToPreB1 = line(scoreAfterA1Pose, prePickupB1Pose);
        preB1ToB1 = curve(
                prePickupB1Pose, controlPrePickupB1ToPickupB1, pickupB1Pose);
        b1ToScore = curve(pickupB1Pose, controlB1ToScore, scoreAfterB1Pose);
        scoreToA2 = curve(scoreAfterB1Pose, control2ScoreToA2, pickupA2Pose);
        a2ToScore = curve(pickupA2Pose, controlA2ToScore, scoreAfterA2Pose);
        scoreToPreB2 = line(scoreAfterA2Pose, prePickupB2Pose);
        preB2ToB2 = curve(
                prePickupB2Pose, controlPrePickupB2ToPickupB2, pickupB2Pose);
        b2ToScore = curve(pickupB2Pose, controlB2ToScore, scoreAfterB2Pose);
        scoreToA3 = curve(scoreAfterB2Pose, control2ScoreToA3, pickupA3Pose);
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
                    follow(scoreToPreB1, "04 SCORE 1 -> PREPICKUP B1", prePickupB1Pose, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathFinishedOrTimedOut()) {
                    follow(preB1ToB1, "05 PREPICKUP B1 -> PICKUP B1", pickupB1Pose, false);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathFinishedOrTimedOut()) setPathState(11);
                break;
            case 11:
                if (waitFinished(B1_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterB1Pose, SHOT);
                    follow(b1ToScore, "06 PICKUP B1 -> SCORE 2", scoreAfterB1Pose, false);
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
                    setPathState(14);
                }
                break;
            case 14:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToA2, "07 SCORE 2 -> PICKUP A2", pickupA2Pose, false);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathFinishedOrTimedOut()) setPathState(16);
                break;
            case 16:
                if (waitFinished(A2_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterA2Pose, SHOT);
                    follow(a2ToScore, "08 PICKUP A2 -> SCORE 3", scoreAfterA2Pose, false);
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
            case 19:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToPreB2, "09 SCORE 3 -> PREPICKUP B2", prePickupB2Pose, false);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathFinishedOrTimedOut()) {
                    follow(preB2ToB2, "10 PREPICKUP B2 -> PICKUP B2", pickupB2Pose, false);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathFinishedOrTimedOut()) setPathState(22);
                break;
            case 22:
                if (waitFinished(B2_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterB2Pose, SHOT);
                    follow(b2ToScore, "11 PICKUP B2 -> SCORE 4", scoreAfterB2Pose, false);
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
                    follow(scoreToA3, "12 SCORE 4 -> PICKUP A3", pickupA3Pose, false);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathFinishedOrTimedOut()) setPathState(27);
                break;
            case 27:
                if (waitFinished(A3_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterA3Pose, SHOT);
                    follow(a3ToScore, "13 PICKUP A3 -> SCORE 5", scoreAfterA3Pose, false);
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
                    setPathState(30);
                }
                break;
            case 30:
                if (shotFinishedOrTimedOut()) {
                    startIntake(1);
                    follow(scoreToPreB3, "14 SCORE 5 -> PREPICKUP B3", prePickupB3Pose, false);
                    setPathState(31);
                }
                break;
            case 31:
                if (pathFinishedOrTimedOut()) {
                    follow(preB3ToB3, "15 PREPICKUP B3 -> PICKUP B3", pickupB3Pose, false);
                    setPathState(32);
                }
                break;
            case 32:
                if (pathFinishedOrTimedOut()) setPathState(33);
                break;
            case 33:
                if (waitFinished(B3_PICKUP_DWELL_MS)) {
                    prepareShotForPose(scoreAfterB3Pose, SHOT);
                    follow(b3ToScore, "16 PICKUP B3 -> SCORE 6", scoreAfterB3Pose, false);
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
            case 36:
                if (shotFinishedOrTimedOut()) {
                    shooter.setEnabled(false);
                    follow(scoreToPark, "17 SCORE 6 -> PARK", parkPose, false);
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
