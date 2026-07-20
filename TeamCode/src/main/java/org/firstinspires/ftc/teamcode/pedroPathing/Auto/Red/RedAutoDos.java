//package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Auto.AutoPaths.*;
//
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.decode.config.Alliance;
//import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
//import org.firstinspires.ftc.teamcode.pedroPathing.Auto.PathStateAutoBase;
//
//@Autonomous(name = "红 AutoDos 远端（新版机构）", group = "Competition")
//public final class RedAutoDos extends PathStateAutoBase {
//    private final Pose p00Start = pose(79.725, 9.431, -90);
//    private final Pose p01PreloadScore = pose(91.0, 14.65060240963856, 250);
//    private final ShotParameters shot01Preload = shot(1831, 0.5195);
//    private final Pose c02PickupReady = pose(103.71084337349397, 4.2409638554216915, -22);
//    private final Pose p02PickupReady = pose(127.42168674698794, 19.66265060240964, -22);
//    private final Pose c03CollectOne = pose(136.72289156626508, 18.313253012048197, 0);
//    private final Pose c03CollectTwo = pose(122.60240963855422, 12.530120481927707, 0);
//    private final Pose p03Pickup = pose(136.09638554216866, 7.518072289156624, 0);
//    private final Pose c04ScoreOne = pose(104.09638554216868, 19.46987951807229, 72);
//    private final Pose p04ScoreOne = pose(92.5, 15, 243);
//    private final ShotParameters shot04One = shot(1826, 0.5194);
//    private final Pose c05PickupReady = pose(103.71084337349397, 4.2409638554216915, -22);
//    private final Pose p05PickupReady = pose(127.42168674698794, 19.66265060240964, -22);
//    private final Pose c06CollectOne = pose(136.72289156626508, 18.313253012048197, 0);
//    private final Pose c06CollectTwo = pose(122.60240963855422, 12.530120481927707, 0);
//    private final Pose p06Pickup = pose(136.09638554216866, 7.518072289156624, 0);
//    private final Pose c07ScoreTwo = pose(104.09638554216868, 19.46987951807229, 72);
//    private final Pose p07ScoreTwo = pose(92.5, 15, 245);
//    private final ShotParameters shot07Two = shot(1826, 0.5194);
//    private final Pose p08Park = pose(105.4, 14.5, 250);
//
//    private static final double PICKUP_MAX_POWER = 0.60;
//
//    private Path preload;
//    private Path pickupReadyOne;
//    private Path collectOne;
//    private Path scoreOne;
//    private Path pickupReadyTwo;
//    private Path collectTwo;
//    private Path scoreTwo;
//    private Path park;
//
//    @Override protected Alliance alliance() { return Alliance.RED; }
//    @Override protected String autoName() { return "RedAutoDos"; }
//    @Override protected Pose startingPose() { return p00Start; }
//    @Override protected void prepareInitialShot() {
//        prepareShotForPose(p01PreloadScore, shot01Preload);
//    }
//
//    @Override
//    protected void buildPaths() {
//        preload = line(p00Start, p01PreloadScore);
//        pickupReadyOne = curve(p01PreloadScore, c02PickupReady, p02PickupReady);
//        collectOne = curve(p02PickupReady, c03CollectOne, c03CollectTwo, p03Pickup);
//        scoreOne = curve(p03Pickup, c04ScoreOne, p04ScoreOne);
//        pickupReadyTwo = curve(p04ScoreOne, c05PickupReady, p05PickupReady);
//        collectTwo = curve(p05PickupReady, c06CollectOne, c06CollectTwo, p06Pickup);
//        scoreTwo = curve(p06Pickup, c07ScoreTwo, p07ScoreTwo);
//        park = line(p07ScoreTwo, p08Park);
//    }
//
//    @Override
//    protected void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follow(preload, "01 START -> PRELOAD SCORE", p01PreloadScore, true);
//                setPathState(1);
//                break;
//            case 1:
//                if (pathFinishedOrTimedOut()) {
//                    requestShot(shot01Preload, DecodeConfig.AUTO_FIRE_DURATION_MS);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (shotFinishedOrTimedOut()) {
//                    shooter.setEnabled(false);
//                    follow(pickupReadyOne, "02 SCORE -> PICKUP READY 1", p02PickupReady, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (pathFinishedOrTimedOut()) {
//                    startIntake(PICKUP_MAX_POWER);
//                    follow(collectOne, "03 COLLECT 1", p03Pickup, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (pathFinishedOrTimedOut()) {
//                    stopIntake();
//                    prepareShotForPose(p04ScoreOne, shot04One);
//                    follow(scoreOne, "04 PICKUP 1 -> SCORE 1", p04ScoreOne, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (pathFinishedOrTimedOut()) {
//                    requestShot(shot04One, DecodeConfig.AUTO_FIRE_DURATION_MS);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (shotFinishedOrTimedOut()) {
//                    shooter.setEnabled(false);
//                    follow(pickupReadyTwo, "05 SCORE 1 -> PICKUP READY 2", p05PickupReady, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (pathFinishedOrTimedOut()) {
//                    startIntake(PICKUP_MAX_POWER);
//                    follow(collectTwo, "06 COLLECT 2", p06Pickup, true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if (pathFinishedOrTimedOut()) {
//                    stopIntake();
//                    prepareShotForPose(p07ScoreTwo, shot07Two);
//                    follow(scoreTwo, "07 PICKUP 2 -> SCORE 2", p07ScoreTwo, true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (pathFinishedOrTimedOut()) {
//                    requestShot(shot07Two, DecodeConfig.AUTO_FIRE_DURATION_MS);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if (shotFinishedOrTimedOut()) {
//                    shooter.setEnabled(false);
//                    follow(park, "08 SCORE 2 -> PARK", p08Park, true);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if (pathFinishedOrTimedOut()) {
//                    finishAuto();
//                    setPathState(-1);
//                }
//                break;
//            default:
//                break;
//        }
//    }
//}
