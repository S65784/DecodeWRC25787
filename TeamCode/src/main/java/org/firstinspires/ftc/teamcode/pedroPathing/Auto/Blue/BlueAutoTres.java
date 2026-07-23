
package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;

import static org.firstinspires.ftc.teamcode.pedroPathing.Auto.AutoPaths.*;

        import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Auto.PathStateAutoBase;

/**
 * BlueAutoCero route reordered to Row 1, Row 2, Row 3, with two side-gate pushes.
 *
 * <p>All non-gate coordinates and mechanism parameters come from BlueAutoCero.
 * Only c04/p04 and c08/p08 come from the tuned BlueAutoCinco gate paths.</p>
 */
@Autonomous(name = "蓝 AutoCinco（近推门1）", group = "Competition")
public final class BlueAutoTres extends PathStateAutoBase {
    private final Pose p00Start = pose(20.543, 122.478, 315.2839);
    private final Pose p01PreloadScore = pose(52.2, 89.8, -53.841815);
    private final ShotParameters shot01Preload = shot(1369, 0.2474);

    // Row 1 (y = 83) runs first.
    private final Pose p02RowOneReady = pose(54, 83.000, 180);
    private final Pose p03RowOnePickup = pose(19, 83.000, 180);
    //25787
    // Gate 1 coordinates remain unchanged.
    private final Pose c04GateOne = pose(39.06772908366534, 69.4183266932271, -90);//114.93227091633466
    private final Pose p04GateOne  = pose(144-127, 72, 180);
    // Dwell held after the gate path ends, before leaving for the scoring pose. The gate path
    // holds its end pose, so the chassis keeps pressing the gate for this whole time.
    // TODO: Measure on the field. Long dwells stall the drive and sag the battery right before
    // a shot, which can push the next spin-up past AUTO_SHOT_TIMEOUT_MS.
    private static final long GATE_ONE_DWELL_MS = 1500;

    private final Pose p05RowOneScore = pose(60, 83.000, -47.726311);
    private final ShotParameters shot05RowOne = shot(1408, 0.2782);

    // Row 2 (y = 59) runs second.
    private final Pose c06RowTwoReady = pose(56.31770967741934, 72.26816129032255, 180);
    private final Pose p06RowTwoReady = pose(58.18709677419355, 59.000, 180);
    private final Pose p07RowTwoPickup = pose(13, 59.000, 180);

    // Gate 2 coordinates remain unchanged.
    private final Pose c08GateTwo = pose(38, 52.91444600280506, -90);//140-20
    private final Pose p08GateTwo = pose(144-128, 72, 180);
    // TODO: Measure on the field. See the note on GATE_ONE_DWELL_MS.
    private static final long GATE_TWO_DWELL_MS = 2500;

    private final Pose c09RowTwoScore = pose(45, 60.000, 180);
    private final Pose p09RowTwoScore = pose(60, 83.000, -47.726311);
    private final ShotParameters shot09RowTwo = shot(1408, 0.2782);

    // Row 3: no gate push.
    private final Pose c10RowThreeReady = pose(57.16209677419354, 42.416129032258056, 180);
    private final Pose p10RowThreeReady = pose(56.5, 36.000, 180);
    private final Pose p11RowThreePickup = pose(12, 36.000, 180);
    private final Pose p12FinalScore = pose(59.923, 107.923, -31.06761);
    private final ShotParameters shot12Final = shot(1348, 0.2300);
    private final Pose p13Park = pose(58, 108.9, 180);

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

    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected String autoName() { return "BlueAutoCinco"; }
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

        //
        gateTwo = curveChain(follower, p07RowTwoPickup, c08GateTwo, p08GateTwo);
        //

        rowTwoScore = curveChain(
                follower, p07RowTwoPickup, c09RowTwoScore, p09RowTwoScore);

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
                    setPathState(450);
                }
                break;
            case 450:
                // Entering state 5 restarts the state timer, so the dwell below is measured
                // from the moment the gate path ended instead of from when it started.
                if (pathFinishedOrTimedOut()) {
                    setPathState(5);
                }
                break;
            case 5:
                if (waitFinished(GATE_ONE_DWELL_MS)) {
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
                    setPathState(10);
                }
                break;


                //
            case 9:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
                    follow(gateTwo, "08 PUSH GATE 2", p08GateTwo, true);
                    setPathState(950);
                }
                break;
            case 950:
                // Same split as state 450: this state only exists to restart the state timer
                // when the gate path ends, so state 10 can dwell from that instant.
                if (pathFinishedOrTimedOut()) {
                    setPathState(10);
                }
                break;
                //



            case 10:
                if (pathFinishedOrTimedOut()) {
                    stopIntake();
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
