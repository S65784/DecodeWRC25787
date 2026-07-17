package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PedroPathing 2.1.2 configuration copied from the team's supplied constants.
 *
 * <p>Run Pedro's Tuning OpMode before competition. In particular, verify the Pinpoint pod
 * offsets and encoder directions on the finished DECODE robot.</p>
 */
public final class Constants {
    private Constants() {
    }

    public static final FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.806)
            .forwardZeroPowerAcceleration(-29)
            .lateralZeroPowerAcceleration(-62)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.027, 0.02))
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.21, 0.000076, 0.03, 0.017))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.018, 0.015, 0.025))
            .secondaryHeadingPIDFCoefficients(
                    new PIDFCoefficients(0.8, 0.0001, 0.02, 0.02))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.009, 0.0007, 0.00000001, 0.6, 0.0003))
            .centripetalScaling(0.001);

    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFrontDrive")
            .rightRearMotorName("RightBackDrive")
            .leftRearMotorName("LeftBackDrive")
            .leftFrontMotorName("LeftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.3)
            .yVelocity(64);

    public static final PinpointConstants localizerConstants = new PinpointConstants()
            // TODO: Measure both offsets again after the Pinpoint pods are mounted permanently.
            .forwardPodY(0)
            .strafePodX(-2.165354)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static final PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 0.5, 0.5);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
