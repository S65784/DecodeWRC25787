package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.control.SharedFlywheelController;
import org.firstinspires.ftc.teamcode.decode.util.RisingEdge;

/**
 * Live Panels tuner for the exact shared-power controller used by ShooterSubsystem.
 *
 * <p>ShooterL is the only velocity feedback source. Both motors receive the same power.</p>
 */
@Configurable
@TeleOp(name = "Motor PIDF Tuner (Panels)", group = "Calibration")
public final class MotorPIDFTunerPanels extends OpMode {
    public static boolean ENABLED = false;
    public static double TARGET_TICKS_PER_SECOND = 1260;
    public static double KP = DecodeConfig.SHOOTER_KP;
    public static double KI = DecodeConfig.SHOOTER_KI;
    public static double KD = DecodeConfig.SHOOTER_KD;
    public static double KF = DecodeConfig.SHOOTER_KF;
    public static double INTEGRAL_POWER_LIMIT = DecodeConfig.SHOOTER_INTEGRAL_LIMIT;

    private static final double TICKS_PER_REVOLUTION = 28;

    private final RisingEdge togglePressed = new RisingEdge();
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private SharedFlywheelController controller;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        leftShooter = hardwareMap.get(DcMotorEx.class, DecodeConfig.LEFT_SHOOTER_MOTOR);
        rightShooter = hardwareMap.get(DcMotorEx.class, DecodeConfig.RIGHT_SHOOTER_MOTOR);

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        configureMotor(leftShooter);
        configureMotor(rightShooter);

        controller = new SharedFlywheelController(leftShooter, rightShooter);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetry.addLine("Open Panels at http://192.168.43.1:8001");
        telemetry.addLine("Toggle ENABLED in Panels or press gamepad1 A.");
        telemetry.update();
    }

    private static void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if (togglePressed.update(gamepad1.a)) {
            ENABLED = !ENABLED;
        }

        controller.setGains(KP, KI, KD, KF, INTEGRAL_POWER_LIMIT);
        controller.setTargetVelocity(ENABLED ? TARGET_TICKS_PER_SECOND : 0);
        controller.update();

        double currentTicksPerSecond = controller.getMeasuredVelocity();
        double targetRpm = TARGET_TICKS_PER_SECOND / TICKS_PER_REVOLUTION * 60;
        double currentRpm = currentTicksPerSecond / TICKS_PER_REVOLUTION * 60;

        panelsTelemetry.debug("enabled", ENABLED);
        panelsTelemetry.debug("target ticks/s", ENABLED ? TARGET_TICKS_PER_SECOND : 0);
        panelsTelemetry.debug("ShooterL ticks/s", currentTicksPerSecond);
        panelsTelemetry.debug("velocity error", controller.getError());
        panelsTelemetry.debug("shared power", controller.getAppliedPower());
        panelsTelemetry.debug("target RPM", ENABLED ? targetRpm : 0);
        panelsTelemetry.debug("current RPM", currentRpm);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        ENABLED = false;
        if (controller != null) {
            controller.stop();
        }
    }
}
