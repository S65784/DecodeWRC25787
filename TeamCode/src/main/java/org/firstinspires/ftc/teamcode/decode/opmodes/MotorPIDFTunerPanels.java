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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.config.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.control.SharedFlywheelController;
import org.firstinspires.ftc.teamcode.decode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decode.util.RisingEdge;

/**
 * Live Panels tuner for the exact shared-power controller used by ShooterSubsystem.
 *
 * <p>ShooterR is the only velocity feedback source. Both motors receive the same power.</p>
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
    private IntakeSubsystem intake;
    private Servo gate;
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

        intake = new IntakeSubsystem(hardwareMap);
        gate = hardwareMap.get(Servo.class, DecodeConfig.GATE_SERVO);
        gate.setPosition(DecodeConfig.GATE_CLOSED_POSITION);

        controller = new SharedFlywheelController(rightShooter, leftShooter);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetry.addLine("Open Panels at http://192.168.43.1:8001");
        telemetry.addLine("Toggle ENABLED in Panels or press gamepad1 A.");
        telemetry.addLine("WARNING: ENABLED opens hao and continuously feeds Intake.");
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

        if (ENABLED) {
            gate.setPosition(DecodeConfig.GATE_OPEN_POSITION);
            intake.beginShootFeed();
        } else {
            gate.setPosition(DecodeConfig.GATE_CLOSED_POSITION);
            intake.endShootFeed();
        }
        intake.update();

        double currentTicksPerSecond = controller.getMeasuredVelocity();
        double targetRpm = TARGET_TICKS_PER_SECOND / TICKS_PER_REVOLUTION * 60;
        double currentRpm = currentTicksPerSecond / TICKS_PER_REVOLUTION * 60;

        // Graph parses numeric telemetry in "name: value" form. addData() emits that form,
        // whereas debug("name", value) emits two separate lines and produces no graph variables.
        panelsTelemetry.addData("enabled", ENABLED ? 1 : 0);
        panelsTelemetry.addData(
                "target ticks/s",
                ENABLED ? TARGET_TICKS_PER_SECOND : 0);
        panelsTelemetry.addData("ShooterR ticks/s", currentTicksPerSecond);
        panelsTelemetry.addData("velocity error", controller.getError());
        panelsTelemetry.addData("shared power", controller.getAppliedPower());
        panelsTelemetry.addData("target RPM", ENABLED ? targetRpm : 0);
        panelsTelemetry.addData("current RPM", currentRpm);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        ENABLED = false;
        if (controller != null) {
            controller.stop();
        }
        if (gate != null) {
            gate.setPosition(DecodeConfig.GATE_CLOSED_POSITION);
        }
        if (intake != null) {
            intake.stop();
        }
    }
}
