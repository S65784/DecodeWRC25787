package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;

@TeleOp(name = "DECODE RED - STICK TURN 100%", group = "Competition")
public final class RedDecodeStickTeleOp extends DecodeTeleOpBase {
    public RedDecodeStickTeleOp() {
        super(Alliance.RED, TurnControl.RIGHT_STICK);
    }
}
