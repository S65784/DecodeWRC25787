package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;

@TeleOp(name = "DECODE BLUE - STICK TURN 100%", group = "Competition")
public final class BlueDecodeStickTeleOp extends DecodeTeleOpBase {
    public BlueDecodeStickTeleOp() {
        super(Alliance.BLUE, TurnControl.RIGHT_STICK);
    }
}
