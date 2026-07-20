package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;

@TeleOp(name = "DECODE BLUE - STICK TURN 80%", group = "Competition")
public final class BlueDecodeTeleOp extends DecodeTeleOpBase {
    public BlueDecodeTeleOp() {
        super(Alliance.BLUE, TurnControl.RIGHT_STICK_80);
    }
}
