package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;

@TeleOp(name = "DECODE TeleOp - BLUE", group = "Competition")
public final class BlueDecodeTeleOp extends DecodeTeleOpBase {
    public BlueDecodeTeleOp() {
        super(Alliance.BLUE);
    }
}
