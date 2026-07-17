package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.config.Alliance;

@TeleOp(name = "DECODE TeleOp - RED", group = "Competition")
public final class RedDecodeTeleOp extends DecodeTeleOpBase {
    public RedDecodeTeleOp() {
        super(Alliance.RED);
    }
}
