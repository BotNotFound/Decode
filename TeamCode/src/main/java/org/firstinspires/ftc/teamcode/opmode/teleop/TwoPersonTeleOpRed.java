package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AllianceColor;

@TeleOp(name = TwoPersonTeleOpRed.OP_MODE_NAME, group = "2driver")
public class TwoPersonTeleOpRed extends TwoPersonTeleOp {
    public static final String OP_MODE_NAME = "2-Driver Teleop (Red)";

    public TwoPersonTeleOpRed() {
        super(false);
    }

    @Override
    public void init() {
        super.init();
        robot.setAllianceColor(AllianceColor.RED);
    }
}
