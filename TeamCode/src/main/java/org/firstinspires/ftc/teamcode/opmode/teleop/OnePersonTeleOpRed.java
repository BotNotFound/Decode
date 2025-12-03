package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = OnePersonTeleOpRed.OP_MODE_NAME, group = "1driver")
@Disabled
public class OnePersonTeleOpRed extends OnePersonTeleOp {
    public static final String OP_MODE_NAME = "1-Driver Teleop (Red)";

    public OnePersonTeleOpRed() {
        super(false);
    }

    @Override
    public void init() {
        super.init();
        robot.setAllianceColor(Robot.AllianceColor.RED);
    }
}
