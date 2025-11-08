package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = TeleOpBlue.OP_MODE_NAME, group = "1driver")
public class TeleOpBlue extends TeleOp {
    public static final String OP_MODE_NAME = "1-Driver Teleop (Blue)";

    @Override
    public void init() {
        super.init();
        robot.setAllianceColor(Robot.AllianceColor.BLUE);
    }

    @Override
    public void init_loop() {
        // do not allow driver to change alliance color
    }
}
