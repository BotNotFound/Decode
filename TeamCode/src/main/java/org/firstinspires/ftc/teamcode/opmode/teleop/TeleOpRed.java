package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = TeleOpRed.OP_MODE_NAME, group = "1driver")
public class TeleOpRed extends TeleOp {
    public static final String OP_MODE_NAME = "1-Driver Teleop (Red)";

    @Override
    public void init() {
        super.init();
        robot.setAllianceColor(Robot.AllianceColor.RED);
    }

    @Override
    public void init_loop() {
        // do not allow driver to change alliance color
    }
}
