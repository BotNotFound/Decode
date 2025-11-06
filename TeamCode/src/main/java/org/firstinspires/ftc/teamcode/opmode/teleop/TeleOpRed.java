package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "1-Driver Teleop (Red)", group = "1driver")
public class TeleOpRed extends TeleOp {
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
