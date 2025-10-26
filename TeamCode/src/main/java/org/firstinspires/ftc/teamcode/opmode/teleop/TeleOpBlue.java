package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.TeleOpRobot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "1-Driver Teleop (Blue)", group = "1driver")
public class TeleOpBlue extends TeleOp {
    @Override
    public void init() {
        super.init();
        robot.setAllianceColor(TeleOpRobot.AllianceColor.BLUE);
    }

    @Override
    public void init_loop() {
        // do not allow driver to change alliance color
    }
}
