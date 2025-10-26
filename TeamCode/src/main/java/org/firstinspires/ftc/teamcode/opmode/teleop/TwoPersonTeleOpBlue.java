package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpRobot;

@TeleOp(name = "2-Driver Teleop (Blue)", group = "2driver")
public class TwoPersonTeleOpBlue extends TwoPersonTeleOp {
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
