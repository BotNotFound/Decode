package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Reset Robot State", group = "00prelude")
public class ClearRobotState extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("!THIS WILL CLEAR ROBOT STATE!");
        telemetry.update();
        waitForStart();
        Robot.clearPersistentState();
    }
}
