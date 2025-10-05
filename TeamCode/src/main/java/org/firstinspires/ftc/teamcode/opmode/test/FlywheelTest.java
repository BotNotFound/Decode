package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.module.Flywheel;

@TeleOp(group = "Test")
public class FlywheelTest extends OpMode {

    private Flywheel flywheel;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void loop() {
        final double power = gamepad1.left_stick_y;
        flywheel.setPower(power);
    }
}
