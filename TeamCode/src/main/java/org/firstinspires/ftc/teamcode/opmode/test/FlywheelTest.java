package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.module.Shooter;

@TeleOp(group = "Test")
public class FlywheelTest extends OpMode {

    private Shooter flywheel;

    @Override
    public void init() {
        flywheel = new Shooter(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        final double power = gamepad1.left_stick_y;
        flywheel.setRPMForAprilTag(power);

        if (gamepad1.a) {
            flywheel.engageKicker();
        }
        else {
            flywheel.disengageKicker();
        }
    }
}
