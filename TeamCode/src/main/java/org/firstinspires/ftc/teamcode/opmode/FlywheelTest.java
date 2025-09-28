package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
