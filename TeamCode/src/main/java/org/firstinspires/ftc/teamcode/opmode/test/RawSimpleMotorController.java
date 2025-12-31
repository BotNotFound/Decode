package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(group = "rawcontroller")
public class RawSimpleMotorController extends RawDeviceController<DcMotorSimple> {
    public RawSimpleMotorController() {
        super(DcMotorSimple.class);
    }

    @Override
    protected void logDeviceData(DcMotorSimple motor) {
        telemetry.addData("Current Motor Power", motor.getPower());
        telemetry.addLine("[Move the left stick up and down to set the motor power]");
    }

    @Override
    protected void controlDevice(DcMotorSimple motor) {
        motor.setPower(gamepad1.left_stick_y);
    }
}