package org.firstinspires.ftc.teamcode.opmode.rawcontrol;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "rawcontroller")
public class RawServoController extends RawDeviceController<Servo> {
    public RawServoController() {
        super(Servo.class);
    }

    @Override
    protected void logDeviceData(Servo servo) {
        telemetry.addData("Current Servo Position", servo.getPosition());
        telemetry.addLine("[Move the left stick up and down to set the servo position]");
    }

    @Override
    protected void controlDevice(Servo servo) {
        servo.setPosition((1 - gamepad1.left_stick_y) / 2);
    }
}
