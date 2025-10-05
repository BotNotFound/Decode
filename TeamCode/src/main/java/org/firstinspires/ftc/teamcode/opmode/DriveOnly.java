package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.module.DriveTrain;

@TeleOp
public class DriveOnly extends OpMode {
    private DriveTrain driveTrain;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
    }

    @Override
    public void loop() {
        driveTrain.setPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
