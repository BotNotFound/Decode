package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.module.PeterGriffin;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp
public class DriveOnly extends OpMode {
    private PeterGriffin peter;

    public static final double STRAFE_SCALE = Math.sqrt(2);

    @Override
    public void init() {
        peter = new PeterGriffin(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x*STRAFE_SCALE, gamepad1.right_stick_x);

        AprilTagPoseFtc temp = peter.getTagPose(24);

        if(temp != null){
            telemetry.addData("Bearing", temp.bearing);

            telemetry.addData("x distance", temp.x);
            telemetry.addData("y distance", temp.y);
            telemetry.addData("z distance", temp.z);
        }

    }
}
