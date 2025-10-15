package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.DriveTrain;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp
public class DriveOnly extends OpMode {
    private FieldCentricDriveTrain driveTrain;

    private AprilTagDetector aprilDetector;

    @Override
    public void init() {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        aprilDetector = new AprilTagDetector(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        driveTrain.setPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        AprilTagPoseFtc temp = aprilDetector.getTagPose(24);

        if(temp != null){
            telemetry.addData("Bearing", temp.bearing);

            telemetry.addData("x distance", temp.x);
            telemetry.addData("y distance", temp.y);
            telemetry.addData("z distance", temp.z);
        }

    }
}
