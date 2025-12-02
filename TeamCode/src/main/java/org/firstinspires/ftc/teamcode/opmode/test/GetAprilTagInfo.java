package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp
public class GetAprilTagInfo extends OpMode {
    private FieldCentricDriveTrain driveTrain;
    private AprilTagDetector detector;

    private static final int RED_TARGET_TAG_ID = Robot.AllianceColor.RED.targetAprilTagID;
    private static final int BLUE_TARGET_TAG_ID = Robot.AllianceColor.BLUE.targetAprilTagID;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        detector = new AprilTagDetector(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.start) {
            driveTrain.resetOdometry();
        }

        logAprilTagInfo(RED_TARGET_TAG_ID);
        logAprilTagInfo(BLUE_TARGET_TAG_ID);
    }

    private void logAprilTagInfo(int tagID) {
        detector.update(tagID);
        final AprilTagPoseFtc detection = detector.getTagPose();
        // getTagPose() currently logs whether or not the tag is detected, so we don't want
        // to duplicate log messages
        if (detection == null) {
            return;
        }

        telemetry.addData("Tag " + tagID + " X", detection.x);
        telemetry.addData("Tag " + tagID + " Y", detection.y);
        telemetry.addData("Tag " + tagID + " Z", detection.z);
        telemetry.addData("Tag " + tagID + " Distance", detection.range);
        telemetry.addData("Tag " + tagID + " Real Bearing", detection.bearing);
    }
}
