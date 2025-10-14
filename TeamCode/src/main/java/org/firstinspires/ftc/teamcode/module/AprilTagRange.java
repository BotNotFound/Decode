package org.firstinspires.ftc.teamcode.module;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagRange {
    public static final String WEBCAM_NAME = "Webcam 1";

    private final AprilTagProcessor processor;
    private final VisionPortal visionPortal;

    private final Telemetry telemetry;

    public AprilTagRange(HardwareMap hardwareMap, Telemetry telemetry) {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(processor)
                .build();
        this.telemetry = telemetry;
    }

    public double getTagRange(int tagID) {
        telemetry.addData("Tag ID: ", tagID);
        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection.id == tagID) {
                telemetry.addData("Tag Status: " , tagID + "found");
                return detection.ftcPose.range;
            } else {
                telemetry.addData("Tag Status: ", tagID + "not found");
            }
        }
        return -1.0;
    }
}
