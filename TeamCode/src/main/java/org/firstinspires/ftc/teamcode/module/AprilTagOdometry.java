package org.firstinspires.ftc.teamcode.module;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagOdometry {
    public static final String WEBCAM_NAME = "Webcam 1";

    private final AprilTagProcessor processor;
    private final VisionPortal visionPortal;

    public AprilTagOdometry(HardwareMap hardwareMap) {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(processor)
                .build();
    }

    @Nullable
    public Position getTagPosition(int tagID) {
        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection.id == tagID) {
                return detection.robotPose.getPosition();
            }
        }
        return null;
    }
}
