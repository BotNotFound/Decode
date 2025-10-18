package org.firstinspires.ftc.teamcode.module;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagDetector {
    public static final String WEBCAM_NAME = "Webcam 1";

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90 + 30, 0, 0);

    private final AprilTagProcessor processor;
    private final VisionPortal visionPortal;

    private final Telemetry telemetry;

    public AprilTagDetector(HardwareMap hardwareMap, Telemetry telemetry) {
        processor = new AprilTagProcessor.Builder().
                setCameraPose(cameraPosition, cameraOrientation).
                build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(processor)
                .build();
        this.telemetry = telemetry;
    }

    @Nullable
    public AprilTagPoseFtc getTagPose(int tagID){
        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection.id == tagID) {
                telemetry.addData("AprilTag ", "detected");
                return detection.ftcPose;
            } else {
                telemetry.addData("AprilTag ", "not detected");
            }
        }
        return null;
    }
}