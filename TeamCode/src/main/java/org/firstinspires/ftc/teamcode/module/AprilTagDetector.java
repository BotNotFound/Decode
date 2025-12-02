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

    private static final Position CAMERA_POSITION = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90 + 30, 0, 0);

    private final AprilTagProcessor processor;
    private final VisionPortal visionPortal;

    private final Telemetry telemetry;

    private Pose3D robotPose;
    private AprilTagPoseFtc tagPose;
    private boolean tagDetected = false;

    public AprilTagDetector(HardwareMap hardwareMap, Telemetry telemetry) {
        processor = new AprilTagProcessor.Builder()
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(processor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        this.telemetry = telemetry;
    }

    /**
     * @apiNote Must be called every relevant loop iteration. Otherwise, the AprilTagDetector object will give stale values.
     * @param tagID the ID of the tag that is being detected
     */
    public void update(int tagID){
        tagDetected = false;

        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection.id == tagID) {
                robotPose = detection.robotPose;
                tagPose = detection.ftcPose;

                tagDetected = true;
                telemetry.addData("AprilTag " + tagID + " ", "detected");
            }
        }

        if(tagDetected){
            telemetry.addData("AprilTag " + tagID + " ", "not detected");
        }
    }

    @Nullable
    public Pose3D getRobotPose(){
        if(tagDetected){
            return robotPose;
        }

        return null;
    }

    @Nullable
    public AprilTagPoseFtc getTagPose(){
        if(tagDetected){
            return tagPose;
        }
        return null;
    }

}