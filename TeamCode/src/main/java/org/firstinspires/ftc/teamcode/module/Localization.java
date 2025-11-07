package org.firstinspires.ftc.teamcode.module;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Localization {
    private static Pose2D robotPose;

    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;

    private static GoBildaPinpointDriver pinpoint;

    private static AprilTagProcessor aprilTagProcessor;
    private static final Position cameraPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private static final YawPitchRollAngles cameraAngle = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);


    public static void setUpLocalization(HardwareMap hardware, Telemetry tele) {
        hardwareMap = hardware;
        telemetry = tele;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraAngle)
                .build();
    }

    public static void setPose(Pose2D newPose) {
        robotPose = newPose;
    }

    public static Pose2D getPose() {
        return robotPose;
    }

}