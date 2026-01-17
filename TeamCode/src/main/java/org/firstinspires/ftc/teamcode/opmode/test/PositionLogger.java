package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.opmode.teleop.OnePersonTeleOp;

@Config
@TeleOp(group = "test")
public class PositionLogger extends OnePersonTeleOp {
    public static double START_X = 115.5;
    public static double START_Y = 129.4;
    public static double START_HEADING = 36;

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void start() {
        super.start();
        pinpoint = hardwareMap.get(
            GoBildaPinpointDriver.class, FieldCentricDriveTrain.PINPOINT_DRIVER_NAME);
        pinpoint.setPosition(
            new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));
    }

    @Override
    public void loop() {
        super.loop();
        final Pose2D curPose = pinpoint.getPosition();
        telemetry.addData("Current X position", curPose.getX(DistanceUnit.INCH));
        telemetry.addData("Current Y position", curPose.getY(DistanceUnit.INCH));
        telemetry.addData("Current heading", curPose.getHeading(AngleUnit.DEGREES));
    }
}
