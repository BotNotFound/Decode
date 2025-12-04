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
    public static double START_POSITION_X = 119.5;
    public static double START_POSITION_Y = 133;
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static double START_HEADING = 35;
    public static AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void start() {
        super.start();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, FieldCentricDriveTrain.PINPOINT_DRIVER_NAME);
        pinpoint.setOffsets(START_POSITION_X, START_POSITION_Y, DISTANCE_UNIT);
        pinpoint.setHeading(START_HEADING, ANGLE_UNIT);
    }

    @Override
    public void loop() {
        super.loop();
        final Pose2D curPose = pinpoint.getPosition();
        telemetry.addData("Current X position", curPose.getX(DISTANCE_UNIT));
        telemetry.addData("Current Y position", curPose.getY(DISTANCE_UNIT));
        telemetry.addData("Current heading", curPose.getHeading(ANGLE_UNIT));
    }
}
