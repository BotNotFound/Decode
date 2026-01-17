package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.opmode.teleop.OnePersonTeleOp;

@Config
@TeleOp(group = "test")
public class ShooterTuningTeleOp extends OnePersonTeleOp {
    public static double shooterRPM = 2900;
    public static double hoodPosition = Shooter.HOOD_SERVO_MIN_POSITION;

    public ShooterTuningTeleOp() {
        super(true, true);
    }

    @Override
    public void loop() {
        if (gamepad1.leftStickButtonWasPressed()) {
            robot.setRobotPose(Robot.getDefaultRobotPose());
        }

        final Pose2D robotPose = robot.getRobotPose();
        final double goalX = robot.getAllianceColor().goalPositionX;
        final double goalY = robot.getAllianceColor().goalPositionY;
        telemetry.addData(
            "Distance to goal", Math.sqrt(
                Math.pow(goalX - robotPose.getX(DistanceUnit.INCH), 2) +
                    Math.pow(goalY - robotPose.getY(DistanceUnit.INCH), 2)
            )
        );
        robot.setFallbackShooterRPM(shooterRPM);
        robot.setFallbackHoodPosition(hoodPosition);
        super.loop();
    }
}
