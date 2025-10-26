package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TeleOpRobot;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TwoPersonTeleOp extends OpMode {    
    private TeleOpRobot robot;

    private boolean intakeToggle = false;

    @Override
    public void init() {
        // assume red alliance until driver indicates otherwise
        robot = new TeleOpRobot(hardwareMap, telemetry, TeleOpRobot.AllianceColor.RED);
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            robot.swapAllianceColor();
        }
        telemetry.addData("Current Alliance", robot.getAllianceColor());
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            robot.setState(TeleOpRobot.RobotState.SHOOT);
        }
        else if (gamepad2.leftBumperWasReleased()) {
            if (!intakeToggle) {
                robot.setState(TeleOpRobot.RobotState.INTAKE);
                intakeToggle = true;
                telemetry.addData("LBumper", "Enter. !Intake");
            }
            else {
                robot.setState(TeleOpRobot.RobotState.NONE);
                intakeToggle = false;
                telemetry.addData("LBumper", "Enter. Intake");
            }
        }
        else if (gamepad2.left_trigger > 0.5) {
            robot.setState(TeleOpRobot.RobotState.REVERSE_INTAKE);
        }
        else if (robot.getState() == TeleOpRobot.RobotState.SHOOT || gamepad1.left_trigger > 0) {
            robot.setState(TeleOpRobot.RobotState.NONE);
            intakeToggle = false;
        }

        robot.loop(gamepad1, gamepad2);

        if (gamepad2.dpadUpWasPressed()) {
            robot.increaseDefaultShooterRPM();
        }
        else if (gamepad2.dpadDownWasPressed()) {
            robot.decreaseDefaultShooterRPM();
        }

        if (gamepad1.startWasPressed()) {
            robot.resetOdometry();
        }

        telemetry.update();
    }
}
