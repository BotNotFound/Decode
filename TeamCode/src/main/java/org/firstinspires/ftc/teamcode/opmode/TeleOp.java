package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TeleOpRobot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    public static double targetRPM = 3000;

    private boolean isIntakeActive = false;

    private TeleOpRobot robot;

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
        if (gamepad1.right_bumper) {
            robot.setState(TeleOpRobot.RobotState.SHOOT);
        }
        else if (gamepad1.leftBumperWasReleased()) {
            if (!isIntakeActive) {
                robot.setState(TeleOpRobot.RobotState.INTAKE);
                isIntakeActive = true;
                telemetry.addData("LBumper", "Enter. !Intake");
            }
            else {
                robot.setState(TeleOpRobot.RobotState.NONE);
                isIntakeActive = false;
                telemetry.addData("LBumper", "Enter. Intake");
            }
        }
        else if (gamepad1.left_trigger > 0.5) {
            robot.setState(TeleOpRobot.RobotState.REVERSE_INTAKE);
        }
        else if (robot.getState() == TeleOpRobot.RobotState.SHOOT || gamepad1.left_trigger > 0) {
            robot.setState(TeleOpRobot.RobotState.NONE);
            isIntakeActive = false;
        }

        robot.loop(gamepad1, gamepad2);

        if (gamepad1.dpadUpWasPressed()) {
            robot.increaseDefaultShooterRPM();
        }
        else if (gamepad1.dpadDownWasPressed()) {
            robot.decreaseDefaultShooterRPM();
        }

        telemetry.addData("target shooter rpm", targetRPM);

        if (gamepad1.startWasPressed()) {
            robot.resetOdometry();
        }

        telemetry.update();
    }
}
