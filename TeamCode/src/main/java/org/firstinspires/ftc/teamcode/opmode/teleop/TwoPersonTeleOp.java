package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "2-Driver Teleop", group = "2driver")
public class TwoPersonTeleOp extends OpMode {
    protected Robot robot;

    /**
     * Should we allow the driver to swap alliance colors in init?
     */
    private final boolean allowAllianceSwap;

    protected TwoPersonTeleOp(boolean allowAllianceSwap) {
        this.allowAllianceSwap = allowAllianceSwap;
    }

    @SuppressWarnings("unused")
    public TwoPersonTeleOp() {
        this(true);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // assume red alliance until driver indicates otherwise
        robot = new Robot(hardwareMap, telemetry, Robot.AllianceColor.RED);
    }

    @Override
    public void init_loop() {
        if (allowAllianceSwap && gamepad1.aWasPressed()) {
            robot.swapAllianceColor();
        }
        telemetry.addData("Current Alliance", robot.getAllianceColor());
    }

    @Override
    public void loop() {
        if (gamepad2.right_trigger > 0.5) {
            robot.setState(Robot.RobotState.SHOOT);
        }
        else if (gamepad2.left_trigger > 0.5) {
            robot.setState(Robot.RobotState.INTAKE);
            if (gamepad1.right_bumper) {
                robot.setHeadingScale(0.5);
            }
            else {
                robot.setHeadingScale(1.0);
            }
        }
        else if (gamepad2.left_bumper) {
            robot.setState(Robot.RobotState.REVERSE_INTAKE);
        }
        else if (gamepad2.right_bumper) {
            robot.setState(Robot.RobotState.PRE_SHOOT);
        }
        else {
            robot.setState(Robot.RobotState.NONE);
            robot.setHeadingScale(1.0);
        }

        robot.loop(gamepad1);

        if (gamepad2.dpadUpWasPressed()) {
            robot.increaseDefaultShooterRPM();
        }
        else if (gamepad2.dpadDownWasPressed()) {
            robot.decreaseDefaultShooterRPM();
        }

        if (gamepad1.start && gamepad1.back) {
            robot.resetOdometry();
        }

        telemetry.update();
    }
}
