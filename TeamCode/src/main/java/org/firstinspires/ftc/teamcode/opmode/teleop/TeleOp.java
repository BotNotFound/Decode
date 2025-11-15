package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "1-Driver Teleop", group = "1driver")
public class TeleOp extends OpMode {
    public static double targetRPM = 3000;

    protected Robot robot;

    /**
     * Should we allow the driver to swap alliance colors in init?
     */
    private final boolean allowAllianceSwap;

    protected TeleOp(boolean allowAllianceSwap) {
        this.allowAllianceSwap = allowAllianceSwap;
    }
    public TeleOp() {
        this(true);
    }

    @Override
    public void init() {
        // assume red alliance until driver indicates otherwise
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        if (gamepad1.right_bumper) {
            robot.setState(Robot.RobotState.SHOOT);
        }
        else if (gamepad1.left_bumper) {
            robot.setState(Robot.RobotState.INTAKE);
        }
        else if (gamepad1.left_trigger > 0.5) {
            robot.setState(Robot.RobotState.REVERSE_INTAKE);
        }
        else {
            robot.setState(Robot.RobotState.NONE);
        }

        robot.loop(gamepad1);

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
