package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "1-Driver Teleop", group = "1driver")
public class OnePersonTeleOp extends OpMode {
    protected Robot robot;

    /**
     * Should we allow the driver to swap alliance colors in init?
     */
    private final boolean allowAllianceSwap;

    /**
     * If true, use robot states intended for tuning instead of their standard counterparts when
     * possible
     */
    private final boolean tuning;

    protected OnePersonTeleOp(boolean allowAllianceSwap, boolean tuning) {
        this.allowAllianceSwap = allowAllianceSwap;
        this.tuning = tuning;
    }

    protected OnePersonTeleOp(boolean allowAllianceSwap) {
        this(allowAllianceSwap, false);
    }

    @SuppressWarnings("unused") // required for opmode annotations to work
    public OnePersonTeleOp() {
        this(true);
    }

    @Override
    public void init() {
        // assume red alliance until driver indicates otherwise
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry, AllianceColor.RED);
    }

    @Override
    public void init_loop() {
        if (allowAllianceSwap && gamepad1.aWasPressed()) {
            robot.swapAllianceColor();
        }
        telemetry.addData("Current Alliance", robot.getAllianceColor());

        robot.logInfo();
    }

    @Override
    public void start() {
        robot.tryLoadPersistentState();
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.5) {
            robot.setState(tuning ? Robot.RobotState.MANUAL_SHOOT : Robot.RobotState.SHOOT);
        }
        else if (gamepad1.left_trigger > 0.5) {
            robot.setState(Robot.RobotState.INTAKE);
        }
        else if (gamepad1.left_bumper) {
            robot.setState(Robot.RobotState.REVERSE_INTAKE);
        }
        else if (gamepad1.right_bumper) {
            robot.setState(tuning ? Robot.RobotState.MANUAL_PRE_SHOOT : Robot.RobotState.PRE_SHOOT);
        }
        else if (gamepad1.yWasPressed()) {
            if (robot.getState() != Robot.RobotState.PARK) {
                robot.setState(Robot.RobotState.PARK);
            }
            else {
                robot.setState(Robot.RobotState.NONE);
            }
        }
        else if (robot.getState() != Robot.RobotState.PARK) {
            robot.setState(Robot.RobotState.NONE);
        }

        robot.loop(gamepad1);

        if (gamepad1.dpadUpWasPressed()) {
            robot.increaseFallbackShooterRPM();
        }
        else if (gamepad1.dpadDownWasPressed()) {
            robot.decreaseFallbackShooterRPM();
        }

        if (gamepad1.dpadRightWasPressed()) {
            robot.rotateSpindexerToNextSlot();
        }
        else if (gamepad1.dpadLeftWasPressed()) {
            robot.rotateSpirotateToPreviousSlot();
        }

        telemetry.addData("default shooter rpm", robot.getFallbackShooterRPM());

        if (gamepad1.start && gamepad1.back) {
            robot.resetFieldCentricHeading();
        }
    }
}
