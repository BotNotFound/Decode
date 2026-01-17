package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "2-Driver Teleop", group = "2driver")
@Disabled
@Config
public class TwoPersonTeleOp extends OpMode {
    protected Robot robot;

    public static double TURRET_OFFSET_SCALE = 5;

    /**
     * Should we allow the driver to swap alliance colors in init?
     */
    private final boolean allowAllianceSwap;

    /**
     * If true, use robot states intended for manual control instead of their standard counterparts
     * when possible
     */
    public static boolean FORCE_FALLBACK = false;

    protected TwoPersonTeleOp(boolean allowAllianceSwap) {
        this.allowAllianceSwap = allowAllianceSwap;
    }

    @SuppressWarnings("unused") // required for opmode annotations to work
    public TwoPersonTeleOp() {
        this(true);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // assume red alliance until driver indicates otherwise
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
        robot.setTurretAimOffset(gamepad2.right_stick_x * TURRET_OFFSET_SCALE);

        if (gamepad2.right_trigger > 0.5) {
            robot.setState(FORCE_FALLBACK ? Robot.RobotState.MANUAL_SHOOT : Robot.RobotState.SHOOT);
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
            robot.setState(
                FORCE_FALLBACK ? Robot.RobotState.MANUAL_PRE_SHOOT : Robot.RobotState.PRE_SHOOT);
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
            robot.setHeadingScale(1.0);
        }

        robot.loop(gamepad1);

        if (gamepad2.dpadUpWasPressed()) {
            robot.increaseFallbackShooterRPM();
        }
        else if (gamepad2.dpadDownWasPressed()) {
            robot.decreaseFallbackShooterRPM();
        }

        if (gamepad2.dpadRightWasPressed()) {
            robot.rotateSpindexerToNextSlot();
        }
        else if (gamepad2.dpadLeftWasPressed()) {
            robot.rotateSpindexerToPreviousSlot();
        }

        if (gamepad1.start && gamepad1.back) {
            robot.resetFieldCentricHeading();
        }

        if (gamepad1.guideWasReleased()) {
            robot.resetRobotPosition();
        }
    }

    @Override
    public void stop() {
        robot.savePersistentState();
    }
}
