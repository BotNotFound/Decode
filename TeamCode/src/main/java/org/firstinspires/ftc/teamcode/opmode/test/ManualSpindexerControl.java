package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.module.Spindexer;

@TeleOp(group = "test")
@Config
public class ManualSpindexerControl extends OpMode {
    /**
     * How far the gamepad joystick has to be from the center before its input should count as
     * rotating the spindexer.  Should be between 0 and 1, exclusive
     */
    public static double MIN_AIM_ACTIVATION_MAGNITUDE = 0.7;

    private Spindexer spindexer;

    private boolean manualPowerControl = false;
    private double targetAngle = 0;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        telemetry.addLine("Toggle control modes with A");
        if (gamepad1.aWasReleased()) {
            manualPowerControl = !manualPowerControl;
        }

        if (manualPowerControl) {
            telemetry.addLine("Power Control Enabled");
            telemetry.addLine("Use the right joystick to set the raw spindexer power");
            spindexer.setPower(gamepad1.right_stick_x);
            return;
        }

        // auto-aim mode
        telemetry.addLine("Rotate the spindexer by pointing the left joystick");

        final double x = gamepad1.left_stick_x;
        final double y = gamepad1.left_stick_y;

        final double magnitude = Math.sqrt((x * x) + (y * y));
        if (magnitude >= MIN_AIM_ACTIVATION_MAGNITUDE) {
            targetAngle = Math.atan2(y, x);
        }

        spindexer.rotateToAngle(targetAngle);
    }
}
