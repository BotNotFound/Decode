package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.module.Turret;

@TeleOp(group = "test")
@Config
public class ManualTurretControl extends OpMode {
    /**
     * How far the gamepad joystick has to be from the center before its input should count as
     * rotating the turret.  Should be between 0 and 1, exclusive
     */
    public static double MIN_AIM_ACTIVATION_MAGNITUDE = 0.7;

    private Turret turret;

    private boolean manualPowerControl;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, telemetry);
        manualPowerControl = false;
    }

    @Override
    public void loop() {
        telemetry.addLine("Toggle control modes with A");
        if (gamepad1.aWasReleased()) {
            manualPowerControl = !manualPowerControl;
        }

        if (manualPowerControl) {
            telemetry.addData("Current Mode", "Power Control");
            telemetry.addLine("Use the right joystick to set the raw turret power");
            turret.setPower(gamepad1.right_stick_x);
            return;
        }

        // auto-aim mode
        telemetry.addData("Current Mode", "Heading Control");
        telemetry.addData("Current Rotation", turret.getCurrentHeading(AngleUnit.DEGREES));
        telemetry.addLine("Rotate the turret by pointing the left joystick");

        final double x = gamepad1.left_stick_x;
        final double y = gamepad1.left_stick_y;

        final double magnitude = Math.sqrt((x * x) + (y * y));
        if (magnitude >= MIN_AIM_ACTIVATION_MAGNITUDE) {
            turret.setTargetHeading(Math.atan2(y, x), AngleUnit.RADIANS);
        }

        turret.update();
    }
}
