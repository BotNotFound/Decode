package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "test")
@Config
public class TurretTest extends OpMode {
    private final String TURRET_GEAR_MOTOR = "Turret Gear Motor";

    private final String SHOOTER_MOTOR_LEFT = "Shooter Motor Left";

    private final String SHOOTER_MOTOR_RIGHT = "Shooter Motor Right";

    private DcMotorEx leftFlyWheelMotor;

    private DcMotorEx rightFlyWheelMotor;

    private DcMotor turretMotor;


    public static double turretSpinPower = 0.1;

    public static double shooterRPM = 3000;


    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_GEAR_MOTOR);

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Press X to rotate turret");
        telemetry.addLine("Press Y to stop turret");
        telemetry.addLine("Press LB to switch turret rotation direction");


    }

    @Override
    public void loop() {

        if (gamepad1.xWasPressed()) {
            turretMotor.setPower(turretSpinPower);
        }
        else if (gamepad1.yWasPressed()) {
            turretMotor.setPower(0);
        }
        else if (gamepad1.leftBumperWasPressed()) {

            if (turretMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (turretMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
                turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

        }
        int ticks = turretMotor.getCurrentPosition();
        double tickPerRev = 145.1 * (113.0 / 12.0);
        double degrees = (ticks / tickPerRev) * 360.0;
        telemetry.addData("Heading: ", degrees);
        telemetry.update();


    }
}
