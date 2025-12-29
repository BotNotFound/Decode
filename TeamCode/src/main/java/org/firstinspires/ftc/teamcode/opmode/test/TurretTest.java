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
        leftFlyWheelMotor= hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_LEFT);

        rightFlyWheelMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_RIGHT);

        turretMotor = hardwareMap.get(DcMotor.class, TURRET_GEAR_MOTOR);

        leftFlyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Press A to start shooter");
        telemetry.addLine("Press B to stop shooter");
        telemetry.addLine("Press X to rotate turret");
        telemetry.addLine("Press Y to stop turret");
        telemetry.addLine("Press LB to switch turret rotation direction");



    }

    @Override
    public void loop() {

        if(gamepad1.dpadUpWasPressed()){
            shooterRPM += 50;
        }
        else if(gamepad1.dpadDownWasPressed()){
            shooterRPM -= 50;
        }
        else if(gamepad1.aWasPressed()){
            leftFlyWheelMotor.setPower(shooterRPM);
            rightFlyWheelMotor.setPower(shooterRPM);
        }
        else if(gamepad1.bWasPressed()){
            leftFlyWheelMotor.setPower(0);
            rightFlyWheelMotor.setPower(0);
        }
        else if(gamepad1.xWasPressed()){
            if(!leftFlyWheelMotor.isBusy() || !rightFlyWheelMotor.isBusy()){
                turretMotor.setPower(turretSpinPower);
            }
        }
        else if(gamepad1.yWasPressed()){
            turretMotor.setPower(0);
        }
        else if(gamepad1.leftBumperWasPressed()){

            if(turretMotor.getDirection() == DcMotorSimple.Direction.FORWARD){
                turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(turretMotor.getDirection() == DcMotorSimple.Direction.REVERSE){
                turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

        }


    }
}
