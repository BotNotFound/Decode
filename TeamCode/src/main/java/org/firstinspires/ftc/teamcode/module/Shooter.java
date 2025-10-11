package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter {
    public static final String LOWER_FLYWHEEL_MOTOR_NAME = "Lower flywheel";
    public static final String UPPER_FLYWHEEL_MOTOR_NAME = "Upper flywheel";
    public static final String KICKER_SERVO_NAME = "Kicker";

    public static double KICKER_IDLE_POSITION = 0.82;
    public static double KICKER_LAUNCH_POSITION = 0.35;

    private final DcMotor lowerFlywheelMotor;
    private final DcMotor upperFlywheelMotor;
    private final Servo kickerServo;

    private final Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        lowerFlywheelMotor = hardwareMap.get(DcMotor.class, LOWER_FLYWHEEL_MOTOR_NAME);
        upperFlywheelMotor = hardwareMap.get(DcMotor.class, UPPER_FLYWHEEL_MOTOR_NAME);
        kickerServo = hardwareMap.get(Servo.class, KICKER_SERVO_NAME);

        lowerFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        upperFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        kickerServo.setPosition(KICKER_IDLE_POSITION);

        this.telemetry = telemetry;
    }

    public void setPower(double power) {
        lowerFlywheelMotor.setPower(power);
        upperFlywheelMotor.setPower(power);

        telemetry.addData("Shooter Power", power);
    }

    public void engageKicker() {
        kickerServo.setPosition(KICKER_LAUNCH_POSITION);

        telemetry.addData("Kicker Position", "engaged");
    }

    public void disengageKicker() {
        kickerServo.setPosition(KICKER_IDLE_POSITION);

        telemetry.addData("Kicker Position", "disengaged");
    }
}
