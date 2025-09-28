package org.firstinspires.ftc.teamcode.module;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    public static final String LOWER_FLYWHEEL_MOTOR_NAME = "Lower flywheel";
    public static final String UPPER_FLYWHEEL_MOTOR_NAME = "Upper flywheel";

    private final DcMotor lowerFlywheelMotor;
    private final DcMotor upperFlywheelMotor;

    public Flywheel(HardwareMap hardwareMap) {
        lowerFlywheelMotor = hardwareMap.get(DcMotor.class, LOWER_FLYWHEEL_MOTOR_NAME);
        upperFlywheelMotor = hardwareMap.get(DcMotor.class, UPPER_FLYWHEEL_MOTOR_NAME);

        lowerFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lowerFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        lowerFlywheelMotor.setPower(power);
        upperFlywheelMotor.setPower(power);
    }
}
