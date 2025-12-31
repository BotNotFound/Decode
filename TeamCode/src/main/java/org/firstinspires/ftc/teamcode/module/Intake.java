package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    public static final String INTAKE_MOTOR_NAME = "Intake";

    //change speed accordingly in auto or teleop classes
    public static double engagePower = 1;

    private final DcMotor intakeMotor;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void startIntake() {
        intakeMotor.setPower(engagePower);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void reverseIntake(){ intakeMotor.setPower(-engagePower); }
}