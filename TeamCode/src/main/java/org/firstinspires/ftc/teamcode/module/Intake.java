package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    public static final String INTAKE_MOTOR_NAME = "Intake";
    public static final String COLOR_SENSOR_NAME = "IntakeColor";

    //change speed accordingly in auto or teleop classes
    public static double engagePower = 1;

    private final DcMotor intakeMotor;
    private final RevColorSensorV3 colorSensor;

    private final Telemetry telemetry;

    public Intake(HardwareMap hwMap, Telemetry tele) {
        intakeMotor = hwMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hwMap.get(RevColorSensorV3.class, COLOR_SENSOR_NAME);

        telemetry = tele;
    }
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public boolean hasBall() {
        return colorSensor.getDistance(DistanceUnit.CM) <= 5;
    }

    public void startIntake() {
        intakeMotor.setPower(engagePower);
        telemetry.addData("CS Distance" ,colorSensor.getDistance(DistanceUnit.CM));
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
