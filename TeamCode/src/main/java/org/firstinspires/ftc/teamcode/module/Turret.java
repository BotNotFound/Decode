package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AllianceColor;

@Config
public class Turret {
    public static final String TURRET_MOTOR_NAME = "Turret";
    private final DcMotor turretMotor;

    // TODO tune controller
    public static double kP = 0.25;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    private final PIDFController aimController;

    private final double goalAngle;

    /**
     * The number of encoder ticks in a single revolution of the motor We are currently using a <a
     * href="">5203 goBuilda motor</a>
     */
    public static final double TICKS_PER_REVOLUTION = 537.7;

    public Turret(HardwareMap hardwareMap, AllianceColor allianceColor) {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
        goalAngle = allianceColor.goalAngle;
        aimController = new PIDFController(kP, kI, kD, kF);
    }

    public void aimAtGoal(double curHeading) {
        aimController.setPIDF(kP, kI, kD, kF);
        aimController.setSetPoint(goalAngle);
        turretMotor.setPower(aimController.calculate(curHeading));
    }
}
