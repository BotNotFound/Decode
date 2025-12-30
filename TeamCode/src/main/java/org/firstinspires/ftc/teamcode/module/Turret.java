package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    /**
     * The number of encoder ticks in a single revolution of the motor We are currently using a
     * <a href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder/">
     *     5203 goBilda motor
     * </a>
     * and a 113:12 gear ratio
     */
    public static final double TICKS_PER_REVOLUTION = 145.1 * (113.0 / 12.0);

    public Turret(HardwareMap hardwareMap, AllianceColor allianceColor) {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
        aimController = new PIDFController(kP, kI, kD, kF);
    }

    public void aimAtGoal(double x, double y, double curHeading) {
        aimController.setPIDF(kP, kI, kD, kF);
        aimController.setSetPoint(Math.atan2(y, x));
        turretMotor.setPower(aimController.calculate(curHeading));
    }

    public void resetRotation() {
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(1);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
