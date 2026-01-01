package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    public static final String TURRET_MOTOR_NAME = "Turret";
    private final DcMotor turretMotor;

    // TODO tune controller
    public static double kP = 0.025;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static double tolerance = 1;
    private final PIDFController aimController;

    public static double TURRET_ROTATION_OFFSET = Math.toRadians(70);
    public static double TURRET_MIN_ROTATION = Math.PI / 2;
    public static double TURRET_MAX_ROTATION = Math.PI * 3 / 2;

    private static double normalizeRadiansPositive(double angle) {
        while (angle >= Math.PI * 2) {
            angle -= Math.PI * 2;
        }
        while (angle < 0) {
            angle += Math.PI * 2;
        }
        return angle;
    }

    private static double clampToSafeRotation(double angle) {
        angle = normalizeRadiansPositive(angle);
        if (angle < TURRET_MIN_ROTATION) {
            return TURRET_MIN_ROTATION;
        }
        return Math.min(angle, TURRET_MAX_ROTATION);
    }

    /**
     * The number of encoder ticks in a single revolution of the motor We are currently using a
     * <a
     * href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder/">
     * 5203 goBilda motor
     * </a>
     * and a 113:12 gear ratio
     */
    public static final double TICKS_PER_REVOLUTION = 145.1 * (113.0 / 12.0);

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aimController = new PIDFController(kP, kI, kD, kF);


    }

    public double getCurrentHeading() {
        final int currentPosition = turretMotor.getCurrentPosition();
        return (currentPosition / TICKS_PER_REVOLUTION * (2.0 * Math.PI)) + TURRET_ROTATION_OFFSET;
    }

    public void setTargetHeading(double targetHeadingRadians) {
        final double safeHeadingRadians = clampToSafeRotation(targetHeadingRadians);
        aimController.setSetPoint(safeHeadingRadians);
    }

    public void aimAtGoal(double x, double y, double curRobotHeading) {
        setTargetHeading(Math.atan2(y, x) - curRobotHeading);
    }

    public void update() {
        aimController.setPIDF(kP, kI, kD, kF);
        aimController.setTolerance(tolerance);
        turretMotor.setPower(aimController.calculate(getCurrentHeading()));
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public boolean isReady() {
        return aimController.atSetPoint();
    }
}
