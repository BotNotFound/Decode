package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    public static double TURRET_ROTATION_OFFSET = 70;
    public static double TURRET_MIN_ROTATION = 90;
    public static double TURRET_MAX_ROTATION = 270;

    private static double normalizeDegreesPositive(double angle) {
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    private static double clampToSafeRotation(double angle) {
        angle = normalizeDegreesPositive(angle);
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

    private final Telemetry telemetry;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aimController = new PIDFController(kP, kI, kD, kF);
        aimController.setSetPoint(getCurrentHeading(AngleUnit.DEGREES));

        this.telemetry = telemetry;
    }

    public double getCurrentHeading(AngleUnit angleUnit) {
        final int currentPosition = turretMotor.getCurrentPosition();
        return angleUnit.fromDegrees((currentPosition / TICKS_PER_REVOLUTION * 360) + TURRET_ROTATION_OFFSET);
    }

    public double getTargetHeading(AngleUnit angleUnit) {
        return angleUnit.getUnnormalized().fromDegrees(aimController.getSetPoint());
    }

    public void setTargetHeading(double targetHeading, AngleUnit angleUnit) {
        final double targetHeadingDegrees = angleUnit.getUnnormalized().toDegrees(targetHeading);
        final double safeHeadingDegrees = clampToSafeRotation(targetHeadingDegrees);
        aimController.setSetPoint(safeHeadingDegrees);
    }

    public void aimAtGoal(double x, double y, double curRobotHeading, AngleUnit robotHeadingUnit) {
        setTargetHeading(Math.atan2(y, x) - robotHeadingUnit.toRadians(curRobotHeading), AngleUnit.RADIANS);
    }

    public void update() {
        aimController.setPIDF(kP, kI, kD, kF);
        aimController.setTolerance(tolerance);
        turretMotor.setPower(aimController.calculate(getCurrentHeading(AngleUnit.DEGREES)));
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public boolean isReady() {
        return aimController.atSetPoint();
    }

    public void logInfo() {
        telemetry.addData("Turret Current Heading (degrees)", getCurrentHeading(AngleUnit.DEGREES));
        telemetry.addData("Turret Target Heading (degrees)", getTargetHeading(AngleUnit.DEGREES));
    }
}
