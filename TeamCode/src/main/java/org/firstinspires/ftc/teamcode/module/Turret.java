package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.controller.PositionalPIDFController;

@Config
public class Turret {
    public static final String TURRET_MOTOR_NAME = "Turret";
    private final DcMotor turretMotor;

    public static double TURRET_OFFSET_X = -76;
    public static double TURRET_OFFSET_Y = 0;
    public static DistanceUnit TURRET_OFFSET_UNIT = DistanceUnit.MM;

    public static double kP = 0.03;
    public static double kI = 0;
    public static double kD = 0.00125;
    public static double kF = 0.12;
    public static double tolerance = 1;
    public static double feedforwardThreshold = 1;
    private final PositionalPIDFController aimController;

    public static double DEFAULT_TURRET_ROTATION_OFFSET = -15;
    private double turretRotationOffset;

    public static double TURRET_MIN_ROTATION = 0;
    public static double TURRET_MAX_ROTATION = 270;

    private static double normalizeDegrees(double angle) {
        angle %= 360;
        if (angle < (TURRET_MAX_ROTATION - 360) / 2) {
            angle += 360;
        }
        return angle;
    }

    private static double clampToSafeRotation(double angle) {
        angle = normalizeDegrees(angle);
        if (angle < TURRET_MIN_ROTATION) {
            return TURRET_MIN_ROTATION;
        }
        return Math.min(angle, TURRET_MAX_ROTATION);
    }

    /**
     * The number of encoder ticks in a single revolution of the motor. We are currently using a <a
     * href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder/">
     * 5203 1150 RPM goBilda motor
     * </a>
     * and a 126:17 gear ratio
     */
    public static final double TICKS_PER_REVOLUTION = ((1.0 + (46.0 / 11.0)) * 28.0) * (126.0 / 17.0);

    private final Telemetry telemetry;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretRotationOffset = DEFAULT_TURRET_ROTATION_OFFSET;

        aimController = new PositionalPIDFController(kP, kI, kD, kF);
        aimController.setTolerance(tolerance);
        aimController.setFeedforwardThreshold(feedforwardThreshold);
        aimController.setSetPoint(getCurrentHeading(AngleUnit.DEGREES));

        this.telemetry = telemetry;
    }

    /**
     * Gets the absolute pose of the turret in the field
     *
     * @param robotPose The robot's current position and heading
     * @return The turret's absolute position and heading
     */
    public Pose2D getTurretPose(Pose2D robotPose) {
        final double robotX = robotPose.getX(DistanceUnit.INCH);
        final double robotY = robotPose.getY(DistanceUnit.INCH);
        final double robotHeading = robotPose.getHeading(AngleUnit.RADIANS);
        final double turretOffsetX = TURRET_OFFSET_UNIT.toInches(TURRET_OFFSET_X);
        final double turretOffsetY = TURRET_OFFSET_UNIT.toInches(TURRET_OFFSET_Y);
        final double turretOffsetHeading = getCurrentHeading(AngleUnit.RADIANS);
        final double turretX = robotX + FieldCentricDriveTrain.rotX(
            turretOffsetX, turretOffsetY, robotHeading);
        final double turretY = robotY + FieldCentricDriveTrain.rotY(
            turretOffsetX, turretOffsetY, robotHeading);
        final double turretHeading = robotHeading + turretOffsetHeading;
        return new Pose2D(
            DistanceUnit.INCH,
            turretX,
            turretY,
            AngleUnit.RADIANS,
            turretHeading
        );
    }

    /* heading getters and setters use unnormalized units b/c we do our own normalization */

    public double getCurrentHeading(AngleUnit angleUnit) {
        final int currentPosition = turretMotor.getCurrentPosition() * (
            turretMotor.getDirection() == DcMotorSimple.Direction.REVERSE ? -1 : 1
        );
        final double currentHeadingDegrees = normalizeDegrees(
            (currentPosition / TICKS_PER_REVOLUTION * 360) + turretRotationOffset);
        return angleUnit.getUnnormalized().fromDegrees(currentHeadingDegrees);
    }

    public void setCurrentHeading(double heading, AngleUnit unit) {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotationOffset = normalizeDegrees(unit.getUnnormalized().toDegrees(heading));
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        setTargetHeading(
            Math.atan2(y, x) - robotHeadingUnit.toRadians(curRobotHeading), AngleUnit.RADIANS);
    }

    public void update() {
        aimController.setPIDF(kP, kI, kD, kF);
        aimController.setTolerance(tolerance);
        aimController.setFeedforwardThreshold(feedforwardThreshold);
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
