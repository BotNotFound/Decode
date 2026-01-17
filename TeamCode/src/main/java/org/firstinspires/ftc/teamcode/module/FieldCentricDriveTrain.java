package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class FieldCentricDriveTrain {
    // Offsets are based on center of turret, not center of robot
    public static double PINPOINT_X_OFFSET = -194;
    public static double PINPOINT_Y_OFFSET = 96;
    public static DistanceUnit PINPOINT_OFFSET_UNIT = DistanceUnit.MM;
    public static double PINPOINT_ENCODER_RESOLUTION = 19.89436789f * (2435.0 / 2446.20483398438);
    public static DistanceUnit PINPOINT_ENCODER_RESOLUTION_UNIT = DistanceUnit.MM;

    public static final String FRONT_RIGHT_DRIVE_MOTOR_NAME = "Front Right";
    public static final String FRONT_LEFT_DRIVE_MOTOR_NAME = "Front Left";
    public static final String BACK_RIGHT_DRIVE_MOTOR_NAME = "Back Right";
    public static final String BACK_LEFT_DRIVE_MOTOR_NAME = "Back Left";

    private final DcMotor frontRightDriveMotor;
    private final DcMotor frontLeftDriveMotor;
    private final DcMotor backRightDriveMotor;
    private final DcMotor backLeftDriveMotor;

    private final Telemetry telemetry;

    public static final String PINPOINT_DRIVER_NAME = "Pinpoint";

    private final GoBildaPinpointDriver pinpointDriver;

    private double fieldCentricHeadingOffset = 0;

    public FieldCentricDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_DRIVE_MOTOR_NAME);
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_DRIVE_MOTOR_NAME);
        backRightDriveMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_DRIVE_MOTOR_NAME);
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_DRIVE_MOTOR_NAME);

        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_DRIVER_NAME);
        pinpointDriver.setOffsets(PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET, PINPOINT_OFFSET_UNIT);
        pinpointDriver.setEncoderResolution(
            PINPOINT_ENCODER_RESOLUTION, PINPOINT_ENCODER_RESOLUTION_UNIT);
        pinpointDriver.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpointDriver.resetPosAndIMU();
    }

    public void resetOdometry() {
        pinpointDriver.resetPosAndIMU();
    }

    public void resetFieldCentricHeading() {
        fieldCentricHeadingOffset = pinpointDriver.getHeading(AngleUnit.RADIANS);
    }

    private double getFieldCentricHeading() {
        return pinpointDriver.getHeading(AngleUnit.RADIANS) - fieldCentricHeadingOffset;
    }

    public static double rotX(double x, double y, double angle) {
        return x * Math.cos(angle) - y * Math.sin(angle);
    }

    public static double rotY(double x, double y, double angle) {
        return x * Math.sin(angle) + y * Math.cos(angle);
    }

    //keep the set power for drivetrain
    public void setPower(double drive, double strafe, double turn) {
        pinpointDriver.update();

        drive = Math.pow(drive, 3);
        strafe = Math.pow(strafe, 3);

        double curRotation = getFieldCentricHeading();

        double rotDrive = rotX(drive, strafe, curRotation);
        double rotStrafe = rotY(drive, strafe, curRotation);

        double leftFrontPower = rotDrive + rotStrafe + turn;
        double leftBackPower = rotDrive - rotStrafe + turn;
        double rightFrontPower = rotDrive - rotStrafe - turn;
        double rightBackPower = rotDrive + rotStrafe - turn;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        frontRightDriveMotor.setPower(rightFrontPower);
        frontLeftDriveMotor.setPower(leftFrontPower);
        backRightDriveMotor.setPower(rightBackPower);
        backLeftDriveMotor.setPower(leftBackPower);
    }

    public Pose2D getRobotPose() {
        return pinpointDriver.getPosition();
    }

    public void setRobotPose(Pose2D pose) {
        pinpointDriver.setPosition(pose);
    }

    /* get current powers using forward kinematics -- see https://www.desmos.com/calculator/je1clj0udl */

    private double getRobotCentricDrivePower() {
        final double frontLeftPower = frontLeftDriveMotor.getPower();
        final double frontRightPower = frontRightDriveMotor.getPower();
        return (frontLeftPower + frontRightPower) / 2;
    }

    private double getRobotCentricStrafePower() {
        final double frontLeftPower = frontLeftDriveMotor.getPower();
        final double backLeftPower = backLeftDriveMotor.getPower();
        return (frontLeftPower - backLeftPower) / 2;
    }

    public double getDrivePower() {
        final double robotCentricDrivePower = getRobotCentricDrivePower();
        final double robotCentricStrafePower = getRobotCentricStrafePower();
        final double curRotation = pinpointDriver.getHeading(
            AngleUnit.RADIANS) + fieldCentricHeadingOffset;
        return robotCentricDrivePower * Math.cos(curRotation) + robotCentricStrafePower * Math.sin(
            curRotation);
    }

    public double getStrafePower() {
        final double robotCentricDrivePower = getRobotCentricDrivePower();
        final double robotCentricStrafePower = getRobotCentricStrafePower();
        final double curRotation = pinpointDriver.getHeading(
            AngleUnit.RADIANS) + fieldCentricHeadingOffset;
        return robotCentricStrafePower * Math.cos(curRotation) - robotCentricDrivePower * Math.sin(
            curRotation);
    }

    public double getTurnPower() {
        final double backLeftPower = backLeftDriveMotor.getPower();
        final double frontRightPower = frontRightDriveMotor.getPower();
        return (backLeftPower - frontRightPower) / 2;
    }

    public void setDrivePower(double power) {
        setPower(power, getStrafePower(), getTurnPower());
    }

    public void setStrafePower(double power) {
        setPower(getDrivePower(), power, getTurnPower());
    }

    public void setTurnPower(double power) {
        setPower(getDrivePower(), getStrafePower(), power);
    }

    public void logInfo() {
        final Pose2D robotPose = getRobotPose();
        final double robotX = robotPose.getX(DistanceUnit.INCH);
        final double robotY = robotPose.getY(DistanceUnit.INCH);
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading", robotPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Field Centric Heading", getFieldCentricHeading());
    }
}