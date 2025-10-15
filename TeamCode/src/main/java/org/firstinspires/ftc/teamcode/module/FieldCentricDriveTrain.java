package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
public class FieldCentricDriveTrain{
    public static final String FRONT_RIGHT_DRIVE_MOTOR_NAME = "Front Right";
    public static final String FRONT_LEFT_DRIVE_MOTOR_NAME = "Front Left";
    public static final String BACK_RIGHT_DRIVE_MOTOR_NAME = "Back Right";
    public static final String BACK_LEFT_DRIVE_MOTOR_NAME = "Back Left";

    private final DcMotor frontRightDriveMotor;
    private final DcMotor frontLeftDriveMotor;
    private final DcMotor backRightDriveMotor;
    private final DcMotor backLeftDriveMotor;

    private final Telemetry telemetry;

    public static final String IMU_NAME = "imu";

    private final IMU imu;

    private AprilTagDetector detector;

    private final PIDController turnController;
    public static double turnP = 0.04;
    public static double turnD = 0.001;

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

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

        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                LOGO_FACING_DIRECTION,
                usbFacingDirection
        )));

        turnController = new PIDController(turnP, 0, turnD, 0, 0);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void setPower(double drive, double strafe, double turn, AprilTagPoseFtc targetTag) {
        if(targetTag != null){
            turn = getAimRotationPower(targetTag.bearing);
            telemetry.addData("turn power", turn);
        }

        setPower(drive, strafe, turn);
    }

    public void setPower(double drive, double strafe, double turn) {
        double curRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("bot angle", curRotation);

        double rotDrive = drive * Math.cos(curRotation) + strafe * Math.sin(curRotation);
        double rotStrafe = drive * Math.sin(curRotation) + strafe * Math.cos(curRotation);

        telemetry.addData("Rotated Drive", rotDrive);
        telemetry.addData("RotatedStrafe", rotDrive);

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

    public double getAimRotationPower(double bearing){
        telemetry.addData("Bearing", bearing);
        return turnController.calculate(bearing);
    }

}
