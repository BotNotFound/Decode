package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.SquIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
public class FieldCentricDriveTrain {
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

    private final SquIDController turnController;
    public static double turnP = 0.025;


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

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_DRIVER_NAME);
        pinpointDriver.setOffsets(8, 32, DistanceUnit.MM);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        turnController = new SquIDController(turnP);
    }

    public void resetOdometry() {
        pinpointDriver.resetPosAndIMU();
    }

    public void setPowerFacingAprilTag(double drive, double strafe, double turn, AprilTagPoseFtc targetTag) {

        if(targetTag != null){
            turn = getAimRotationPower(targetTag.bearing);
            telemetry.addData("turn power", turn);
        }

        setPower(drive, strafe, turn);
    }

    public void setPower(double drive, double strafe, double turn) {
        pinpointDriver.update();

        drive = Math.pow(drive, 3);
        strafe = Math.pow(strafe, 3);

        double curRotation = pinpointDriver.getHeading(AngleUnit.RADIANS);
        telemetry.addData("Current Angle", curRotation);

        double rotDrive = drive * Math.cos(curRotation) - strafe * Math.sin(curRotation);
        double rotStrafe = drive * Math.sin(curRotation) + strafe * Math.cos(curRotation);

        telemetry.addData("Rotated Drive", rotDrive);
        telemetry.addData("Rotated Strafe", rotDrive);

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
        turnController.setP(turnP);
        return turnController.calculate(bearing);
    }

}
