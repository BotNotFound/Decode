package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;

public class Constants {
    public static final FollowerConstants followerConstants = new FollowerConstants()
        .mass(11.975)
        .forwardZeroPowerAcceleration(-37.132714179943896)
        .lateralZeroPowerAcceleration(-76.01161208193545)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0.03))
        .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0, 0.05))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.001, 0, 0.03))
        .centripetalScaling(0);

    public static final MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName(FieldCentricDriveTrain.FRONT_RIGHT_DRIVE_MOTOR_NAME)
        .rightRearMotorName(FieldCentricDriveTrain.BACK_RIGHT_DRIVE_MOTOR_NAME)
        .leftRearMotorName(FieldCentricDriveTrain.BACK_LEFT_DRIVE_MOTOR_NAME)
        .leftFrontMotorName(FieldCentricDriveTrain.FRONT_LEFT_DRIVE_MOTOR_NAME)
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(79.27496049535556)
        .yVelocity(61.312163105161176);

    public static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .build();
    }

    public static final PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(FieldCentricDriveTrain.PINPOINT_Y_OFFSET)
        .strafePodX(FieldCentricDriveTrain.PINPOINT_X_OFFSET)
        .distanceUnit(FieldCentricDriveTrain.PINPOINT_OFFSET_UNIT)
        .hardwareMapName(FieldCentricDriveTrain.PINPOINT_DRIVER_NAME)
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        // adjust these in the encoder tests
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
