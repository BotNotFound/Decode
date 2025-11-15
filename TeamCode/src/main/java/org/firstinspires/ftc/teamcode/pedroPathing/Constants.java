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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;

public class Constants {
    public static final FollowerConstants followerConstants = new FollowerConstants()
        .mass(9.162566)
        .forwardZeroPowerAcceleration(-36.278554120599594)
        .lateralZeroPowerAcceleration(-74.29537018748395)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.066, 0, 0, 0.02))
        .headingPIDFCoefficients(new PIDFCoefficients(0.866, 0, 0, 0.03))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0, 0, 0.01))
        .centripetalScaling(0.0009);

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
            .xVelocity(61.33050344872662)
            .yVelocity(50.51176969272884);

    public static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static final PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(32)
            .strafePodX(8)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(FieldCentricDriveTrain.PINPOINT_DRIVER_NAME)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // adjust these in the encoder tests
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
