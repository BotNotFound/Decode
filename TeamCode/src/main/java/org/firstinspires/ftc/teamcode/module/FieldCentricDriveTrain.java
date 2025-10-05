package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

@Config
public class FieldCentricDriveTrain extends DriveTrain {
    private static final double DEGREES_TO_RADIANS = 180.0 / Math.PI;

    public static final String IMU_NAME = "imu";

    private final IMU imu;

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public FieldCentricDriveTrain(HardwareMap hardwareMap) {
        super(hardwareMap);

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                LOGO_FACING_DIRECTION,
                usbFacingDirection
        )));
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    @Override
    public void setPower(double drive, double strafe, double turn) {
        final double curRotation = imu.getRobotYawPitchRollAngles().getYaw() * DEGREES_TO_RADIANS;

        super.setPower(
                drive * Math.cos(curRotation) + strafe * Math.sin(curRotation),
                drive * Math.sin(curRotation) + strafe * Math.cos(curRotation),
                turn
        );
    }
}
