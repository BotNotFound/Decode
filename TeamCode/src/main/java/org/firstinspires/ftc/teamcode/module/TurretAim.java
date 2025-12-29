package org.firstinspires.ftc.teamcode.module;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AllianceColor;

public class TurretAim {
    public static final String TURRET_MOTOR_NAME = "Turret";
    private final DcMotor turretMotor;
    private final Telemetry telemetry;

    // TODO: figure out what these need to be
    public static final double RED_GOAL_ANGLE = 0;
    public static final double BLUE_GOAL_ANGLE = 0;
    private double goalAngle = RED_GOAL_ANGLE;
    private static final double POINTS_PER_REVOLUTION = 537.7;
    private static final double TURRET_GEAR_CONVERSION = 1;

    public TurretAim(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
        this.telemetry = telemetry;
        if (allianceColor.targetAprilTagID == 20) {
            goalAngle = BLUE_GOAL_ANGLE;
        }

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(1);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // TODO: motor.isBusy() loop here, im not sure whether to have that in Robot or here
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static double motorToTurretAngle(double motorAngle) {
        return motorAngle * TURRET_GEAR_CONVERSION;
    }
}
