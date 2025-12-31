package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    public static final String TURRET_MOTOR_NAME = "Turret";
    private final DcMotor turretMotor;

    public static double TURRET_MOTOR_POWER = 1.0;

    public static double TURRET_INITIAL_ROTATION = Math.toRadians(70);
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
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetHeading(double targetHeadingRadians) {
        final int targetPosition = (int) ((targetHeadingRadians - TURRET_INITIAL_ROTATION) / (2.0 * Math.PI) * TICKS_PER_REVOLUTION);
        if (targetPosition != turretMotor.getTargetPosition() || turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setTargetPosition(targetPosition);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void aimAtGoal(double x, double y, double curRobotHeading) {
        final double targetHeading = clampToSafeRotation(Math.atan2(y, x) - curRobotHeading);
        setTargetHeading(targetHeading);
        update();
    }

    public void update() {
        if (turretMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setPower(TURRET_MOTOR_POWER);
        }
    }

    public void setPower(double power) {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(power);
    }

    public boolean isReady() {
        return turretMotor.isBusy();
    }
}
