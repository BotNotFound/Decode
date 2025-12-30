package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AllianceColor;

@Config
public class Turret {
    public static final String TURRET_MOTOR_NAME = "Turret";
    private final DcMotor turretMotor;

    public static double TURRET_MOTOR_POWER = 1.0;

    /**
     * The number of encoder ticks in a single revolution of the motor We are currently using a
     * <a href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder/">
     *     5203 goBilda motor
     * </a>
     * and a 113:12 gear ratio
     */
    public static final double TICKS_PER_REVOLUTION = 145.1 * (113.0 / 12.0);

    public Turret(HardwareMap hardwareMap, AllianceColor allianceColor) {
        turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void aimAtGoal(double x, double y, double curHeading) {
        final double targetHeading = Math.atan2(y, x) - curHeading;
        final int targetPosition = (int) (targetHeading / (2.0 * Math.PI) * TICKS_PER_REVOLUTION);
        turretMotor.setTargetPosition(targetPosition);
        update();
    }

    public void update() {
        turretMotor.setPower(TURRET_MOTOR_POWER);
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isReady() {
        return turretMotor.isBusy();
    }
}
