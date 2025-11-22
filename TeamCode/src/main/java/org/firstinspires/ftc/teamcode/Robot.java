package org.firstinspires.ftc.teamcode;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
public class Robot {
    public static final String TAG = "Robot";

    public enum AllianceColor {
        RED(24),
        BLUE(20);

        public final int targetAprilTagID;

        AllianceColor(int targetAprilTagID) {
            this.targetAprilTagID = targetAprilTagID;
        }


        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case RED:
                    return "Red Alliance";
                case BLUE:
                    return "Blue Alliance";
                default:
                    return super.toString();
            }
        }
    }

    public enum RobotState {
        INTAKE,
        REVERSE_INTAKE,
        PRE_SHOOT,
        SHOOT,
        NONE,
    }

    public static double SLOW_TRANSFER_POWER = 0.2;
    public static double SLOW_TRANSFER_DURATION = 0.5;

    private double moveScale = 1;
    private double headingScale = 1;
    private boolean shotReady = false;

    /* Modules */
    private final FieldCentricDriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;
    private final AprilTagDetector aprilTagDetector;

    private AllianceColor allianceColor;
    private RobotState currentState;

    private final ElapsedTime timeSinceShotReady;
    private final ElapsedTime shotPrepTime = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor color) {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);

        setAllianceColor(color);

        currentState = RobotState.NONE;

        timeSinceShotReady = new ElapsedTime();
    }

    public void setAllianceColor(AllianceColor color) {
        allianceColor = color;
    }

    public void swapAllianceColor() {
        switch (allianceColor) {
            case RED:
                setAllianceColor(AllianceColor.BLUE);
                break;

            case BLUE:
                setAllianceColor(AllianceColor.RED);
                break;
        }
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    private void prepareToShoot() {
        AprilTagPoseFtc target = aprilTagDetector.getTagPose(allianceColor.targetAprilTagID);
        driveTrain.aimAtAprilTag(target);
        shooter.setRPMForAprilTag(target);
    }

    public void setState(RobotState newState) {
        if (newState == currentState) {
            return;
        }

        switch (newState) {
            case INTAKE:
                shooter.disengageKicker();
                intake.startIntake();
                transfer.startTransfer();
                break;

            case REVERSE_INTAKE:
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                break;

            case PRE_SHOOT:
                intake.startIntake();
                transfer.reverseTransfer();
                shooter.disengageKicker();
                break;

            case SHOOT:
                intake.stopIntake();
                transfer.stopTransfer();
                shooter.disengageKicker();
                shotPrepTime.reset();
                break;

            case NONE:
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                transfer.stopTransfer();
                break;
        }
        currentState = newState;
    }

    public RobotState getState() {
        return currentState;
    }

    public void loop(Gamepad gamepad1) {
        loop(-gamepad1.left_stick_y * moveScale, gamepad1.left_stick_x * moveScale, gamepad1.right_stick_x * headingScale);
    }

    public void loop(double drivePower, double strafePower, double turnPower) {
        setDrivePowers(drivePower, strafePower, turnPower);
        loopWithoutMovement();
    }

    public void loopWithoutMovement() {
        switch (currentState) {
            case PRE_SHOOT:
                shooter.setRPMForAprilTag(aprilTagDetector.getTagPose(allianceColor.targetAprilTagID));
                break;

            case SHOOT:
                prepareToShoot();

                if (!shooter.isReady()) {
                    // if shooter isn't ready, don't put any balls in
                    shooter.disengageKicker();
                    transfer.stopTransfer();
                    intake.stopIntake();

                    if (shotReady) {
                        Log.d(TAG, "Shot completed in " + timeSinceShotReady.milliseconds() + " millis");
                    }
                    shotReady = false;
                }
                else if (!shooter.isKickerEngaged()) {
                    // if shooter is ready, but kicker isn't engaged, don't risk
                    // prematurely shooting balls
                    shooter.engageKicker();
                    transfer.stopTransfer();
                    intake.stopIntake();
                }
                else {
                    // otherwise, we are truly ready to feed balls
                    if (!shotReady) {
                        Log.d(TAG, "Ready to shoot after " + shotPrepTime.milliseconds() + " millis");
                        shotPrepTime.reset();
                        shotReady = true;
                        timeSinceShotReady.reset();
                    }
                    intake.startIntake();

                    if (timeSinceShotReady.seconds() <= SLOW_TRANSFER_DURATION) {
                        transfer.setTransferPower(SLOW_TRANSFER_POWER);
                    }
                    else {
                        transfer.startTransfer();
                    }
                }
                break;

            case INTAKE:
                transfer.startTransfer();
                break;

            case REVERSE_INTAKE:
            case NONE:
                break;
        }
    }

    public boolean isShotReady() {
        return currentState == RobotState.SHOOT && shooter.isReady();
    }

    /* Module-specific methods */

    public void setDrivePowers(double drive, double strafe, double turn) {
        driveTrain.setPower(drive, strafe, turn);
    }

    public void resetOdometry() {
        driveTrain.resetOdometry();
    }

    public void increaseDefaultShooterRPM() {
        shooter.increaseDefaultRPM();
    }

    public void decreaseDefaultShooterRPM() {
        shooter.decreaseDefaultRPM();
    }

    public double getDrivePower() {
        return driveTrain.getDrivePower();
    }

    public double getStrafePower() {
        return driveTrain.getStrafePower();
    }

    public double getTurnPower() {
        return driveTrain.getTurnPower();
    }

    public void setMoveScale(double moveScale) {
        this.moveScale = moveScale;
    }

    public void setHeadingScale(double newHeadingScale) {
        headingScale = newHeadingScale;
    }
}
