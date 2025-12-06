package org.firstinspires.ftc.teamcode;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.ArtifactLocation;
import org.firstinspires.ftc.teamcode.module.ArtifactTracker;
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

    private double moveScale = 1;
    private double headingScale = 1;
    private boolean shotReady = false;
    private int shotsTaken = 0;
    private boolean keepBallsApart;

    /* Modules */
    private final FieldCentricDriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;
    private final AprilTagDetector aprilTagDetector;
    private final ArtifactTracker ballTracker;

    private AllianceColor allianceColor;
    private RobotState currentState;

    private final ElapsedTime stateStopwatch;
    private final ElapsedTime timeSinceShotReady;
    private final ElapsedTime shotPrepTime;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor color) {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);
        ballTracker = new ArtifactTracker(hardwareMap, telemetry);

        setAllianceColor(color);

        currentState = RobotState.NONE;

        stateStopwatch = new ElapsedTime();
        timeSinceShotReady = new ElapsedTime();
        shotPrepTime = new ElapsedTime();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Log.i(TAG, "Robot initialized");
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

    public int getShotsTaken() {
        if (currentState != RobotState.SHOOT) {
            return 0;
        }
        return shotsTaken;
    }

    private void prepareToShoot() {
        AprilTagPoseFtc target = aprilTagDetector.getTagPose();
        Pose3D robot = aprilTagDetector.getRobotPose();
        driveTrain.aimAtAprilTag(target, robot);
        shooter.setRPMForAprilTag(target);
    }

    public void setState(RobotState newState) {
        if (newState == currentState) {
            return;
        }

        Log.v(TAG, "State " + currentState + " lasted for " + stateStopwatch.seconds() + " seconds");
        Log.i(TAG, "Switched to new state: " + newState);
        stateStopwatch.reset();

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

                shotReady = false;
                shotPrepTime.reset();
                shotsTaken = 0;
                keepBallsApart = ballTracker.hasBall(ArtifactLocation.BACK) &&
                        ballTracker.hasBall(ArtifactLocation.MIDDLE);

                Log.d(TAG, "enter shoot {" +
                        (ballTracker.hasBall(ArtifactLocation.FRONT) ? "front | " : "      | ") +
                        (ballTracker.hasBall(ArtifactLocation.MIDDLE) ? "middle | " : "       | ") +
                        (ballTracker.hasBall(ArtifactLocation.BACK) ? "back | " : "     | ") +
                        "keepBallsApart = " + keepBallsApart + "}");
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
        ballTracker.reportDetections();

        switch (currentState) {
            case SHOOT:
                aprilTagDetector.update(allianceColor.targetAprilTagID);

                prepareToShoot();

                if (!shooter.isReady()) {
                    intake.stopIntake();
                    if (keepBallsApart && shotsTaken < 1) {
                        transfer.reverseTransfer();
                    }
                    else {
                        transfer.stopTransfer();
                    }

                    if (shotReady) {
                        shotsTaken++;
                        Log.d(TAG, "Shot #" + shotsTaken + " completed in " + timeSinceShotReady.milliseconds() + " millis");
                    }
                    shotReady = false;

                    break;
                }

                if (!shotReady) {
                    Log.d(TAG, "Ready to shoot after " + shotPrepTime.milliseconds() + " millis");
                    shotPrepTime.reset();
                    shotReady = true;
                    timeSinceShotReady.reset();
                }

                if (ballTracker.hasBall(ArtifactLocation.BACK)) {
                    intake.stopIntake();
                    transfer.stopTransfer();
                    shooter.engageKicker();
                }
                else {
                    intake.startIntake();
                    transfer.startTransfer();
                    shooter.engageKicker();
                }
                break;

            case PRE_SHOOT:
                aprilTagDetector.update(allianceColor.targetAprilTagID);
                shooter.setRPMForAprilTag(aprilTagDetector.getTagPose());

            case INTAKE:
                if (ballTracker.hasAllArtifacts()) {
                    transfer.stopTransfer();
                    intake.stopIntake();
                }
                else if (
                        ballTracker.hasBall(ArtifactLocation.BACK) &&
                                ballTracker.hasBall(ArtifactLocation.MIDDLE)
                ) {
                    transfer.stopTransfer();
                    intake.startIntake();
                }
                else {
                    transfer.startTransfer();
                    intake.startIntake();
                }
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

    public double getDefaultShooterRPM() {
        return shooter.defaultRPM;
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

    public int getHeldArtifactCount() {
        return ballTracker.getArtifactCount();
    }
}