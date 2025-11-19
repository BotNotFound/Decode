package org.firstinspires.ftc.teamcode;

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
    public enum AllianceColor {
        RED(24, Math.PI / 4),
        BLUE(20, -Math.PI / 4);

        public final int targetAprilTagID;
        public final double defaultShootingRotation;

        AllianceColor(int targetAprilTagID, double defaultShootingRotation) {
            this.targetAprilTagID = targetAprilTagID;
            this.defaultShootingRotation = defaultShootingRotation;
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

    public static double MOVE_SCALE = 1;
    public static int MAX_DETECTION_LULL_TIME = 1;


    /* Modules */
    private final FieldCentricDriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;
    private final AprilTagDetector aprilTagDetector;

    private AllianceColor allianceColor;
    private RobotState currentState;

    private final ElapsedTime timeSinceLastTagDetection;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor color) {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);

        timeSinceLastTagDetection = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        setAllianceColor(color);

        currentState = RobotState.NONE;
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
        if (target != null) {
            // We've detected a tag; do auto-aim as normal
            timeSinceLastTagDetection.reset();
            driveTrain.aimAtAprilTag(target);
            shooter.setRPMForAprilTag(target);
            return;
        }

        if (timeSinceLastTagDetection.time() <= MAX_DETECTION_LULL_TIME) {
            // We don't have a detection, but there was one recently enough that we're probably
            // just blurring the image with our movement.  Let the robot settle so the image
            // can clear before we use the fallback rotation
            shooter.setRPMForAprilTag(null);
            return;
        }

        // At this point, we can safely assume that there really are no april tags in view.
        // Rotate to fallback heading and prepare to shoot
        driveTrain.tryAimAtAprilTag(null, allianceColor.defaultShootingRotation);
        shooter.setRPMForAprilTag(null);
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
            case SHOOT:
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
        loop(-gamepad1.left_stick_y * MOVE_SCALE, gamepad1.left_stick_x * MOVE_SCALE, gamepad1.right_stick_x * MOVE_SCALE);
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

                if(shooter.isReady()) {
                    shooter.engageKicker();
                    intake.startIntake();
                    transfer.startTransfer();
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
}
