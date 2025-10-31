package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
public class TeleOpRobot {
    public enum AllianceColor {
        RED(20),
        BLUE(24);

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
        SHOOT,
        NONE,
    }

    public static final double STRAFE_SCALE = Math.sqrt(2);
    public static double MOVE_SCALE = 0.7;

    /* Modules */
    private final FieldCentricDriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;
    private final AprilTagDetector aprilTagDetector;

    private AllianceColor allianceColor;
    private RobotState currentState;

    public TeleOpRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor color) {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);

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

    public void setState(RobotState newState) {
        switch (newState) {
            case INTAKE:
                shooter.disengageKicker();
                intake.startIntake();
                break;

            case REVERSE_INTAKE:
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
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
        loop(-gamepad1.left_stick_y * MOVE_SCALE, gamepad1.left_stick_x * STRAFE_SCALE * MOVE_SCALE, gamepad1.right_stick_x * MOVE_SCALE);
    }

    public void loop(double drivePower, double strafePower, double turnPower) {
        switch (currentState) {
            case SHOOT:
                AprilTagPoseFtc target = aprilTagDetector.getTagPose(allianceColor.targetAprilTagID);
                driveTrain.setPowerFacingAprilTag(drivePower, strafePower, turnPower, target);
                driveTrain.setPowerFacingAprilTag(drivePower, strafePower, turnPower, target);

                shooter.engageKicker();
                shooter.setRPMForAprilTag(target);

                if(shooter.isReady()) {
                    intake.startIntake();
                    transfer.startTransfer();
                }
                break;

            case INTAKE:
                driveTrain.setPower(drivePower, strafePower, turnPower);
                if (intake.hasBall()) {
                    transfer.startTransfer();
                }
                break;

            case REVERSE_INTAKE:
            case NONE:
                driveTrain.setPower(drivePower, strafePower, turnPower);
                break;
        }
    }

    /* Module-specific methods */

    public void resetOdometry() {
        driveTrain.resetOdometry();
    }

    public void increaseDefaultShooterRPM() {
        shooter.increaseDefaultRPM();
    }

    public void decreaseDefaultShooterRPM() {
        shooter.decreaseDefaultRPM();
    }
}
