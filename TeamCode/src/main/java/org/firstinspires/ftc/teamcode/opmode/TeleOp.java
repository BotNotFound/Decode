package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    public static final double STRAFE_SCALE = Math.sqrt(2);
    public static final int BLUE_APRIL_TAG_ID = 24;
    private FieldCentricDriveTrain driveTrain;

    private static final int RED_APRIL_TAG_ID = 20;

    private Shooter shooter;
    public static double targetRPM = 3000;

    private Intake intake;
    private Transfer transfer;
    private boolean isIntakeActive = false;
    private AprilTagDetector aprilDetector;

    // true is RED, false is BLUE
    private boolean isRedAlliance = true;
    private int aprilTagID = RED_APRIL_TAG_ID;

    private enum RobotStates {
        INTAKE, REVERSE_INTAKE, SHOOT, NONE
    }
    private RobotStates currentState= RobotStates.NONE;

    @Override
    public void init() {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        aprilDetector = new AprilTagDetector(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        if(gamepad1.aWasPressed()) {
            isRedAlliance = false;
        }
        if(isRedAlliance) {
            telemetry.addData("Alliance", "Red");
        }
        else {
            telemetry.addData("Alliance", "Blue");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if(isRedAlliance) {
            aprilTagID = BLUE_APRIL_TAG_ID;
        }
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            currentState = RobotStates.SHOOT;
        }
        else if (gamepad1.leftBumperWasReleased()){
            if (!isIntakeActive) {
                currentState = RobotStates.INTAKE;
                isIntakeActive = true;
                telemetry.addData("LBumper", "Enter. !Intake");
            }
            else {
                currentState = RobotStates.NONE;
                isIntakeActive = false;
                telemetry.addData("LBumper", "Enter. Intake");
            }
        }
        else if (gamepad1.left_trigger > 0.5) {
            currentState = RobotStates.REVERSE_INTAKE;
        }
        else if (currentState == RobotStates.SHOOT || gamepad1.left_trigger > 0) {
            currentState = RobotStates.NONE;
            isIntakeActive = false;
        }

        switch (currentState) {
            case SHOOT:
                AprilTagPoseFtc target = aprilDetector.getTagPose(aprilTagID);
                driveTrain.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x, target);

                shooter.engageKicker();
                shooter.setRPM(target);

                if(target != null){telemetry.addData("Range", target.range);}

                if(shooter.isReady()){
                    intake.startIntake();
                    transfer.startTransfer();
                }

                break;

            case INTAKE:
                driveTrain.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x);
                shooter.disengageKicker();
                intake.startIntake();
                if (intake.hasBall()) {
                    transfer.startTransfer();
                }
                break;

            case REVERSE_INTAKE:
                driveTrain.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x);
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                break;

            default:
                driveTrain.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x);
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                transfer.stopTransfer();
                break;
        }

        if(gamepad1.dpadUpWasPressed()) {
            shooter.editDefaultRPM(true);
        }
        else if(gamepad1.dpadDownWasPressed()) {
            shooter.editDefaultRPM(false);
        }

        telemetry.addData("target shooter rpm", targetRPM);

        if(gamepad1.startWasPressed()) {
            driveTrain.resetOdometry();
        }

        telemetry.update();
    }
}
