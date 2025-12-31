package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@TeleOp(group = "test")
public class ShooterTuningTeleOp extends OpMode {
    private FieldCentricDriveTrain driveTrain;

    private Shooter shooter;
    public static double targetRPM = 3000;

    private Intake intake;
    private Transfer transfer;
    private boolean intakeToggle = false;
    private AprilTagDetector aprilDetector;

    // true is RED, false is BLUE
    private boolean allianceColor = true;
    private int aprilTagID = 20;

    private enum RobotStates {
        INTAKE, SHOOT, NONE
    }

    private RobotStates currentState = RobotStates.NONE;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        aprilDetector = new AprilTagDetector(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            allianceColor = !allianceColor;
        }
        if (allianceColor) {
            telemetry.addData("Alliance", "Red");
        }
        else {
            telemetry.addData("Alliance", "Blue");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (allianceColor) {
            aprilTagID = 24;
        }
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            currentState = RobotStates.SHOOT;
        }
        else if (gamepad1.leftBumperWasReleased()) {
            if (!intakeToggle) {
                currentState = RobotStates.INTAKE;
                intakeToggle = true;
                telemetry.addData("LBumper", "Enter. !Intake");
            }
            else {
                currentState = RobotStates.NONE;
                intakeToggle = false;
                telemetry.addData("LBumper", "Enter. Intake");
            }
        }
        else if (currentState == RobotStates.SHOOT) {
            currentState = RobotStates.NONE;
            intakeToggle = false;
        }

        aprilDetector.update(aprilTagID);
        AprilTagPoseFtc target = aprilDetector.getTagPose();
        Pose3D robot = aprilDetector.getRobotPose();

        if (target != null) {
            telemetry.addData("Distance to April Tag", target.range);
        }

        switch (currentState) {
            case SHOOT:
                // TODO replace with appropriate method
//                driveTrain.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x, target, robot);
                shooter.setRPM(targetRPM);

                if (shooter.isReady()) {
                    shooter.engageKicker();
                    intake.startIntake();
                    transfer.startTransfer();
                }

                break;

            case INTAKE:
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x);
                shooter.disengageKicker();
                intake.startIntake();
                transfer.startTransfer();
                break;

            default:
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x);
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                transfer.stopTransfer();
                break;
        }

        if (gamepad1.dpadUpWasPressed()) {
            shooter.increaseDefaultRPM();
            targetRPM = shooter.defaultRPM;
        }
        else if (gamepad1.dpadDownWasPressed()) {
            shooter.decreaseDefaultRPM();
            targetRPM = shooter.defaultRPM;
        }

        telemetry.addData("target shooter rpm", targetRPM);

        if (gamepad1.startWasPressed()) {
            driveTrain.resetOdometry();
        }

        telemetry.update();
    }
}
