package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.module.PeterGriffin;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    public static final double STRAFE_SCALE = Math.sqrt(2);
    public static final int BLUE_APRIL_TAG_ID = 24;

    private static final int RED_APRIL_TAG_ID = 20;

    public static double targetRPM = 3000;

    private PeterGriffin peter;
    private boolean isIntakeActive = false;

    // true is RED, false is BLUE
    private boolean isRedAlliance = true;
    private int aprilTagID = RED_APRIL_TAG_ID;

    private enum RobotStates {
        INTAKE, REVERSE_INTAKE, SHOOT, NONE
    }
    private RobotStates currentState= RobotStates.NONE;

    @Override
    public void init() {
        peter= new PeterGriffin(hardwareMap, telemetry);
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
                AprilTagPoseFtc target = peter.getTagPose(aprilTagID);
                peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x, target);

                peter.engageKicker();
                peter.setRPM(target);

                if(target != null){telemetry.addData("Range", target.range);}

                if(peter.isReady()){
                    peter.startIntake();
                    peter.startTransfer();
                }

                break;

            case INTAKE:
                peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x);
                peter.disengageKicker();
                peter.startIntake();
                if (peter.hasBall()) {
                    peter.startTransfer();
                }
                break;

            case REVERSE_INTAKE:
                peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x);
                peter.disengageKicker();
                peter.setIntakePower(-1);
                peter.reverseTransfer();
                break;

            default:
                peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * STRAFE_SCALE, gamepad1.right_stick_x);
                peter.disengageKicker();
                peter.setRPM(0);
                peter.stopIntake();
                peter.stopTransfer();
                break;
        }

        if(gamepad1.dpadUpWasPressed()) {
            peter.editDefaultRPM(true);
        }
        else if(gamepad1.dpadDownWasPressed()) {
            peter.editDefaultRPM(false);
        }

        telemetry.addData("target shooter rpm", PeterGriffin.targetRPM);

        if(gamepad1.startWasPressed()) {
            peter.resetOdometry();
        }

        telemetry.update();
    }
}
