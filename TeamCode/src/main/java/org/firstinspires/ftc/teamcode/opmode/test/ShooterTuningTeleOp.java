package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.module.PeterGriffin;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ShooterTuningTeleOp extends OpMode {
    private PeterGriffin peter;

    private boolean intakeToggle = false;
    private boolean intakeHold = false;
    // true is RED, false is BLUE
    private boolean allianceColor = true;
    private int aprilTagID = 20;

    private enum RobotStates {
        INTAKE, SHOOT, NONE
    }
    private RobotStates currentState= RobotStates.NONE;

    @Override
    public void init() {
        peter= new PeterGriffin(hardwareMap, telemetry);
    }

    @Override
    public void init_loop(){
        if(gamepad1.aWasPressed()){
            allianceColor = !allianceColor;
        }
        if(allianceColor){
            telemetry.addData("Alliance", "Red");
        }
        else{
            telemetry.addData("Alliance", "Blue");
        }
        telemetry.update();
    }

    @Override
    public void start(){
        if(allianceColor){
            aprilTagID = 24;
        }
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            currentState = RobotStates.SHOOT;
        }
        else if(gamepad1.leftBumperWasReleased()){
            if(!intakeToggle){
                currentState = RobotStates.INTAKE;
                intakeToggle = true;
                telemetry.addData("LBumper", "Enter. !Intake");
            }
            else{
                currentState = RobotStates.NONE;
                intakeToggle = false;
                intakeHold = false;
                telemetry.addData("LBumper", "Enter. Intake");
            }
        }
//        else if(gamepad1.left_bumper){
//            currentState = RobotStates.INTAKE;
//            intakeToggle = true;
//            intakeHold = true;
//        }
//        else if(intakeHold){
//            currentState = RobotStates.NONE;
//            intakeToggle = false;
//            intakeHold = false;
//        }
        else if(currentState == RobotStates.SHOOT){
            currentState = RobotStates.NONE;
            intakeToggle = false;
            intakeHold = false;
        }

        switch (currentState){
            case SHOOT:
                AprilTagPoseFtc target = peter.getTagPose(aprilTagID);
                peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x, target);

                peter.engageKicker();
                peter.setRPM(PeterGriffin.targetRPM);

                if(target != null){telemetry.addData("Range", target.range);}

                peter.startIntake();
                peter.startTransfer();

                break;

            case INTAKE:
                peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x);
                peter.disengageKicker();
                peter.startIntake();
                peter.startTransfer();
                break;

            default:
                peter.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x);
                peter.disengageKicker();
                peter.setRPM(0);
                peter.stopIntake();
                peter.stopTransfer();
                break;
        }

        if(gamepad1.dpadUpWasPressed()){
            peter.editDefaultRPM(true);
        }
        else if(gamepad1.dpadDownWasPressed()){
            peter.editDefaultRPM(false);
        }

        telemetry.addData("target shooter rpm", PeterGriffin.targetRPM);

        if(gamepad1.startWasPressed()){
            peter.resetOdometry();
        }

        telemetry.update();
    }
}
