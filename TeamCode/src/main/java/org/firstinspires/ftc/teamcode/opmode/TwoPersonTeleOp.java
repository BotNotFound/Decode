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
public class TwoPersonTeleOp extends OpMode {
    private FieldCentricDriveTrain driveTrain;

    private Shooter shooter;
    public static double targetRPM = 3000;

    private Intake intake;
    private Transfer transfer;
    private boolean intakeToggle = false;
    private boolean intakeHold = false;
    private AprilTagDetector aprilDetector;

    // true is RED, false is BLUE
    private boolean allianceColor = true;
    private int aprilTagID = 20;

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
        if(gamepad2.right_bumper){
            currentState = RobotStates.SHOOT;
        }
        else if(gamepad2.leftBumperWasReleased()){
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
        else if(gamepad2.left_trigger > 0.5){
            currentState = RobotStates.REVERSE_INTAKE;
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
        else if(currentState == RobotStates.SHOOT || gamepad1.left_trigger > 0){
            currentState = RobotStates.NONE;
            intakeToggle = false;
            intakeHold = false;
        }

        switch (currentState){
            case SHOOT:
                AprilTagPoseFtc target = aprilDetector.getTagPose(aprilTagID);
                driveTrain.setPowerFacingAprilTag(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x, target);

                shooter.engageKicker();
                shooter.setRPM(target);

                if(target != null){telemetry.addData("Range", target.range);}

                if(shooter.isReady()){
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

            case REVERSE_INTAKE:
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x);
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                break;

            default:
                driveTrain.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x * Math.sqrt(2), gamepad1.right_stick_x);
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                transfer.stopTransfer();
                break;
        }

        if(gamepad2.dpadUpWasPressed()){
            shooter.editDefaultRPM(true);
        }
        else if(gamepad2.dpadDownWasPressed()){
            shooter.editDefaultRPM(false);
        }

        telemetry.addData("target shooter rpm", targetRPM);

        if(gamepad1.startWasPressed()){
            driveTrain.resetOdometry();
        }

        telemetry.update();
    }
}
