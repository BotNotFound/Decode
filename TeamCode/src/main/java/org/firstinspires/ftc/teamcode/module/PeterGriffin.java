package org.firstinspires.ftc.teamcode.module;

import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;


public class PeterGriffin {
    
    public static double targetRPM = 3000;
    private FieldCentricDriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private AprilTagDetector aprilDetector;
    private final Telemetry telemetry;


    //constructor
    public PeterGriffin(HardwareMap hwMap, Telemetry tele){
        this.telemetry=tele;
        driveTrain = new FieldCentricDriveTrain(hwMap, tele);
        driveTrain.resetOdometry();

        shooter = new Shooter(hwMap, tele);
        intake = new Intake(hwMap, tele);
        transfer = new Transfer(hwMap, tele);
        aprilDetector = new AprilTagDetector(hwMap, tele);
        telemetry.addData("Robot", "Initialized");
        telemetry.update();

    }

    //all the getter methods and state function for the robot
    public AprilTagPoseFtc getTagPose(int tagID){
       return aprilDetector.getTagPose(tagID);
    }
    
    public void resetOdometry(){
        driveTrain.resetOdometry();
    }
    public void setPowerFacingAprilTag(double drive, double strafe, double turn, AprilTagPoseFtc targetTag) {
        driveTrain.setPowerFacingAprilTag(drive, strafe, turn, targetTag);
    }

    public void setPowerFacingAprilTag(double drive, double strafe, double turn) {
        driveTrain.setPowerFacingAprilTag(drive, strafe, turn);
    }
    public void setIntakePower(double power) {
        intake.setPower(power);
    }
    public boolean hasBall() {
        return intake.hasBall();
    }
    public void startIntake() {
        intake.startIntake();
    }

    public void stopIntake() {
        intake.setPower(0);
    }
    public void setRPM(double rpm) {
      shooter.setRPM(rpm);
    }

    public void setRPM(AprilTagPoseFtc tagPose) {
        shooter.setRPM(tagPose);
    }

    public void editDefaultRPM(boolean increase) {
       shooter.editDefaultRPM(increase);
    }

    public void engageKicker() {
        shooter.engageKicker();
    }

    public void disengageKicker() {
        shooter.disengageKicker();
    }

    public boolean isReady() {
        return shooter.isReady();
    }
    public void startTransfer() {
        transfer.startTransfer();
    }


    public void stopTransfer() {
       transfer.stopTransfer();
    }
    
    public void reverseTransfer() {
        transfer.reverseTransfer();
    }



}
