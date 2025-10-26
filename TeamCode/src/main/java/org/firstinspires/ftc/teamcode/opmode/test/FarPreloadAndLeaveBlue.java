package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.concurrent.TimeUnit;
@Autonomous(name="Far Preload and Leave Blue")
public class FarPreloadAndLeaveBlue extends LinearOpMode {
    private FieldCentricDriveTrain driveTrain;

    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;

    private AprilTagDetector detector;

    int subRPM = 4100;

    @Override
    public void runOpMode(){
        ElapsedTime runtime = new ElapsedTime();
        driveTrain= new FieldCentricDriveTrain(hardwareMap, telemetry);

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);

        detector = new AprilTagDetector(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }



        runtime.reset();

        while (runtime.time(TimeUnit.SECONDS) < 10 && opModeIsActive()){

            AprilTagPoseFtc tagPose = detector.getTagPose(20);
            if(tagPose != null){
                driveTrain.setPowerFacingAprilTag(0, 0, 0, tagPose);
                shooter.setRPMForAprilTag(tagPose);
            }else {
                shooter.setRPMForAprilTag(subRPM);
            }
            shooter.engageKicker();
            intake.startIntake();
            transfer.startTransfer();
            telemetry.addData("Time: ", runtime.time(TimeUnit.SECONDS));
            telemetry.update();
        }

        shooter.setRPMForAprilTag(0);
        driveTrain.setPower(0, 0, 0);
        intake.stopIntake();
        transfer.stopTransfer();
        shooter.disengageKicker();

        sleep(1500);
        driveTrain.setPower(0.3, -1, 0.0);
        sleep(420);

        //then we just stop the robot
        driveTrain.setPower(0.0, 0.0, 0.0);

    }
}