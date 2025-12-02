package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.concurrent.TimeUnit;
@Autonomous(name="Far Preload and Leave Red")
public class FarPreloadAndLeaveRed extends LinearOpMode {

    @Override
    public void runOpMode(){
        ElapsedTime runtime = new ElapsedTime();
        final FieldCentricDriveTrain driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);

        final Shooter shooter = new Shooter(hardwareMap, telemetry);
        final Intake intake = new Intake(hardwareMap);
        final Transfer transfer = new Transfer(hardwareMap, telemetry);

        final AprilTagDetector detector = new AprilTagDetector(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        int subRPM = 4100;


        runtime.reset();

        while (runtime.time(TimeUnit.SECONDS) < 10 && opModeIsActive()){
            detector.update(24);
            AprilTagPoseFtc tagPose = detector.getTagPose();
            Pose3D robotPose = detector.getRobotPose();

            if(tagPose != null){
                driveTrain.setPowerFacingAprilTag(0, 0, 0, tagPose, robotPose);
                shooter.setRPMForAprilTag(tagPose);
            }else{
                shooter.setRPM(subRPM);
            }
            shooter.setRPM(subRPM);
            shooter.engageKicker();
            intake.startIntake();
            transfer.startTransfer();
            telemetry.addData("Time: ", runtime.time(TimeUnit.SECONDS));
            telemetry.update();
        }

        shooter.setRPM(0);
        driveTrain.setPower(0, 0, 0);
        intake.stopIntake();
        transfer.stopTransfer();
        shooter.disengageKicker();

        sleep(1500);
        driveTrain.setPower(0.3, 1, 0.0);
        sleep(420);

        //then we just stop the robot
        driveTrain.setPower(0.0, 0.0, 0.0);

    }
}