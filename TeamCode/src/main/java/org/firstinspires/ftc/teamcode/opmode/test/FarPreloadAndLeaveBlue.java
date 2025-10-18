package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.DriveTrain;
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
            driveTrain.setPower(0, 0, 0, tagPose);
            shooter.setRPM(tagPose);
            shooter.engageKicker();
            intake.startIntake();
            transfer.startTransfer();
        }

        shooter.setRPM(0);
        driveTrain.setPower(0, 0, 0);
        intake.stopIntake();
        transfer.stopTransfer();
        shooter.disengageKicker();

        sleep(1500);
        driveTrain.setPower(0.3, 1, 0.0);
        sleep(500);

        //then we just stop the robot
        driveTrain.setPower(0.0, 0.0, 0.0);

    }
}