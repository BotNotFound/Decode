package org.firstinspires.ftc.teamcode.opmode.test;

//imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.module.DriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;

//I'm not sure what to put for the group here
@Autonomous(name="Leaving the Launch Zone Only")
public class LeaveAuto extends LinearOpMode{
    private DriveTrain driveTrain;

    @Override
    public void runOpMode(){
        ElapsedTime runtime = new ElapsedTime();
        driveTrain= new DriveTrain(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }


        sleep(1500);
        driveTrain.setPower(0.67, 0.0, 0.0);
        sleep(2000);

        //then we just stop the robot
        driveTrain.setPower(0.0, 0.0, 0.0);

    }

}