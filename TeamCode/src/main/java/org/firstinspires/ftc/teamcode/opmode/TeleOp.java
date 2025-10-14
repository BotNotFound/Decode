package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.module.AprilTagRange;
import org.firstinspires.ftc.teamcode.module.DriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private DriveTrain driveTrain;
    private Shooter shooter;

    private Intake intake;

    private Transfer transfer;

    private AprilTagRange aprilTagRange;

    private double shooterPower = 0.75;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        aprilTagRange = new AprilTagRange(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        driveTrain.setPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if(gamepad2.a){
            shooter.engageKicker();
        }
        else if(gamepad2.b){
            shooter.disengageKicker();
        }

        if(gamepad2.dpadRightWasPressed()){
            shooterPower += 0.05;
        }
        else if(gamepad2.dpadRightWasReleased()){
            shooterPower -= 0.05;
        }

        //right trigger intake, left trigger eject
        if(gamepad2.right_trigger > 0.1) {
            intake.setPower(gamepad2.right_trigger);
            transfer.startTransfer();
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-gamepad2.left_trigger);
            transfer.reverseTransfer();
        }

        shooter.setPower(shooterPower);
        telemetry.update();
    }
}
