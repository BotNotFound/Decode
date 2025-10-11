package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.module.DriveTrain;
import org.firstinspires.ftc.teamcode.module.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private DriveTrain driveTrain;
    private Shooter shooter;
    private double shooterPower = 0.75;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        driveTrain.setPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if(gamepad1.a){
            shooter.engageKicker();
        }
        else if(gamepad1.b){
            shooter.disengageKicker();
        }

        if(gamepad1.dpadRightWasPressed()){
            shooterPower += 0.05;
        }
        else if(gamepad1.dpadRightWasReleased()){
            shooterPower -= 0.05;
        }

        shooter.setPower(shooterPower);

        telemetry.update();
    }
}
