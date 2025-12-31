package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.teleop.OnePersonTeleOp;

@Config
@TeleOp(group = "test")
public class ShooterTuningTeleOp extends OnePersonTeleOp {
    public static double shooterRPM = 2900;

    public ShooterTuningTeleOp() {
        super(true, true);
    }

    @Override
    public void loop() {
        robot.setFallbackShooterRPM(shooterRPM);
        super.loop();
    }
}
