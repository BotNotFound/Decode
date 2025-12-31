package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.teleop.OnePersonTeleOp;

@Config
@TeleOp(group = "test")
public class ShooterTuningTeleOp extends OnePersonTeleOp {
    public ShooterTuningTeleOp() {
        super(true, true);
    }
}
