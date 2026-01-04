package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "Empty Far Red Auto", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class EmptyFarRedAuto extends AutonomousBase {
    public EmptyFarRedAuto() {
        super(new Pose(89.5, 10.1, Math.toRadians(90)), AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[0];
    }
}