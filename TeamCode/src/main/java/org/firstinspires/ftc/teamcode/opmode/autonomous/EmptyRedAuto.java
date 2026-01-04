package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "Empty Red Auto", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class EmptyRedAuto extends AutonomousBase {
    public EmptyRedAuto() {
        super(new Pose(125.5, 128, Math.toRadians(35)), AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[0];
    }
}
