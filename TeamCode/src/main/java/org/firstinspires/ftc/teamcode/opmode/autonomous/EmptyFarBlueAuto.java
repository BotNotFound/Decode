package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

@Autonomous(name = "Empty Far Blue Auto", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class EmptyFarBlueAuto extends AutonomousBase {
    public EmptyFarBlueAuto() {
        super(new Pose(89.5, 10.1, Math.toRadians(90)).mirror(), AllianceColor.BLUE);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[0];
    }
}
