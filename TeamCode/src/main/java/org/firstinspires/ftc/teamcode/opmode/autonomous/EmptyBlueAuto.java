package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

@Autonomous(name = "Empty Blue Auto", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class EmptyBlueAuto extends AutonomousBase {
    public EmptyBlueAuto() {
        super(new Pose(117.9, 131.3, Math.toRadians(-54)).mirror(), AllianceColor.BLUE);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[0];
    }
}
