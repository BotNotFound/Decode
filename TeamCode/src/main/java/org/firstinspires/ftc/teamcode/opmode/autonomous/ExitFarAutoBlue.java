package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

@Autonomous(name = "Exit Far (Blue)", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class ExitFarAutoBlue extends AutonomousBase {
    private static final Pose start = new Pose(89.5, 8.9, Math.toRadians(0)).mirror();
    private static final Pose end = new Pose(120, 9, Math.toRadians(0)).mirror();

    public ExitFarAutoBlue() {
        super(start, AllianceColor.BLUE);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[]{
            AutonomousStage.line(start, end, Robot.RobotState.NONE)
        };
    }
}
