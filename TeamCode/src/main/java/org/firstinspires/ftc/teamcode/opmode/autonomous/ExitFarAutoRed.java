package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "Exit Far (Red)", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class ExitFarAutoRed extends AutonomousBase {
    private static final Pose start = new Pose(89.5, 8.9, Math.toRadians(0));
    private static final Pose end = new Pose(120, 9, Math.toRadians(0));

    public ExitFarAutoRed() {
        super(start, AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[]{
            AutonomousStage.line(start, end, Robot.RobotState.NONE)
        };
    }
}
