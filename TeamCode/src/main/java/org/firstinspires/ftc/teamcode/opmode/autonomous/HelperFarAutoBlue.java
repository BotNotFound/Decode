package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

@Autonomous(name = "Helper Far Auto (blue)", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class HelperFarAutoBlue extends AutonomousBase {
    private static final Pose start = new Pose(89.5, 8.9, Math.toRadians(0)).mirror();
    private static final Pose shoot = new Pose(85, 16.2, Math.toRadians(-60)).mirror();
    private static final Pose preIntake = new Pose(120, 9, Math.toRadians(0)).mirror();
    private static final Pose intakePreset = new Pose(135, 9, Math.toRadians(0)).mirror();
    private static final Pose intakePartner = new Pose(135, 12, Math.toRadians(90)).mirror();

    public HelperFarAutoBlue() {
        super(start, AllianceColor.BLUE);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[]{
            AutonomousStage.line(start, shoot, Robot.RobotState.PRE_SHOOT),
            AutonomousStage.shootFromPoint(shoot),
            AutonomousStage.line(shoot, preIntake, Robot.RobotState.NONE),
            AutonomousStage.line(preIntake, intakePreset, Robot.RobotState.INTAKE),
            AutonomousStage.line(intakePreset, shoot, Robot.RobotState.PRE_SHOOT),
            AutonomousStage.shootFromPoint(shoot),
            AutonomousStage.line(shoot, preIntake, Robot.RobotState.NONE),
            AutonomousStage.line(preIntake, intakePartner, Robot.RobotState.INTAKE),
            AutonomousStage.line(intakePartner, shoot, Robot.RobotState.PRE_SHOOT),
            AutonomousStage.shootFromPoint(shoot),
            AutonomousStage.line(shoot, preIntake, Robot.RobotState.NONE)
        };
    }
}
