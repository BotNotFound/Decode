package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class HelperFarAutoRed extends AutonomousBase {
    private static final Pose start = new Pose(89.5, 8.9, Math.toRadians(0));
    private static final Pose shoot = new Pose(88.8, 16.2, Math.toRadians(-60));
    private static final Pose preIntake = new Pose(120, 10, Math.toRadians(0));
    private static final Pose intakePreset = new Pose(135, 10, Math.toRadians(0));
    private static final Pose intakePartner = new Pose(135, 12, Math.toRadians(90));

    public HelperFarAutoRed() {
        super(start, AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[]{
            AutonomousStage.line(start, shoot, Robot.RobotState.PRE_SHOOT),
            AutonomousStage.shootFromPoint(shoot),
            AutonomousStage.line(start, preIntake, Robot.RobotState.NONE),
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
