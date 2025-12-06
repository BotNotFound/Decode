package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

// arithmetic is applied uniformly to ensure everything is mirrored
@SuppressWarnings("PointlessArithmeticExpression")
@Autonomous(name = "12 Artifact Blue", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class StandardTwelveArtifactBlue extends AutonomousBase {
    public StandardTwelveArtifactBlue() {
        super(new Pose(125.5, 128, Math.toRadians(35)).mirror(), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(180 - 35))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.hitGate, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootFirstRow.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(180 - 35))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.intakeSecondRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootSecondRow.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(180 - 35))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootThirdRow.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(180 - 35))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.NONE),

        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain intakeFirstRow;
        public PathChain hitGate;
        public PathChain shootFirstRow;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain intakeThirdRow;
        public PathChain shootThirdRow;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125.5, 128).mirror(),
                                    new Pose(94, 86.5).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 35))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94, 86.5).mirror(),
                                    new Pose(94, 80).mirror(),
                                    new Pose(96, 80).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                    .addPath(

                            new BezierLine(new Pose(96, 80).mirror(), new Pose(132.5, 80).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                    .build();
            hitGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.5, 80).mirror(),
                                    new Pose(115, 77).mirror(),
                                    new Pose(132, 67.67).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(132, 67.67).mirror(),
                                    new Pose(94, 86.5).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 35))
                    .build();

            intakeSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94, 86.5).mirror(),
                                    new Pose(94, 56).mirror(),
                                    new Pose(96, 56).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
                    .addPath(
                            new BezierLine(new Pose(96, 56).mirror(), new Pose(135, 56).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                    .build();
            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(135, 58).mirror(),
                                    new Pose(111, 53).mirror(),
                                    new Pose(94, 86.5).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 35))
                    .build();

            intakeThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94, 86.5).mirror(),
                                    new Pose(94, 35).mirror(),
                                    new Pose(96, 35).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 35), Math.toRadians(180 - 0))
                    .addPath(
                            new BezierLine(new Pose(96, 35).mirror(), new Pose(135, 35).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(135, 35).mirror(),
                                    new Pose(94, 86.5).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 35))
                    .build();

            leaveLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94, 86.5).mirror(),
                                    new Pose(114.091, 71.748).mirror(),
                                    new Pose(114.091, 71.748).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                    .build();
        }
    }
}

