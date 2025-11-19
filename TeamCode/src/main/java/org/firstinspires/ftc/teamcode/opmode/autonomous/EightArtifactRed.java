package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "8 Artifact Red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class EightArtifactRed extends AutonomousBase {
    public EightArtifactRed() {
        super(new Pose(106.8, 135.414, Math.toRadians(0)), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToFirstRowArtifacts, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootFirstRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToSecondRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeSecondRowArtifacts, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootSecondRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToThirdRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootThirdRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(
                        follower.pathBuilder().addPath(
                                new BezierLine(
                                        autoPath.shootThirdRow.endPoint(),
                                        new Pose(116.559, 64.120)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(0)).build(),
                        Robot.RobotState.NONE
                ),
        };
    }

    public static class Paths {

        public final PathChain shootPreloads;
        public final PathChain goToFirstRowArtifacts;
        public final PathChain intakeFirstRow;
        public final PathChain shootFirstRow;
        public final PathChain goToSecondRow;
        public final PathChain intakeSecondRowArtifacts;
        public final PathChain shootSecondRow;
        public final PathChain goToThirdRow;
        public final PathChain intakeThirdRow;
        public final PathChain shootThirdRow;

        public Paths(Follower follower) {
            shootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.622, 135.414), new Pose(95.344, 99.816))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            goToFirstRowArtifacts = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.344, 99.816), new Pose(90, 84))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90, 84), new Pose(135, 84))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135, 84), new Pose(99.101, 100.353))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            goToSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(99.101, 100.353), new Pose(90, 60))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            intakeSecondRowArtifacts = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90, 60), new Pose(135, 60))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(135, 60),
                                    new Pose(92.303, 68.154),
                                    new Pose(91.051, 92.482)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            goToThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91.051, 92.482), new Pose(90, 37))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            intakeThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90, 37), new Pose(140, 37))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(140, 37), new Pose(91.051, 92.482))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();
        }
    }
}
