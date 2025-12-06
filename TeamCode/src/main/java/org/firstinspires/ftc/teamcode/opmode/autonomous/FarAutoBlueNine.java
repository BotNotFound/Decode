package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

// arithmetic is applied uniformly to ensure everything is mirrored
@SuppressWarnings("PointlessArithmeticExpression")
@Autonomous(name = "far auto blue nine artifacts", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class FarAutoBlueNine extends AutonomousBase {

    public FarAutoBlueNine() {
        super(new Pose(92.048, 0.355, Math.toRadians(90)).mirror(), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.NONE),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                        .setLinearHeadingInterpolation(Math.toRadians(180 - 90), Math.toRadians(180 - 67))
                        .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToArtifactRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeArtifacts, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootArtifactRowOne, Robot.RobotState.NONE),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPath.shootArtifactRowOne.endPose()))
                        .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 67))
                        .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToSecondArtifactRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeSecondArtifactRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootSecondArtifactRow, Robot.RobotState.NONE),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPath.shootSecondArtifactRow.endPose()))
                        .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180 - 67))
                        .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.INTAKE),
        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain goToArtifactRow;
        public PathChain intakeArtifacts;
        public PathChain shootArtifactRowOne;
        public PathChain goToSecondArtifactRow;
        public PathChain intakeSecondArtifactRow;
        public PathChain shootSecondArtifactRow;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(92.048, 0.355).mirror(), new Pose(87.908, 4.177).mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 90), Math.toRadians(180 - 67))
                    .build();

            goToArtifactRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.908, 4.177).mirror(), new Pose(105.900, 29.498).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 67), Math.toRadians(180 - 0))
                    .build();

            intakeArtifacts = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(105.900, 29.498).mirror(), new Pose(134.100, 29.498).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
                    .build();

            shootArtifactRowOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(134.100, 29.498).mirror(), new Pose(87.908, 4.177).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 67))
                    .build();

            goToSecondArtifactRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.908, 4.177).mirror(), new Pose(144.000, 21.900).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 67), Math.toRadians(-90))
                    .build();

            intakeSecondArtifactRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144.000, 21.900).mirror(), new Pose(144.000, 2.500).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            shootSecondArtifactRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144.000, 2.500).mirror(), new Pose(87.908, 4.177).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180 - 67))
                    .build();

            leaveLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.908, 4.177).mirror(), new Pose(110.840, 4.018).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 67), Math.toRadians(180 - 0))
                    .build();
        }
    }
}
