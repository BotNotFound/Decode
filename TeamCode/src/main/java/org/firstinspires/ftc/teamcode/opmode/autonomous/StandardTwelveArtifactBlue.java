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

@Autonomous(name = "12 Artifact Blue", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class StandardTwelveArtifactRed extends AutonomousBase {
    public StandardTwelveArtifactRed() {
        super(new Pose(106.8, 135.414, Math.toRadians(0)).mirror(), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToFirstRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.hitGate, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootFirstRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToSecondRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeSecondRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootSecondRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-47))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToThirdRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootThirdRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-48.5))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.NONE),

        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain goToFirstRow;
        public PathChain intakeFirstRow;
        public PathChain hitGate;
        public PathChain shootFirstRow;
        public PathChain goToSecondRow;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain goToThirdRow;
        public PathChain intakeThirdRow;
        public PathChain shootThirdRow;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(106.800, 135.414).mirror(),
                                    new Pose(95.344, 99.816).mirror(),
                                    new Pose(95.344, 99.816).mirror(),
                                    new Pose(95.344, 99.816).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-45))
                    .build();

            goToFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.344, 99.816).mirror(),
                                    new Pose(92.000, 84.000).mirror(),
                                    new Pose(92.000, 84.000).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-45), Math.toRadians(180-0))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(92.000, 84.000).mirror(), new Pose(140, 84.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                    .build();

            hitGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(140, 84.000).mirror(),
                                    new Pose(109.722, 82.838).mirror(),
                                    new Pose(135.900, 73.244).mirror(),
                                    new Pose(135.900, 73.244).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-90))
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(135.900, 73.244).mirror(),
                                    new Pose(99.101, 100.353).mirror(),
                                    new Pose(99.101, 100.353).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-45))
                    .build();

            goToSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(99.101, 100.353).mirror(),
                                    new Pose(101.800, 62).mirror(),
                                    new Pose(101.800, 62).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-45), Math.toRadians(180-0))
                    .build();

            intakeSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.800, 62).mirror(), new Pose(140.975, 62).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                    .build();

            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(140.975, 62).mirror(),
                                    new Pose(78.469, 63.347).mirror(),
                                    new Pose(85.862, 84.854).mirror(),
                                    new Pose(85.862, 83.846).mirror(),
                                    new Pose(86.030, 85.862).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-47))
                    .build();

            goToThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.030, 85.862).mirror(),
                                    new Pose(100.400, 37).mirror(),
                                    new Pose(100.400, 37).mirror(),
                                    new Pose(100.400, 37).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-47), Math.toRadians(180-0))
                    .build();

            intakeThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.400, 37).mirror(), new Pose(140.975, 37).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(140.975, 37).mirror(),
                                    new Pose(91.743, 90.903).mirror(),
                                    new Pose(91.743, 90.903).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-48.5))
                    .build();

            leaveLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.743, 90.903).mirror(),
                                    new Pose(114.091, 71.748).mirror(),
                                    new Pose(114.091, 71.748).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-48.5), Math.toRadians(180-0))
                    .build();
        }
    }
}

