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

@Autonomous(name = "12 Artifact Red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class StandardTwelveArtifactRed extends AutonomousBase {
    public StandardTwelveArtifactRed() {
        super(new Pose(106.8, 135.414, Math.toRadians(0)), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToFirstRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.hitGate, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootFirstRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToSecondRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeSecondRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootSecondRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.goToThirdRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootThirdRow.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48.5))
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
                                    new Pose(106.800, 135.414),
                                    new Pose(95.344, 99.816),
                                    new Pose(95.344, 99.816),
                                    new Pose(95.344, 99.816)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            goToFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.344, 99.816),
                                    new Pose(91.000, 84.000),
                                    new Pose(91.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91.000, 84.000), new Pose(140, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            hitGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(140.000, 84.000),
                                    new Pose(125.349, 75.949),
                                    new Pose(130.390, 77.797),
                                    new Pose(134.954, 70.740)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(78.67))
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134.954, 70.740),
                                    new Pose(99.101, 100.353),
                                    new Pose(99.101, 100.353)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(78.67), Math.toRadians(45))
                    .build();

            goToSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(99.101, 100.353),
                                    new Pose(91, 62),
                                    new Pose(91, 62)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            intakeSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91, 62), new Pose(150, 62))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(150, 62),
                                    new Pose(78.469, 63.347),
                                    new Pose(85.862, 84.854),
                                    new Pose(85.862, 83.846),
                                    new Pose(85.862 , 85.862)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                    .build();

            goToThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.862, 85.862),
                                    new Pose(91, 37),
                                    new Pose(91, 37),
                                    new Pose(91, 37)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(0))
                    .build();

            intakeThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91, 37), new Pose(150, 37))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(150, 37),
                                    new Pose(91.743, 90.903),
                                    new Pose(91.743, 90.903)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48.5))
                    .build();

            leaveLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.743, 90.903),
                                    new Pose(114.091, 71.748),
                                    new Pose(114.091, 71.748)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48.5), Math.toRadians(0))
                    .build();
        }
    }
}

