package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

// arithmetic is applied uniformly to ensure everything is mirrored
@SuppressWarnings("PointlessArithmeticExpression")
@Autonomous(name = "far auto blue six artifacts", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class FarAutoBlueSix extends AutonomousBase {

    public FarAutoBlueSix() {
        super(new Pose(89.5, 10.1, Math.toRadians(90)).mirror(), AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
            new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 90), Math.toRadians(64.7))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.goToArtifactRowOne, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.goToArtifactRowTwo, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeArtifacts, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootArtifacts, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPath.shootArtifacts.endPose()))
                    .setLinearHeadingInterpolation(
                        Math.toRadians(180 - 90), Math.toRadians(180 - 67))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.INTAKE),
        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain goToArtifactRowOne;
        public PathChain goToArtifactRowTwo;
        public PathChain intakeArtifacts;
        public PathChain shootArtifacts;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(89.5, 10.1).mirror(),
                        new Pose(86.7, 18.03).mirror()
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180 - 90), Math.toRadians(64.7))
                .build();

            goToArtifactRowOne = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(86.7, 18.03).mirror(),
                        new Pose(126.5, 17.5).mirror()
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(64.7), Math.toRadians(-90))
                .build();

            goToArtifactRowTwo = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(126.5, 17.5).mirror(),
                        new Pose(137.8, 23.25).mirror()
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
            //the end pose should be adjusted with Position Logger
            intakeArtifacts = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(137.8, 23.25).mirror(), new Pose(141.336, 2.164).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-110))
                .build();

            shootArtifacts = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(141.336, 2.164).mirror(), new Pose(90.562, 16.980).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(64.7))
                .build();

            leaveLaunchZone = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(90.562, 16.980).mirror(), new Pose(105.711, 10.155).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(64.7), Math.toRadians(180 - 0))
                .build();
        }
    }
}
