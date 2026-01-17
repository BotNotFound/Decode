package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "far auto red nine artifacts", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class FarAutoRedNine extends AutonomousBase {

    public FarAutoRedNine() {
        super(new Pose(92.048, 0.355, Math.toRadians(90)), AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
            new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.goToArtifactRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeArtifacts, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootArtifactRowOne, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootArtifactRowOne.endPose()))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.goToSecondArtifactRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeSecondArtifactRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootSecondArtifactRow, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootSecondArtifactRow.endPose()))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(67))
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
                .addPath(new BezierLine(new Pose(92.048, 0.355), new Pose(87.908, 4.177)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                .build();

            goToArtifactRow = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(87.908, 4.177), new Pose(105.900, 27.3))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .build();

            intakeArtifacts = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(105.900, 27.3), new Pose(134.100, 27.3))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            shootArtifactRowOne = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(134.100, 27.3), new Pose(87.908, 4.177))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();

            goToSecondArtifactRow = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(87.908, 4.177), new Pose(137, 20))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(-90))
                .build();

            intakeSecondArtifactRow = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(137, 20), new Pose(137, 1.3))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

            shootSecondArtifactRow = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(137, 1.3), new Pose(87.908, 4.177))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(67))
                .build();

            leaveLaunchZone = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(87.908, 4.177), new Pose(110.840, 4.018))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .build();
        }
    }
}
