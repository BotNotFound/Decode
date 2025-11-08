package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

import java.util.Arrays;

@Autonomous(name = "6 Artifact Red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class SixArtifactRed extends AutonomousBase {
    public SixArtifactRed() {
        super(new Pose(111, 135, 0), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths paths = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(paths.preShootPreloads, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(paths.shootPreloads, Robot.RobotState.SHOOT),
                new AutonomousStage(paths.jigglePreloads, Robot.RobotState.NONE),
                new AutonomousStage(paths.reShootPreloads, Robot.RobotState.SHOOT),
                new AutonomousStage(paths.goToFirstRow, Robot.RobotState.NONE),
                new AutonomousStage(paths.intakeFirstRow, Robot.RobotState.INTAKE),
                new AutonomousStage(paths.preShootFirstRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(paths.shootFirstRow, Robot.RobotState.SHOOT),
                new AutonomousStage(paths.jiggleFirstRow, Robot.RobotState.NONE),
                new AutonomousStage(paths.reShootFirstRow, Robot.RobotState.SHOOT),
        };
    }

    public static class Paths {

        public PathChain preShootPreloads;
        public PathChain shootPreloads;
        public PathChain jigglePreloads;
        public PathChain reShootPreloads;
        public PathChain goToFirstRow;
        public PathChain intakeFirstRow;
        public PathChain preShootFirstRow;
        public PathChain shootFirstRow;
        public PathChain jiggleFirstRow;
        public PathChain reShootFirstRow;

        public Paths(Follower follower) {
            preShootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.000, 135.000), new Pose(95.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            shootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 100.000), new Pose(95.000, 100.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            jigglePreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 100.000), new Pose(85.000, 90.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            reShootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000, 90.000), new Pose(95.000, 100.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            goToFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 100.000), new Pose(100.000, 83.750))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.000, 83.750), new Pose(130.000, 83.750))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            preShootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.000, 83.750), new Pose(95.000, 100.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 100.000), new Pose(95.000, 100.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            jiggleFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 100.000), new Pose(85.000, 90.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            reShootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000, 90.000), new Pose(95.000, 100.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();
        }
    }
}
