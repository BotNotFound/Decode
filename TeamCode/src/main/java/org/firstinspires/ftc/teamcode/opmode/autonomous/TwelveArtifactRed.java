package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "12 Artifact Red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class TwelveArtifactRed extends AutonomousBase {
    public TwelveArtifactRed() {
        super((new Pose(22.000, 120.000, Math.toRadians(-90)).mirror()), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.SHOOT),
                new AutonomousStage(autoPath.goToFirstRowArtifacts, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.SHOOT),
                new AutonomousStage(autoPath.goToSecondRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeSecondRowArtifacts, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.SHOOT),
                new AutonomousStage(autoPath.goToThirdRow, Robot.RobotState.NONE),
                new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.SHOOT)
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
                            new BezierLine((new Pose(22.000, 120.000).mirror()), (new Pose(50.900, 93.400)).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-132))
                    .build();

            goToFirstRowArtifacts = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.900, 93.400).mirror(),
                                    new Pose(53.900, 93.300).mirror(),
                                    new Pose(39.000, 84.500).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-132), Math.toRadians(-180))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.000, 84.500).mirror(), new Pose(13.200, 84.500).mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.200, 84.500).mirror(), new Pose(35.800, 107.800).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-132))
                    .build();

            goToSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(35.800, 107.800).mirror(),
                                    new Pose(42.300, 64.300).mirror(),
                                    new Pose(40.300, 60.000).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-132), Math.toRadians(-180))
                    .build();

            intakeSecondRowArtifacts = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.300, 60.000).mirror(), new Pose(7.400, 59.700).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(7.400, 59.700).mirror(),
                                    new Pose(61.000, 60.700).mirror(),
                                    new Pose(57.600, 87.400).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-132))
                    .build();

            goToThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.600, 87.400).mirror(), new Pose(40.300, 35.700).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-133), Math.toRadians(-180))
                    .build();

            intakeThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.300, 35.700).mirror(), new Pose(7.300, 35.700).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(7.300, 35.700).mirror(), new Pose(52.400, 89.600).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-133))
                    .build();
        }
    }
}
