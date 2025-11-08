package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

@Autonomous(name = "12 Artifact Blue", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class TwelveArtifactBlue extends AutonomousBase {
    public TwelveArtifactBlue() {
        super(new Pose(22.000, 120.000), Robot.AllianceColor.BLUE);
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
                            new BezierLine(new Pose(22.000, 120.000), new Pose(50.900, 93.400))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                    .build();

            goToFirstRowArtifacts = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.900, 93.400),
                                    new Pose(53.900, 93.300),
                                    new Pose(39.000, 84.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.000, 84.500), new Pose(13.200, 84.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.200, 84.500), new Pose(35.800, 107.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                    .build();

            goToSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(35.800, 107.800),
                                    new Pose(42.300, 64.300),
                                    new Pose(40.300, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();

            intakeSecondRowArtifacts = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.300, 60.000), new Pose(7.400, 59.700))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(7.400, 59.700),
                                    new Pose(61.000, 60.700),
                                    new Pose(57.600, 87.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                    .build();

            goToThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.600, 87.400), new Pose(40.300, 35.700))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                    .build();

            intakeThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.300, 35.700), new Pose(7.300, 35.700))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(7.300, 35.700), new Pose(52.400, 89.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();
        }
    }
}

