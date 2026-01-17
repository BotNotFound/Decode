package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "12 Artifact Red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class StandardTwelveArtifactRed extends AutonomousBase {

    public static Pose startPosition = new Pose(115.5, 129.4, Math.toRadians(36));

    public static double headingToShoot = Math.toRadians(-60);

    public StandardTwelveArtifactRed() {
        super(startPosition, AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
            new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                    .setConstantHeadingInterpolation(headingToShoot)
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.hitGate, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPath.shootFirstRow.endPose()))
                    .setConstantHeadingInterpolation(headingToShoot)
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.intakeSecondRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPath.shootSecondRow.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(35))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPath.shootThirdRow.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(35))
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
                        startPosition,
                        new Pose(94, 86.5)
                    )
                )
                .setLinearHeadingInterpolation(startPosition.getHeading(), headingToShoot)
                .build();

            intakeFirstRow = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(94, 86.5),
                        new Pose(94, 82),
                        new Pose(96, 82)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(

                    new BezierLine(new Pose(96, 82), new Pose(132.5, 82))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
            hitGate = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(132.5, 82),
                        new Pose(115, 77),
                        new Pose(128, 72)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

            shootFirstRow = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(128, 72),
                        new Pose(94, 86.5)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), headingToShoot)
                .build();

            intakeSecondRow = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(94, 86.5),
                        new Pose(94, 58.5),
                        new Pose(96, 58.5)
                    )
                )
                .setLinearHeadingInterpolation(headingToShoot, Math.toRadians(0))
                .addPath(
                    new BezierCurve(new Pose(96, 58.5), new Pose(138.5, 58.5), new Pose(135, 55))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
            shootSecondRow = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(135, 58.5),
                        new Pose(111, 53),
                        new Pose(94, 86.5)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), headingToShoot)
                .build();

            intakeThirdRow = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(94, 86.5),
                        new Pose(94, 35),
                        new Pose(96, 35)
                    )
                )
                .setLinearHeadingInterpolation(headingToShoot, Math.toRadians(0))
                .addPath(
                    new BezierLine(new Pose(96, 35), new Pose(135, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            shootThirdRow = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(135, 35),
                        new Pose(94, 86.5)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), headingToShoot)
                .build();

            leaveLaunchZone = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(94, 86.5),
                        new Pose(114.091, 71.748),
                        new Pose(114.091, 71.748)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        }
    }
}

