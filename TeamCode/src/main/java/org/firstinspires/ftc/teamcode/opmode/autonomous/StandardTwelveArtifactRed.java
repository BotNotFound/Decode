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

    public static Pose startPosition = new Pose(117.9, 131.3, Math.toRadians(-54));

    public static double headingToShoot = Math.toRadians(-40);

    public StandardTwelveArtifactRed() {
        super(startPosition, AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
            new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                    .setConstantHeadingInterpolation(headingToShoot)
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.moveToFirstRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.hitGate, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootFirstRow.endPose()))
                    .setConstantHeadingInterpolation(headingToShoot)
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.moveToSecondRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeSecondRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootSecondRow.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(35))
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.moveToThirdRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootThirdRow.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(35))
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.NONE),

        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain moveToFirstRow;
        public PathChain intakeFirstRow;
        public PathChain hitGate;
        public PathChain shootFirstRow;
        public PathChain moveToSecondRow;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain moveToThirdRow;
        public PathChain intakeThirdRow;
        public PathChain shootThirdRow;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower.pathBuilder()
                .addPath(new BezierLine(startPosition, new Pose(88.6617, 97.5312)))
                .setLinearHeadingInterpolation(startPosition.getHeading(), headingToShoot)
                .build();

            moveToFirstRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(88.6617, 97.5312),
                    new Pose(98, 87),
                    new Pose(100.9081, 87.3834)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

            intakeFirstRow = follower.pathBuilder()
                .addPath(
                    new BezierLine(new Pose(100.9081, 87.3834), new Pose(123.83, 87.267))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

            hitGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(123.83, 87.267),
                    new Pose(115, 79),
                    new Pose(128, 78)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

            shootFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(128, 78), new Pose(92.3, 99.48)))
                .setLinearHeadingInterpolation(Math.toRadians(0), headingToShoot)
                .build();

            moveToSecondRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(92.3, 99.48),
                    new Pose(96.42, 64.07),
                    new Pose(96.42, 64.07)
                ))
                .setLinearHeadingInterpolation(headingToShoot, Math.toRadians(0))
                .build();

            intakeSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96.42, 64.07), new Pose(120.25, 62.08)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

            shootSecondRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(120.25, 62.08),
                    new Pose(86.95, 95),
                    new Pose(86.95, 95.18)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), headingToShoot)
                .build();

            moveToThirdRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(86.95, 95.18),
                    new Pose(97.06, 41),
                    new Pose(97.06, 41.13)
                ))
                .setLinearHeadingInterpolation(headingToShoot, Math.toRadians(0))
                .build();

            intakeThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(97.06, 41.13), new Pose(125.18, 39.93)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            shootThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(125.18, 39.93), new Pose(86.95, 95.18)))
                .setLinearHeadingInterpolation(Math.toRadians(0), headingToShoot)
                .build();

            leaveLaunchZone = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(86.95, 95.18),
                    new Pose(111.52, 78.55),
                    new Pose(111.52, 78.55)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        }
    }
}

