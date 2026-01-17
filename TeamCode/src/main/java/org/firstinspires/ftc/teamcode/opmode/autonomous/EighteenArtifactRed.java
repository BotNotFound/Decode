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

@Autonomous(name = "18 Artifact Red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
//TODO: Please tune the  positions and headings using PositionLogger, do not remove this TODO unless done so.
public class EighteenArtifactRed extends AutonomousBase {
    public EighteenArtifactRed() {
        super(new Pose(115.5, 129.4, 36), AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
            new AutonomousStage(autoPath.shootPreloadArtifacts, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootPreloadArtifacts.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(44))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.intake2ndRowArtifacts, Robot.RobotState.INTAKE),
            new AutonomousStage(
                autoPath.shoot2ndRowArtifacts, Robot.RobotState.PRE_SHOOT), new AutonomousStage(
            follower.pathBuilder().addPath(new BezierPoint(autoPath.shoot2ndRowArtifacts.endPose()))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build(),
            Robot.RobotState.SHOOT
        ),
            new AutonomousStage(autoPath.firstCycleGateIntake, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.firstCycleGateIntake.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(20))
                    .build(),
                Robot.RobotState.GATE_INTAKE
            ),
            new AutonomousStage(autoPath.firstCycleGateIntakeShoot, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.firstCycleGateIntakeShoot.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(-45))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.secondCycleGateIntake, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.secondCycleGateIntake.endPose()))
                    .setConstantHeadingInterpolation(20)
                    .build(),
                Robot.RobotState.GATE_INTAKE
            ),
            new AutonomousStage(autoPath.secondCycleGateIntakeShoot, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.secondCycleGateIntakeShoot.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(-45))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.firstRowArtifacts, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.firstRowArtifactsShoot, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.firstRowArtifactsShoot.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(-60))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.thirdRowArtifactIntake, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.thirdRowArtifactIntakeShoot, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.thirdRowArtifactIntakeShoot.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(-50))
                    .build(),
                Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.NONE),

        };
    }

    public static class Paths {
        public PathChain shootPreloadArtifacts;
        public PathChain intake2ndRowArtifacts;
        public PathChain shoot2ndRowArtifacts;
        public PathChain firstCycleGateIntake;
        public PathChain firstCycleGateIntakeShoot;
        public PathChain secondCycleGateIntake;
        public PathChain secondCycleGateIntakeShoot;
        public PathChain firstRowArtifacts;
        public PathChain firstRowArtifactsShoot;
        public PathChain thirdRowArtifactIntake;
        public PathChain thirdRowArtifactIntakeShoot;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloadArtifacts = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(118.890, 127.724),

                        new Pose(90.989, 90.213)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(44))

                .build();

            intake2ndRowArtifacts = follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(90.989, 90.213),
                        new Pose(87.889, 56.267),
                        new Pose(128.553, 59.490)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))

                .build();

            shoot2ndRowArtifacts = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(128.553, 59.490),

                        new Pose(91.144, 90.368)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))

                .build();

            firstCycleGateIntake = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(91.144, 90.368),

                        new Pose(132.065, 63.552)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20))

                .build();

            firstCycleGateIntakeShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(132.065, 63.552),

                        new Pose(90.989, 90.523)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-45))

                .build();

            secondCycleGateIntake = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(90.989, 90.523),

                        new Pose(132.065, 63.707)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20))

                .build();

            secondCycleGateIntakeShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(132.065, 63.707),

                        new Pose(90.989, 90.368)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-45))

                .build();

            firstRowArtifacts = follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(90.989, 90.368),
                        new Pose(103.079, 82.463),
                        new Pose(129.344, 83.238)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))

                .build();

            firstRowArtifactsShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(129.344, 83.238),

                        new Pose(90.989, 90.368)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))

                .build();

            thirdRowArtifactIntake = follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(90.989, 90.368),
                        new Pose(101.467, 24.945),
                        new Pose(105.490, 35.502),
                        new Pose(128.537, 35.657)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(0))

                .build();

            thirdRowArtifactIntakeShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(128.537, 35.657),

                        new Pose(87.269, 84.788)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))

                .build();

            leaveLaunchZone = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(87.269, 84.788),

                        new Pose(95.058, 78.985)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(0))

                .build();
        }
    }


}
