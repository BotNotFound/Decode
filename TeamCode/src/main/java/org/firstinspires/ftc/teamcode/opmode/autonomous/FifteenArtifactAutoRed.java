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

//TODO: Please tune the  positions and headings using PositionLogger, do not remove this TODO unless done so.
@Autonomous(name = "Fifteen Artifact Auto Red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class FifteenArtifactAutoRed extends AutonomousBase{

    public FifteenArtifactAutoRed() {
        super(new Pose(115.5, 129.4, 36), AllianceColor.RED);

    }

    protected AutonomousStage[] buildStageSequence() {
        Paths autoPaths = new Paths(follower);
        return new AutonomousStage[] {
                new AutonomousStage(autoPaths.shootPreloads, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPaths.shootPreloads.endPose()))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                        , Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPaths.intakeFirstRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPaths.shootFirstRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPaths.shootFirstRow.endPose()))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                        , Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPaths.intakeSecondRow, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPaths.shootSecondRow, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPaths.shootSecondRow.endPose()))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                        , Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPaths.firstGateCycleIntake, Robot.RobotState.GATE_INTAKE),
                new AutonomousStage(autoPaths.firstGateCycleShoot, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPaths.firstGateCycleShoot.endPose()))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                        ,Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPaths.secondGateCycleIntake, Robot.RobotState.GATE_INTAKE),
                new AutonomousStage(autoPaths.secondGateCycleShoot, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(follower.pathBuilder().addPath(new BezierPoint(autoPaths.secondGateCycleShoot.endPose()))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
                        , Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPaths.leaveLaunchZone, Robot.RobotState.NONE),
        };
    }


    public static class Paths {
        public PathChain shootPreloads;
        public PathChain intakeFirstRow;
        public PathChain shootFirstRow;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain firstGateCycleIntake;
        public PathChain firstGateCycleShoot;
        public PathChain secondGateCycleIntake;
        public PathChain secondGateCycleShoot;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.570, 122.661),

                                    new Pose(93.272, 93.630)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            intakeFirstRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(93.272, 93.630),
                                    new Pose(97.589, 83.164),
                                    new Pose(129.994, 83.967)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.994, 83.967),

                                    new Pose(89.705, 89.222)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intakeSecondRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.705, 89.222),
                                    new Pose(95.327, 55.111),
                                    new Pose(128.834, 59.578)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootSecondRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.834, 59.578),

                                    new Pose(87.133, 86.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            firstGateCycleIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.133, 86.131),

                                    new Pose(132.325, 61.222)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            firstGateCycleShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.325, 61.222),

                                    new Pose(87.459, 86.173)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            secondGateCycleIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.459, 86.173),

                                    new Pose(132.366, 61.015)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            secondGateCycleShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.366, 61.015),

                                    new Pose(87.443, 86.121)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            leaveLaunchZone = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.443, 86.121),

                                    new Pose(103.214, 76.341)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }

}
