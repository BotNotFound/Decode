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

//TODO: Please tune the  positions and headings using PositionLogger, do not remove this TODO unless done so.
@Autonomous(name = "3 Cycles Far Auto", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class ThreeCyclesFarAutoRed extends AutonomousBase {
    public ThreeCyclesFarAutoRed() {
        super(new Pose(118.890, 127.724, Math.toRadians(44)), AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPaths = new Paths(follower);
        return new AutonomousStage[]{
            new AutonomousStage(autoPaths.shootPreloads, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPaths.shootPreloads.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build()
                , Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPaths.firstStageIntakeOne, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.firstStageGoBack, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.firstStageIntakeTwo, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.firstStageShoot, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPaths.firstStageShoot.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build()
                , Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPaths.secondStageIntakeOne, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.secondStageGoBack, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.secondStageIntakeTwo, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.secondStageShoot, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPaths.secondStageShoot.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build()
                , Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPaths.thirdStageIntakeOne, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.thirdStageGoBack, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.thirdStageIntakeTwo, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPaths.thirdStageShoot, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                follower.pathBuilder().addPath(new BezierPoint(autoPaths.thirdStageShoot.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build()
                , Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPaths.leaveLaunchZone, Robot.RobotState.NONE),
        };

    }


    public static class Paths {
        public PathChain shootPreloads;
        public PathChain firstStageIntakeOne;
        public PathChain firstStageGoBack;
        public PathChain firstStageIntakeTwo;
        public PathChain firstStageShoot;
        public PathChain secondStageIntakeOne;
        public PathChain secondStageGoBack;
        public PathChain secondStageIntakeTwo;
        public PathChain secondStageShoot;
        public PathChain thirdStageIntakeOne;
        public PathChain thirdStageGoBack;
        public PathChain thirdStageIntakeTwo;
        public PathChain thirdStageShoot;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(88.419, 7.849),

                        new Pose(80.427, 15.644)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

            firstStageIntakeOne = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(80.427, 15.644),

                        new Pose(133.480, 7.697)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .build();

            firstStageGoBack = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(133.480, 7.697),

                        new Pose(125.738, 7.527)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            firstStageIntakeTwo = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(125.738, 7.527),

                        new Pose(141.352, 7.814)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            firstStageShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(141.352, 7.814),

                        new Pose(86.117, 9.459)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            secondStageIntakeOne = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(86.117, 9.459),

                        new Pose(140.286, 7.808)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            secondStageGoBack = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(140.286, 7.808),

                        new Pose(125.923, 7.613)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            secondStageIntakeTwo = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(125.923, 7.613),

                        new Pose(138.551, 7.809)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            secondStageShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(138.551, 7.809),

                        new Pose(86.059, 8.904)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            thirdStageIntakeOne = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(86.059, 8.904),

                        new Pose(137.287, 15.123)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            thirdStageGoBack = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(137.287, 15.123),

                        new Pose(86.152, 8.793)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            thirdStageIntakeTwo = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(86.152, 8.793),

                        new Pose(138.154, 7.880)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            thirdStageShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(138.154, 7.880),

                        new Pose(84.085, 12.372)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

            leaveLaunchZone = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(84.085, 12.372),

                        new Pose(84.419, 31.139)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        }
    }
}
