package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

// arithmetic is applied uniformly to ensure everything is mirrored
@SuppressWarnings("PointlessArithmeticExpression")
@Autonomous(name = "far auto blue", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class FarAutoBluePreload extends AutonomousBase {
    public FarAutoBluePreload() {
        super(new Pose(89.5, 10.1, Math.toRadians(90)).mirror(), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.NONE),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(180-64.7))
                                .build(),
                        Robot.RobotState.SHOOT
                ),
                new AutonomousStage(autoPath.leavingLaunchZone, Robot.RobotState.NONE)
        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain leavingLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(89.5, 10.1).mirror(),
                                    new Pose(86.7, 18.03).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 90), Math.toRadians(180-64.7))
                    .build();

            leavingLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.7, 18.03).mirror(), new Pose(108.9, 8.53).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-64.7), Math.toRadians(180 - 0))
                    .build();
        }
    }
}
