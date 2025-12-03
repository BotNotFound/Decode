package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpRed;

@Autonomous(name = "far auto red", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class FarAutoRedPreload extends AutonomousBase {

    public FarAutoRedPreload() {
        super(new Pose(86.6, 8.000, Math.toRadians(90)), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.NONE),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(72))
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
                            new BezierCurve(
                                    new Pose(86.6, 8.000),
                                    new Pose(86.758, 11.806),
                                    new Pose(83.001, 11.091)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72))
                    .build();

            leavingLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.001, 11.091), new Pose(110.191, 13.058))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(0))
                    .build();
        }
    }
}
