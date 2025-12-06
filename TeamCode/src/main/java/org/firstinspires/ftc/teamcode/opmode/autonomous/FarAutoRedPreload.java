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
    //use position logger for positions
    public FarAutoRedPreload() {
        super(new Pose(89.5, 10.1, Math.toRadians(90)), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.NONE),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(64.7))
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
                                    new Pose(89.5, 10.1),
                                    new Pose(86.7, 18.03)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(64.7))
                    .build();

            leavingLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.7, 18.03), new Pose(108.9, 8.53))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(64.7), Math.toRadians(0))
                    .build();
        }
    }
}
