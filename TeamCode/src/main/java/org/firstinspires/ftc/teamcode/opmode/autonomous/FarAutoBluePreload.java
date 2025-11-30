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

@Autonomous(name = "far auto blue", group = "blue", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class FarAutoBluePreload extends AutonomousBase {

    public FarAutoBluePreload() {
        super(new Pose(84.432, 8.407, Math.toRadians(90)).mirror(), Robot.AllianceColor.BLUE);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
                new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setConstantHeadingInterpolation(Math.toRadians(180 - 72))
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
                                    new Pose(84.432, 8.407).mirror(),
                                    new Pose(86.758, 11.806).mirror(),
                                    new Pose(83.001, 11.091).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 90), Math.toRadians(180 - 72))
                    .build();

            leavingLaunchZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.001, 11.091).mirror(), new Pose(110.191, 13.058).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 72), Math.toRadians(180))
                    .build();
        }
    }
}
