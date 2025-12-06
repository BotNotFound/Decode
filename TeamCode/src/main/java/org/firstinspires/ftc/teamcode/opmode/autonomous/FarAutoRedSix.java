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

@Autonomous(name = "far auto red six artifacts", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class FarAutoRedSix extends AutonomousBase{

    public FarAutoRedSix() {
        super(new Pose(89.5, 10.1, Math.toRadians(90)), AllianceColor.RED);
    }
    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                    follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(64.7))
                            .build(),
                    Robot.RobotState.SHOOT
            ),
                new AutonomousStage(autoPath.goToArtifactRowOne, Robot.RobotState.NONE),
//            new AutonomousStage(autoPath.goToArtifactRowTwo, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeArtifacts, Robot.RobotState.INTAKE),
                new AutonomousStage(autoPath.shootArtifacts, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                    follower.pathBuilder().addPath(new BezierPoint(autoPath.shootArtifacts.endPose()))
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                            .build(),
                    Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.INTAKE),
        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain goToArtifactRowOne;
        public PathChain goToArtifactRowTwo; 
        public PathChain intakeArtifacts;
        public PathChain shootArtifacts;
        public PathChain leaveLaunchZone;
      
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
      
          goToArtifactRowOne = follower
            .pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(86.700, 18.030),
                            new Pose(130.000, 6.000),
                            new Pose(137.000, 20.000)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(64.7), Math.toRadians(-90))
            .build();
            
          goToArtifactRowTwo = follower
            .pathBuilder()
            .addPath (
              new BezierLine(
                new Pose(126.5, 17.5),
                      new Pose(137, 23.25)
              )
            )
            .setConstantHeadingInterpolation(Math.toRadians(-90))
            .build();
          //the end pose should be adjusted with Position Logger
          intakeArtifacts = follower
            .pathBuilder()
            .addPath(
                    new BezierLine(new Pose(137, 23.25), new Pose(147, 11))
            )
            .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-110))
            .build();
      
          shootArtifacts = follower
            .pathBuilder()
            .addPath(
              new BezierLine(new Pose(147, 11), new Pose(90.562, 16.980))
            )
            .setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(64.7))
            .build();
      
          leaveLaunchZone = follower
            .pathBuilder()
            .addPath(
              new BezierLine(new Pose(90.562, 16.980), new Pose(105.711, 10.155))
            )
            .setLinearHeadingInterpolation(Math.toRadians(64.7), Math.toRadians(0))
            .build();
        }
      }
}
