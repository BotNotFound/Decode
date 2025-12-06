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

@Autonomous(name = "far auto red six artifacts", group = "red", preselectTeleOp = TwoPersonTeleOpRed.OP_MODE_NAME)
public class FarAutoRedSix extends AutonomousBase{

    public FarAutoRedSix() {
        super(new Pose(86.6, 8.000, Math.toRadians(90)), Robot.AllianceColor.RED);
    }
    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
                new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.PRE_SHOOT),
            new AutonomousStage(
                        follower.pathBuilder().addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                                .build(),
                        Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.goToArtifactRow, Robot.RobotState.NONE),
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
        public PathChain goToArtifactRow;
        public PathChain intakeArtifacts;
        public PathChain shootArtifacts;
        public PathChain leaveLaunchZone;
      
        public Paths(Follower follower) {
          shootPreloads = follower
            .pathBuilder()
            .addPath(
              new BezierCurve(
                new Pose(86.600, 8.407),
                new Pose(86.758, 11.806),
                new Pose(82.700, 11.091)
              )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
            .build();
      
          goToArtifactRow = follower
            .pathBuilder()
            .addPath(
                    //use position logger
              new BezierLine(
                new Pose(82.700, 11.091),
                new Pose(141.836, 29.133)
              )
            )
            .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(-90))
            .build();
      
          intakeArtifacts = follower
            .pathBuilder()
            .addPath(
              new BezierLine(new Pose(141.836, 29.133), new Pose(141.336, 2.164))
            )
            .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
            .build();
      
          shootArtifacts = follower
            .pathBuilder()
            .addPath(
              new BezierLine(new Pose(141.336, 2.164), new Pose(90.562, 16.980))
            )
            .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(67))
            .build();
      
          leaveLaunchZone = follower
            .pathBuilder()
            .addPath(
              new BezierLine(new Pose(90.562, 16.980), new Pose(105.711, 10.155))
            )
            .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
            .build();
        }
      }
}
