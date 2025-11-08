package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStage;

@Autonomous(group = "test")
public class AutoTest extends AutonomousBase {
    public AutoTest() {
        super(new Pose(0, 0), Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        return new AutonomousStage[] {
                new AutonomousStage(
                        follower.pathBuilder().addPath(
                                new BezierLine(new Pose(0, 0), new Pose(10, 0))
                        ).build(),
                        Robot.RobotState.NONE
                ),
                new AutonomousStage(
                        follower.pathBuilder()
                                .addPath(new BezierPoint(new Pose(1, 0)))
                                .build(),
                        Robot.RobotState.SHOOT
                )
        };
    }
}
