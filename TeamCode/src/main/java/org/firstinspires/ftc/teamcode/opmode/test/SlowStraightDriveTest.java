package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStage;

@Config
@Autonomous(group = "test")
public class SlowStraightDriveTest extends AutonomousBase {
    public static int speedFactor = 3; //the number of path chains we create

    private static final Pose startPosition = new Pose(90, 37);
    private static final Pose endPosition = new Pose(140, 37);

    public SlowStraightDriveTest() {
        super(startPosition, Robot.AllianceColor.RED);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        final AutonomousStage[] pathTest = new AutonomousStage[speedFactor];
        double diffX = endPosition.getX() - startPosition.getX();
        double avg = diffX / speedFactor;
        for (int i = 1; i <= speedFactor; i++) {
            double oldX = startPosition.getX() + (i - 1) * avg;
            double newX = startPosition.getX() + i * avg;
            //add the new Path
            pathTest[i - 1] = new AutonomousStage(follower.pathBuilder().addPath(
                    new BezierLine(new Pose(oldX, startPosition.getY()), new Pose(newX, startPosition.getY()))
            ).build(),
                    Robot.RobotState.NONE);

        }
        return pathTest;
    }
}

