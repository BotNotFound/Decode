package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStage;
import com.acmerobotics.dashboard.config.Config;

/*Purpose of this test: We test if by setting multiple pathchains while intaking the robot slows down */
@Config
public class AutoConfig{
    public static int speedFactor = 3; //the number of path chains we create
}

@Autonomous(group = "test")
public class SlowStraightDriveTest extends AutonomousBase {
    int speedFactor = AutoConfig.speedFactor;

    Pose startPosition = new Pose(90, 37);
    Pose endPosition = new Pose(140, 37);

    private double[] array= new double[speedFactor];
    private AutonomousStage[] pathTest = new AutonomousStage[speedFactor];


    public SlowStraightDriveTest() {
        //adjust starting position
        super(startPosition, Robot.AllianceColor.RED);
    }
    //end point 140.000, 84.000
    @Override
    protected AutonomousStage[] buildStageSequence() {
        buildStages();
        return pathTest;
    }


    private void buildStages() {
        double diffX = endPosition.getX()-startPosition.getX();
        double avg = diffX/speedFactor;
        for(int i=1; i<= speedFactor; i++){
            double oldX = startPosition.getX()+(i-1)*avg;
            double newX = startPosition.getX()+i*avg;
            //add the new Path
            pathTest[i-1]= new AutonomousStage(follower.pathBuilder().addPath(
                new BezierLine(new Pose(oldX, startPosition.getY()), new Pose(newX, startPosition.getY()))
                ).build(),
                Robot.RobotState.NONE);
    
        }
    
    }
}

