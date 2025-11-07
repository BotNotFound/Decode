package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/*
 * Step 1: make all the points public static so they can be configured on dash
 * Step 2: Use Robot robot
 * Step 3: set up pathStates using enum
 * Step 4: change pose values according to visualizer
 */
@Config
@Autonomous(name = "12ArtifactAutoRed", group = "autonomous")
public class TwelveArtifactRed extends OpMode {
    /**
     * The various states of the robot autonomous
     */
    enum AutonomousState {
        BEGIN,
        SHOOT_PRELOADS,
        INTAKE_ROW_1_AND_SHOOT,
        INTAKE_SECOND_ROW,
        SHOOT_ROW_2,
        INTAKE_ROW_3,
        SHOOT_ROW_3,
        COMPLETE
    }

    private Follower follower;
    private AutonomousState pathState = AutonomousState.BEGIN;

    //all the positions
    public static Pose startPose, shootPreloadedArtifacts, firstControlPoint, firstThreeArtifactsPose, intakeFirstThreeArtifactsPose, shootFirstThreeArtifactPose, secondControlPoint, secondThreeArtifactPose, intakeSecondThreeArtifactsPose, thirdControlPoint, shootSecondThreeArtifactPose, fourthControlPoint, thirdThreeArtifactPose, intakeLastThreeArtifactsPose, shootThirdThreeArtifactPose;


    private PathChain firstCycleChain;
    private PathChain secondCycleChain;
    private PathChain thirdCycleChain;
    private PathChain fourthCycleChain;
    private PathChain fifthCycleChain;
    private PathChain sixthCycleChain;
    private PathChain seventhCycleChain;


// Mechanism subsystem references

    private Robot robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, Robot.AllianceColor.RED);

        follower.setStartingPose(startPose);

        buildPaths();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = AutonomousState.BEGIN;
    }

    @Override
    public void loop() {
        follower.update();
        runStateMachine();

        Pose cur = follower.getPose();
        telemetry.addData("X", cur.getX());
        telemetry.addData("Y", cur.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(cur.getHeading()));
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    private void runStateMachine() {
        if (follower.isBusy()) {
            return;
        }

        switch (pathState) {
            // goes to the shooting position for preload
            //this is good
            case BEGIN:

                follower.followPath(firstCycleChain, true);
                pathState = AutonomousState.SHOOT_PRELOADS;
                break;
        /*shoots the preloads, spits out the remaining balls if there are any
        then it goes to the position where its ready to pick up the next three artifacts
        */
        //this is good
            case SHOOT_PRELOADS:
                robot.setState(Robot.RobotState.SHOOT);
                robot.setState(Robot.RobotState.NONE);
                follower.followPath(secondCycleChain, true);
                pathState = AutonomousState.INTAKE_ROW_1_AND_SHOOT;
                break;
            /* intakes the first three artifacts, goes to the shooting position for the first three artifacts
             *
             */
            //this is good
            case INTAKE_ROW_1_AND_SHOOT:
                robot.setState(Robot.RobotState.INTAKE);
                follower.followPath(thirdCycleChain, true);
                robot.setState(Robot.RobotState.SHOOT);
                pathState = AutonomousState.INTAKE_SECOND_ROW;
                break;
            /*
             * at the shooting position of the robot, it shoots the first row of three atifacts collected
             * goes to the next position ready to collect the next second row of three artifacts
             */
            // this is good
            case INTAKE_SECOND_ROW:
                robot.setState(Robot.RobotState.NONE);
                //if balls are stuck then reverse the transfer so we don't pick up more than 3
                robot.setState(Robot.RobotState.REVERSE_INTAKE);
                follower.followPath(fourthCycleChain, true);
                pathState = AutonomousState.SHOOT_ROW_2;
                break;
        /* 
        Here, the robot picks up that second row of artifacts and then shoots
         */
        // this is good
            case SHOOT_ROW_2:
                robot.setState(Robot.RobotState.INTAKE); 
                follower.followPath(fifthCycleChain, true);
                //now at a good shooting position
                robot.setState(Robot.RobotState.SHOOT);
                //if balls are stuck then reverse the transfer so we don't pick up more than 3
                robot.setState(Robot.RobotState.REVERSE_INTAKE);
                pathState = AutonomousState.INTAKE_ROW_3;
                /* 
                 * go to the next position ready to pick up that third and last row of artifacts
                 */
            //this is good
            case INTAKE_ROW_3:
                robot.setState(Robot.RobotState.NONE);
                robot.setState(Robot.RobotState.INTAKE);
                follower.followPath(sixthCycleChain, true);
                pathState = AutonomousState.SHOOT_ROW_3;
                break;
            /*
             * shoo the third and last row of artifacts
             */
            case SHOOT_ROW_3:
                follower.followPath(seventhCycleChain, true);
                robot.setState(Robot.RobotState.SHOOT);
                pathState = AutonomousState.COMPLETE;
                break;
            
            case COMPLETE:
              break; // we are done yay

        }
    }

    private void buildPaths() {
        // Your path chain definitions...
        startPose = new Pose(119.3, 124.2, Math.toDegrees(90));
        //adjust these positions accordingly
        shootPreloadedArtifacts = new Pose(95, 95, Math.toDegrees(51));
        firstControlPoint = new Pose(92.6, 96.2, Math.toDegrees(0));
        firstThreeArtifactsPose = new Pose(101.9, 84, Math.toDegrees(-1));
        intakeFirstThreeArtifactsPose = new Pose(127.7, 84, Math.toDegrees(-1));
        shootFirstThreeArtifactPose = new Pose(100, 106, Math.toDegrees(48));
        secondControlPoint = new Pose(100, 84, Math.toRadians(0));
        secondThreeArtifactPose = new Pose(100.9, 59.2, Math.toDegrees(-1));
        intakeSecondThreeArtifactsPose = new Pose(129.8, 59.5, Math.toDegrees(-1));
        thirdControlPoint = new Pose(83.2, 58.3, Math.toDegrees(0));
        shootSecondThreeArtifactPose = new Pose(92.1, 100.7, Math.toDegrees(43));
        fourthControlPoint = new Pose(98, 71, Math.toDegrees(0));
        thirdThreeArtifactPose = new Pose(103.8, 35.3, Math.toDegrees(0));
        intakeLastThreeArtifactsPose = new Pose(134.4, 35.3, Math.toDegrees(0));
        shootThirdThreeArtifactPose = new Pose(86.7, 89.5, Math.toDegrees(47));

        // FIRST CYCLE: Shooting the preloaded artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierLine(start, shootPreloadedArtifacts))
                    .setLinearHeadingInterpolation(start.getHeading(), shootPreloadedArtifacts.getHeading());
            firstCycleChain = builder.build();
        }
        // SECOND CYCLE: Going to the next first three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierCurve(shootPreloadedArtifacts, firstControlPoint, firstThreeArtifactsPose))
                    .setLinearHeadingInterpolation(shootPreloadedArtifacts.getHeading(), firstThreeArtifactsPose.getHeading());
            secondCycleChain = builder.build();
        }

        // THIRD CYCLE: Shooting the next first three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierLine(firstThreeArtifactsPose, intakeFirstThreeArtifactsPose))
                    .setLinearHeadingInterpolation(firstThreeArtifactsPose.getHeading(), intakeFirstThreeArtifactsPose.getHeading())
                    .addPath(new BezierLine(intakeFirstThreeArtifactsPose, shootFirstThreeArtifactPose))
                    .setLinearHeadingInterpolation(intakeFirstThreeArtifactsPose.getHeading(), shootFirstThreeArtifactPose.getHeading());

            thirdCycleChain = builder.build();
        }


        // FORUTH CYCLE: Picking up the next second three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierCurve(shootFirstThreeArtifactPose, secondControlPoint, secondThreeArtifactPose))
                    .setLinearHeadingInterpolation(shootFirstThreeArtifactPose.getHeading(), secondThreeArtifactPose.getHeading());
            fourthCycleChain = builder.build();
        }

        // FIFTH CYCLE: Shooting the next second three artifacts

        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierLine(secondThreeArtifactPose, intakeSecondThreeArtifactsPose))
                    .setLinearHeadingInterpolation(secondThreeArtifactPose.getHeading(), intakeSecondThreeArtifactsPose.getHeading())
                    .addPath(new BezierCurve(intakeSecondThreeArtifactsPose, thirdControlPoint, shootSecondThreeArtifactPose))
                    .setLinearHeadingInterpolation(intakeSecondThreeArtifactsPose.getHeading(), shootSecondThreeArtifactPose.getHeading());

            fifthCycleChain = builder.build();

        }

        // SIXTH CYCLE : Picking up the last three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierCurve(shootSecondThreeArtifactPose, fourthControlPoint, thirdThreeArtifactPose))
                    .setLinearHeadingInterpolation(shootSecondThreeArtifactPose.getHeading(), thirdThreeArtifactPose.getHeading());

            sixthCycleChain = builder.build();
        }

        //SEVENTH CYCLE: Shooting the last three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierLine(thirdThreeArtifactPose, intakeLastThreeArtifactsPose))
                    .setLinearHeadingInterpolation(thirdThreeArtifactPose.getHeading(), intakeLastThreeArtifactsPose.getHeading())
                    .addPath(new BezierLine(intakeLastThreeArtifactsPose, shootThirdThreeArtifactPose))
                    .setLinearHeadingInterpolation(intakeLastThreeArtifactsPose.getHeading(), shootThirdThreeArtifactPose.getHeading());

            seventhCycleChain = builder.build();
        }

    }
}