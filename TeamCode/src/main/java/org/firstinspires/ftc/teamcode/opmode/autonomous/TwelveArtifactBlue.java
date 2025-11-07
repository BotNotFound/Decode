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
@Autonomous(name = "12ArtifactAutoBlue", group = "autonomous")
public class TwelveArtifactBlue extends OpMode {
    /**
     * The various states of the robot autonomous
     */
    enum AutonomousState {
        /**
         * case 0: goes to the shooting position for preload
         */
        BEGIN,

        /**
         * case 1: shoots the preloads, spits out the remaining balls if there are any
         * then it goes to the position where its ready to pick up the next three artifacts
         */
        SHOOT_PRELOADS,
        /**
         * case 2: intakes the first three artifacts, goes to the shooting position for the first three artifacts
         */
        INTAKE_ROW_1,

        /**
         * case 3: at the shooting position of the robot, it shoots the first row of three artifacts
         * collected, then goes to the next position ready to collect the next second row of three artifacts
         */
        SHOOT,

        /**
         * case 4: Here, the robot picks up that second row of artifacts and then shoots
         */
        INTAKE_AND_SHOOT_ROW_2,

        /**
         * case 5: go to the next position ready to pick up that third and last row of artifacts
         */
        INTAKE_AND_SHOOT_ROW_3,

        /**
         * Final case: idle the robot until auto completes
         */
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
            // case 0, goes to the shooting position for preload
            case BEGIN:

                follower.followPath(firstCycleChain, true);
                pathState = AutonomousState.SHOOT_PRELOADS;
                break;
        /*case 1, shoots the preloads, spits out the remaining balls if there are any
        then it goes to the position where its ready to pick up the next three artifacts
        */
            case SHOOT_PRELOADS:
                AprilTagPoseFtc target = aprilTagDetector.getTagPose(20); //20 for blue
                driveTrain.setPowerFacingAprilTag(0, 0, 0, target); //not sure what to do 
                shooter.setRPMForAprilTag(target);

                if (shooter.isReady()) {
                    shooter.engageKicker();
                    transfer.startTransfer();
                    intake.startIntake();
                }
                //if balls are stuck then reverse the transfer so we don't pick up more than 3
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                follower.followPath(secondCycleChain, true);
                pathState = AutonomousState.INTAKE_ROW_1;
                break;
            /* case 2, intakes the first three artifacts, goes to the shooting position for the first three artifacts
             *
             */
            case INTAKE_ROW_1:
                intake.startIntake();
                transfer.startTransfer();
                follower.followPath(thirdCycleChain, true);
                pathState = AutonomousState.SHOOT;
                break;
            /*case 3
             * at the shooting position of the robot, it shoots the first row of three atifacts collected
             * goes to the next position ready to collect the next second row of three artifacts
             */
            case SHOOT:
                AprilTagPoseFtc target1 = aprilTagDetector.getTagPose(20); //20 for blue
                driveTrain.setPowerFacingAprilTag(0, 0, 0, target1); //not sure what to do
                shooter.setRPMForAprilTag(target1);

                if (shooter.isReady()) {
                    shooter.engageKicker();
                    transfer.startTransfer();
                    intake.startIntake();
                }
                //if balls are stuck then reverse the transfer so we don't pick up more than 3
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                follower.followPath(fourthCycleChain, true);
                pathState = AutonomousState.INTAKE_AND_SHOOT_ROW_2;
                break;
        /* case 4
        Here, the robot picks up that second row of artifacts and then shoots
         */
            case INTAKE_AND_SHOOT_ROW_2:
                intake.startIntake();
                transfer.startTransfer();
                follower.followPath(fifthCycleChain, true);
                //now at a good shooting position
                AprilTagPoseFtc target2 = aprilTagDetector.getTagPose(20); //20 for blue
                driveTrain.setPowerFacingAprilTag(0, 0, 0, target2); //not sure what to do
                shooter.setRPMForAprilTag(target2);

                if (shooter.isReady()) {
                    shooter.engageKicker();
                    transfer.startTransfer();
                    intake.startIntake();
                }
                //if balls are stuck then reverse the transfer so we don't pick up more than 3
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                pathState = AutonomousState.INTAKE_AND_SHOOT_ROW_3;
                /* case 5
                 * go to the next position ready to pick up that third and last row of artifacts
                 */
            case INTAKE_AND_SHOOT_ROW_3:
                intake.startIntake();
                transfer.startTransfer();
                follower.followPath(sixthCycleChain, true);
                pathState = AutonomousState.COMPLETE;
                break;
        }
    }

    private void buildPaths() {
        // Your path chain definitions...
        startPose = new Pose(24.0, 12.0, Math.toRadians(0));
        //adjust these positions accordingly
        shootPreloadedArtifacts = new Pose(24.0, 12.0, Math.toRadians(0));
        firstControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        firstThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        intakeFirstThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        shootFirstThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        secondControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        secondThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        intakeSecondThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        thirdControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        shootSecondThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        fourthControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        thirdThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        intakeLastThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        shootThirdThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));

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

        // CYCLE: Shooting the next first three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierLine(firstThreeArtifactsPose, intakeFirstThreeArtifactsPose))
                    .setLinearHeadingInterpolation(firstThreeArtifactsPose.getHeading(), intakeFirstThreeArtifactsPose.getHeading())
                    .addPath(new BezierLine(intakeFirstThreeArtifactsPose, shootFirstThreeArtifactPose))
                    .setLinearHeadingInterpolation(intakeFirstThreeArtifactsPose.getHeading(), shootFirstThreeArtifactPose.getHeading());

            thirdCycleChain = builder.build();
        }


        // CYCLE: Picking up the next second three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierCurve(shootFirstThreeArtifactPose, secondControlPoint, secondThreeArtifactPose))
                    .setLinearHeadingInterpolation(shootFirstThreeArtifactPose.getHeading(), secondThreeArtifactPose.getHeading());
            fourthCycleChain = builder.build();
        }

        //CYCLE: Shooting the next second three artifacts

        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierLine(secondThreeArtifactPose, intakeSecondThreeArtifactsPose))
                    .setLinearHeadingInterpolation(secondThreeArtifactPose.getHeading(), intakeSecondThreeArtifactsPose.getHeading())
                    .addPath(new BezierCurve(intakeSecondThreeArtifactsPose, thirdControlPoint, shootSecondThreeArtifactPose))
                    .setLinearHeadingInterpolation(intakeSecondThreeArtifactsPose.getHeading(), shootSecondThreeArtifactPose.getHeading());

            fifthCycleChain = builder.build();

        }

        // CYCLE : Picking up the last three artifacts
        {
            PathBuilder builder = follower.pathBuilder();
            builder
                    .addPath(new BezierCurve(shootSecondThreeArtifactPose, fourthControlPoint, thirdThreeArtifactPose))
                    .setLinearHeadingInterpolation(shootSecondThreeArtifactPose.getHeading(), thirdThreeArtifactPose.getHeading());

            sixthCycleChain = builder.build();
        }

        //CYCLE: Shooting the last three artifacts
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
