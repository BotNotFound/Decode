package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "12ArtifactAutoBlue", group = "autonomous")
public class TwelveArtifactBlue extends OpMode {
    /**
     * The various states of the robot autonomous
     */
    enum AutonomousState {
        /**
         * case 0, goes to the shooting position for preload
         */
        BEGIN,

        /**
         * shoots the preloads, spits out the remaining balls if there are any
         * then it goes to the position where its ready to pick up the next three artifacts
         */
        SHOOT_PRELOADS,
    }

    private Follower follower;
    private AutonomousState pathState = AutonomousState.BEGIN;
    private final Pose startPose = new Pose(12.0, 12.0, Math.toRadians(0));


    private PathChain firstCycleChain;
    private PathChain secondCycleChain;
    private PathChain thirdCycleChain;
    private PathChain fourthCycleChain;
    private PathChain fifthCycleChain;
    private PathChain sixthCycleChain;
    private PathChain seventhCycleChain;


// Mechanism subsystem references
    private FieldCentricDriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private AprilTagDetector aprilTagDetector;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);
        
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

                if(shooter.isReady()) {
                    shooter.engageKicker();
                    transfer.startTransfer();
                    intake.startIntake();
                }
                //if balls are stuck then reverse the transfer so we don't pick up more than 3
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                follower.followPath(secondCycleChain, true);
                pathState = 2;
                break;
            /* case 2, intakes the first three artifacts, goes to the shooting position for the first three artifacts
             *
             */
            case 2:
                intake.startIntake();
                transfer.startTransfer();
                follower.followPath(thirdCycleChain, true);
                pathState = 3;
                break;
            /*case 3
             * at the shooting position of the robot, it shoots the first row of three atifacts collected
             * goes to the next position ready to collect the next second row of three artifacts
             */
            case 3:
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
                pathState = 4;
                break;
        /* case 4
        Here, the robot picks up that second row of artifacts and then shoots
         */
            case 4:
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
                pathState = 5;
                /* case 5
                 * go to the next position ready to pick up that third and last row of artifacts
                 */
            case 5:
                intake.startIntake();
                transfer.startTransfer();
                follower.followPath(sixthCycleChain, true);
                pathState = 6;
                break;
        }
    }

    private void buildPaths() {
        // Your path chain definitions...
        Pose start = startPose;
        //adjust these positions accordingly
        final Pose shootPreloadedArtifacts = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose firstControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose firstThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose intakeFirstThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose shootFirstThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose secondControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose secondThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose intakeSecondThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose thirdControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose shootSecondThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose fourthControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose thirdThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose intakeLastThreeArtifactsPose = new Pose(24.0, 12.0, Math.toRadians(0));
        final Pose shootThirdThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));

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
