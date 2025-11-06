package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.module.AprilTagDetector;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Transfer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


// import your Constants class (with tuned PIDF etc)
// import your Robot hardware class (intake, shooter, deposit mechanism)

@Autonomous(name="12ArtifactAutoBlue", group="autonomous")
public class TwelveArtifactAuto_DECODE extends OpMode {
    private Follower follower;
    private int pathState = 0;
    private Pose startPose;


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
    private TeleOpRobot robot;

@Override
public void init(){
        follower = new Follower(hardwareMap);
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);

        // Example starting pose: adjust to your actual start corner/orientation
        startPose = new Pose(12.0, 12.0, Math.toRadians(0)); 
        
        follower.setStartingPose(startPose);

        // TODO: load your tuned constants into FollowerConstants here
        // e.g., FollowerConstants.translationalPIDFCoeffs.setCoefficients(P,I,D,F);

        buildPaths();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
}

@Override
public void start(){
    pathState= 0;

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
    switch(pathState){
        // case 0, goes to the shooting position for preload
        case 0:
            
            follower.followPath(firstCycleChain, true);
            pathState = 1;
            break;
        /*case 1, shoots the preloads, spits out the remaining balls if there are any
        then it goes to the position where its ready to pick up the next three artifacts
        */
        case 1:
            if(!follower.isBusy()) {
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
            }
            break;
        /* case 2, intakes the first three artifacts, goes to the shooting position for the first three artifacts
         * 
         */
        case 2:
            if(!follower.isBusy()) {
                intake.startIntake();
                transfer.startTransfer();
                follower.followPath(thirdCycleChain, true);
                pathState=3;
            }  
            break;
        /*case 3
         * at the shooting position of the robot, it shoots the first row of three atifacts collected
         * goes to the next position ready to collect the next second row of three artifacts
         */
        case 3:
            if(!follower.isBusy()) {
           
            AprilTagPoseFtc target1 = aprilTagDetector.getTagPose(20); //20 for blue
                driveTrain.setPowerFacingAprilTag(0, 0, 0, target1); //not sure what to do 
                shooter.setRPMForAprilTag(target1);

                if(shooter.isReady()) {
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
            }
            break;
        /* case 4
        Here, the robot picks up that second row of artifacts and then shoots
         */ 
        case 4: 
            if(!follower.isBusy()) {
                intake.startIntake();
                transfer.startTransfer();
                follower.followPath(fifthCycleChain, true);
                //now at a good shooting position
                AprilTagPoseFtc target2 = aprilTagDetector.getTagPose(20); //20 for blue
                driveTrain.setPowerFacingAprilTag(0, 0, 0, target2); //not sure what to do 
                shooter.setRPMForAprilTag(target2);

                if(shooter.isReady()) {
                    shooter.engageKicker();
                    transfer.startTransfer();
                    intake.startIntake();
                }
                //if balls are stuck then reverse the transfer so we don't pick up more than 3
                shooter.disengageKicker();
                intake.setPower(-1);
                transfer.reverseTransfer();
                pathState = 5;
            }
            /* case 5
             * go to the next position ready to pick up that third and last row of artifacts
             */
            case 5:
                if(!follower.isBusy()) {
                    intake.startIntake();
                    transfer.startTransfer();
                    follower.followPath(sixthCycleChain, true);
                    pathState = 6;
                }
            break;
            
        default:
            // do nothing or idle
            break;
    }
}
    private void buildPaths(){
        // Your path chain definitions...
        Pose start= startPose;
        //adjust these positions accordingly
        Pose shootPreloadedArtifacts= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose firstControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        Pose firstThreeArtifactsPose= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose intakeFirstThreeArtifactsPose= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose shootFirstThreeArtifactPose = new Pose(24.0, 12.0, Math.toRadians(0));
        Pose secondControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        Pose secondThreeArtifactPose= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose intakeSecondThreeArtifactsPose= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose thirdControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        Pose shootSecondThreeArtifactPose= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose fourthControlPoint = new Pose(24.0, 12.0, Math.toRadians(0));
        Pose thirdThreeArtifactPose= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose intakeLastThreeArtifactsPose= new Pose(24.0, 12.0, Math.toRadians(0));
        Pose shootThirdThreeArtifactPose =new Pose(24.0, 12.0, Math.toRadians(0));

        // FIRST CYCLE: Shooting the preloaded artifacts
    {
        PathChain.Builder builder = follower.pathBuilder();
        builder
            .addPath(new BezierLine(new Point(start), new Point(shootPreloadedArtifacts)))
            .setLinearHeadingInterpolation(start.getHeading(), shootPreloadedArtifacts.getHeading());
        firstCycleChain = builder.build();
    }
    // SECOND CYCLE: Going to the next first three artifacts
    {
        PathChain.Builder builder = follower.pathBuilder();
        builder
            .addPath(new BezierCurve(new Point(shootPreloadedArtifacts), new Point(firstControlPoint), new Point(firstThreeArtifactsPose)))
            .setLinearHeadingInterpolation(shootPreloadedArtifacts.getHeading(), firstThreeArtifactsPose.getHeading());
        secondCycleChain = builder.build();
    }

    // CYCLE: Shooting the next first three artifacts
    {
        PathChain.Builder builder = follower.pathBuilder();
        builder
            .addPath(new BezierLine(new Point(firstThreeArtifactsPose), new Point(intakeFirstThreeArtifactsPose)))
            .setLinearHeadingInterpolation(firstThreeArtifactsPose.getHeading(), intakeFirstThreeArtifactsPose.getHeading());
            .addPath(new BezierLine(new Point(intakeFirstThreeArtifactsPose), new Point(shootFirstThreeArtifactPose)))
            .setLinearHeadingInterpolation(intakeFirstThreeArtifactsPose.getHeading(), shootFirstThreeArtifactPose.getHeading());
                
        thirdCycleChain = builder.build();
    }


    // CYCLE: Picking up the next second three artifacts
    {
        PathChain.Builder builder = follower.pathBuilder();
        builder
            .addPath(new BezierCurve(new Point(shootFirstThreeArtifactPose), new Point(secondControlPoint), new Point(secondThreeArtifactPose)))
            .setLinearHeadingInterpolation(shootFirstThreeArtifactPose.getHeading(), secondThreeArtifactPose.getHeading())
        fourthCycleChain = builder.build();
    }

    //CYCLE: Shooting the next second three artifacts 

    {
        PathChain.Builder builder = follower.pathBuilder();
        builder
            .addPath(new BezierLine(new Point(secondThreeArtifactPose), new Point(intakeSecondThreeArtifactsPose)))
            .setLinearHeadingInterpolation(secondThreeArtifactPose.getHeading(), intakeSecondThreeArtifactsPose.getHeading())
            .addPath(new BezierCurve(new Point(intakeSecondThreeArtifactsPose), new Point(thirdControlPoint), new Point(shootSecondThreeArtifactPose)))
            .setLinearHeadingInterpolation(intakeSecondThreeArtifactsPose.getHeading(), shootSecondThreeArtifactPose.getHeading());

            fifthCycleChain = builder.build();
        
    }

    // CYCLE : Picking up the last three artifacts
    {
        PathChain.Builder builder = follower.pathBuilder();
        builder
        .addPath(new BezierCurve(new Point(shootSecondThreeArtifactPose), new Point(fourthControlPoint), new Point(thirdThreeArtifactPose)))
        .setLinearHeadingInterpolation(shootSecondThreeArtifactPose.getHeading(), thirdThreeArtifactPose.getHeading())

        sixthCycleChain = builder.build();
    }

    //CYCLE: Shooting the last three artifacts
    {
        PathChain.Builder builder= follower.pathBuilder();
        builder
            .addPath(new BezierLine(new Point(thirdThreeArtifactPose), new Point(intakeLastThreeArtifactsPose)))
            .setLinearHeadingInterpolation(thirdThreeArtifactPose.getHeading(), intakeLastThreeArtifactsPose.getHeading())
            .addPath(new BezierLine(new Point(intakeLastThreeArtifactsPose), new Point(shootThirdThreeArtifactPose)));
            .setLinearHeadingInterpolation(intakeLastThreeArtifactsPose.getHeading(), shootThirdThreeArtifactPose.getHeading());
                
        seventhCycleChain = builder.build();
    }
        
    }
}
