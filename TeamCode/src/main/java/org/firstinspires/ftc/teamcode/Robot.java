package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.module.ArtifactLocation;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.RobotLift;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Spindexer;
import org.firstinspires.ftc.teamcode.module.Turret;

public class Robot {
    private static final String TAG = "Robot";

    public static final Pose2D DEFAULT_ROBOT_POSE = new Pose2D(DistanceUnit.INCH, 125.5, 128, AngleUnit.DEGREES, 35);

    private static final class PersistentState {
        private static PersistentState saved = null;

        private final Pose2D robotPose;
        private final boolean[] artifactDetections;

        private PersistentState(Pose2D robotPose, boolean[] artifactDetections) {
            this.robotPose = robotPose;
            this.artifactDetections = artifactDetections;
        }

        public static void saveRobotState(Robot robot) {
            if (robot == null) {
                return;
            }

            saved = new PersistentState(
                    robot.driveTrain.getRobotPose(),
                    robot.spindexer.getArtifactDetections()
            );
        }

        public static boolean tryLoadRobotState(Robot robot) {
            if (saved == null || robot == null) {
                return false;
            }

            robot.driveTrain.setRobotPose(saved.robotPose);
            robot.spindexer.setArtifactDetections(saved.artifactDetections);

            saved = null;
            return true;
        }
    }

    public void savePersistentState() {
        PersistentState.saveRobotState(this);
    }

    public void tryLoadPersistentState() {
        if (!PersistentState.tryLoadRobotState(this)) {
            loadDefaultState();
        }
    }

    public void loadDefaultState() {
        driveTrain.setRobotPose(DEFAULT_ROBOT_POSE);
    }

    public enum RobotState {
        INTAKE,             // Robot is collecting artifacts on the field
        REVERSE_INTAKE,     // Robot is ejecting artifacts
        PRE_SHOOT,          // Robot is preparing to shoot, but cannot actually launch artifacts
        MANUAL_PRE_SHOOT,   // Robot is preparing to shoot at a hardcoded speed (used for tuning)
        SHOOT,              // Robot is shooting artifacts into the goal
        MANUAL_SHOOT,       // Robot is shooting using hardcoded speeds (used for tuning)
        NONE,               // No special function; robot is just moving
        PARK,               // Robot has parked (match is about to end)
    }

    private double fallbackRPM = 2900;
    private double fallbackHoodPosition = Shooter.HOOD_SERVO_MIN_POSITION;

    private double moveScale = 1;
    private double headingScale = 1;
    private boolean shotReady = false;
    private int shotsTaken = 0;

    /* Modules */
    private final FieldCentricDriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Turret turret;
    private final RobotLift lift;

    private AllianceColor allianceColor;
    private RobotState currentState;

    private final ElapsedTime stateStopwatch;
    private final ElapsedTime timeSinceShotReady;
    private final ElapsedTime shotPrepTime;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor color) {
        this(hardwareMap, telemetry, color, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor color, boolean preloadedArtifacts) {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap, telemetry, preloadedArtifacts);
        turret = new Turret(hardwareMap, telemetry);
        lift = new RobotLift(hardwareMap, telemetry);

        setAllianceColor(color);

        stateStopwatch = new ElapsedTime();
        timeSinceShotReady = new ElapsedTime();
        shotPrepTime = new ElapsedTime();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        setState(RobotState.NONE);

        Log.i(TAG, "Robot initialized");
    }

    public void setAllianceColor(AllianceColor color) {
        allianceColor = color;
    }

    public void swapAllianceColor() {
        switch (allianceColor) {
            case RED:
                setAllianceColor(AllianceColor.BLUE);
                break;

            case BLUE:
                setAllianceColor(AllianceColor.RED);
                break;
        }
    }

    public void logInfo() {
        spindexer.logInfo();
        shooter.logInfo();
        turret.logInfo();
        lift.logInfo();
        driveTrain.logInfo();
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public int getShotsTaken() {
        if (currentState != RobotState.SHOOT) {
            return 0;
        }
        return shotsTaken;
    }

    private void prepareToShoot(double goalOffsetX, double goalOffsetY) {
        if (currentState == RobotState.MANUAL_SHOOT || currentState == RobotState.MANUAL_PRE_SHOOT) {
            shooter.setRPM(fallbackRPM);
            shooter.setHoodPosition(fallbackHoodPosition);
        }
        else {
            shooter.setRPMForGoal(Math.sqrt(goalOffsetX * goalOffsetX + goalOffsetY * goalOffsetY));
            shooter.adjustHood();
        }
    }


    public void setState(RobotState newState) {
        if (newState == currentState) {
            return;
        }

        if (newState != RobotState.PARK) {
            lift.lowerRobot();
        }

        Log.v(TAG, "State " + currentState + " lasted for " + stateStopwatch.seconds() + " seconds");
        Log.i(TAG, "Switched to new state: " + newState);
        stateStopwatch.reset();

        switch (newState) {
            case INTAKE:
                spindexer.intakeIntoEmptySlot();
                shooter.disengageKicker();
                intake.startIntake();
                break;

            case REVERSE_INTAKE:
                spindexer.beginIntaking();
                shooter.disengageKicker();
                intake.reverseIntake();
                break;

            case MANUAL_PRE_SHOOT:
            case PRE_SHOOT:
                intake.stopIntake();
                shooter.disengageKicker();
                break;

            case MANUAL_SHOOT:
            case SHOOT:
                intake.stopIntake();
                shooter.engageKicker();
                spindexer.setArtifactDetections(new boolean[ArtifactLocation.values().length]); // zero artifact detections

                shotReady = false;
                shotPrepTime.reset();
                shotsTaken = 0;

                Log.d(TAG, "enter shoot {" +
                        (spindexer.hasArtifact(ArtifactLocation.SLOT_ONE) ? "front | " : "      | ") +
                        (spindexer.hasArtifact(ArtifactLocation.SLOT_TWO) ? "middle | " : "       | ") +
                        (spindexer.hasArtifact(ArtifactLocation.SLOT_THREE) ? "back" : "    ") +
                        "}");
                break;

            case NONE:
                spindexer.beginIntaking();
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                break;

            case PARK:
                spindexer.setPower(0);
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                lift.raiseRobot();
                break;
        }
        currentState = newState;
    }

    public RobotState getState() {
        return currentState;
    }

    public void loop(Gamepad gamepad1) {
        loop(-gamepad1.left_stick_y * moveScale, gamepad1.left_stick_x * moveScale, gamepad1.right_stick_x * headingScale);
    }

    public void loop(double drivePower, double strafePower, double turnPower) {
        if (currentState != RobotState.PARK) {
            setDrivePowers(drivePower, strafePower, turnPower);
        }
        loopWithoutMovement();
    }

    public void loopWithoutMovement() {
        if (currentState == RobotState.PARK) {
            return;
        }

        final Pose2D robotPose = driveTrain.getRobotPose();
        final double goalOffsetX = allianceColor.goalPositionX - robotPose.getX(DistanceUnit.INCH);
        final double goalOffsetY = allianceColor.goalPositionY - robotPose.getY(DistanceUnit.INCH);
        turret.aimAtGoal(
                goalOffsetX,
                goalOffsetY,
                robotPose.getHeading(AngleUnit.RADIANS),
                AngleUnit.RADIANS
        );

        spindexer.updateSpindexer();
        turret.update();

        switch (currentState) {
            case MANUAL_SHOOT:
            case SHOOT:
                prepareToShoot(goalOffsetX, goalOffsetY);

                if (!isShotReady()) {
                    if (shotReady) {
                        spindexer.setPower(0);
                        spindexer.rotateToNextSlot();

                        shotsTaken++;
                        Log.d(TAG, "Shot #" + shotsTaken + " completed in " + timeSinceShotReady.milliseconds() + " millis");
                    }
                    shotReady = false;
                    break;
                }

                if (!shotReady) {
                    spindexer.setPower(-1);

                    Log.d(TAG, "Ready to shoot after " + shotPrepTime.milliseconds() + " millis");
                    shotPrepTime.reset();
                    shotReady = true;
                    timeSinceShotReady.reset();
                }
                break;

            case MANUAL_PRE_SHOOT:
            case PRE_SHOOT:
                prepareToShoot(goalOffsetX, goalOffsetY);
                holdUpBall();
                break;

            case INTAKE:
                spindexer.intakeIntoEmptySlot();
                if (spindexer.hasAllArtifacts()) {
                    intake.idleWithBall();
                }
                else {
                    intake.startIntake();
                }
                break;

            case REVERSE_INTAKE:
            case PARK:
                break;

            case NONE:
                holdUpBall();
                break;
        }

        logInfo();
    }

    private void holdUpBall() {
        intake.idleWithBall();
    }

    public boolean isShotReady() {
        return shooter.isReady() &&
                turret.isReady();
    }

    /* Module-specific methods */

    public Pose2D getRobotPose() {
        return driveTrain.getRobotPose();
    }

    public void setRobotPose(Pose2D pose) {
        driveTrain.setRobotPose(pose);
    }

    public void setDrivePowers(double drive, double strafe, double turn) {
        driveTrain.setPower(drive, strafe, turn);
    }

    public void resetFieldCentricHeading() {
        driveTrain.resetFieldCentricHeading();
    }

    public void increaseFallbackShooterRPM() {
        fallbackRPM += 50;
    }

    public void decreaseFallbackShooterRPM() {
        fallbackRPM -= 50;
    }

    public double getFallbackShooterRPM() {
        return fallbackRPM;
    }

    public void setFallbackShooterRPM(double rpm) {
        fallbackRPM = rpm;
    }

    public double getFallbackHoodPosition() {
        return fallbackHoodPosition;
    }

    public void setFallbackHoodPosition(double position) {
        fallbackHoodPosition = position;
    }

    public void setMoveScale(double moveScale) {
        this.moveScale = moveScale;
    }

    public void setHeadingScale(double newHeadingScale) {
        headingScale = newHeadingScale;
    }

    public int getHeldArtifactCount() {
        return spindexer.getArtifactCount();
    }

    public void rotateSpindexerToNextSlot() {
        spindexer.rotateToNextSlot();
    }

    public void rotateSpindexerToPreviousSlot() {
        spindexer.rotateToPreviousSlot();
    }
}