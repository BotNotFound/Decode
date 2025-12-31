package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Robot {
    public static final String TAG = "Robot";

    private static final class PersistentState {
        private static PersistentState saved = null;

        private final Pose2D robotPose;

        private PersistentState(Pose2D robotPose) {
            this.robotPose = robotPose;
        }

        public static void saveRobotState(Robot robot) {
            if (robot == null) {
                return;
            }

            saved = new PersistentState(robot.driveTrain.getRobotPose());
        }

        public static void tryLoadRobotState(Robot robot) {
            if (saved == null || robot == null) {
                return;
            }

            robot.driveTrain.setRobotPose(saved.robotPose);

            saved = null;
        }
    }

    public void savePersistentState() {
        PersistentState.saveRobotState(this);
    }

    public void tryLoadPersistentState() {
        PersistentState.tryLoadRobotState(this);
    }

    public enum RobotState {
        INTAKE,
        REVERSE_INTAKE,
        PRE_SHOOT,
        SHOOT,
        NONE,
        PARK,
    }

    public static double fallbackRPM = 2900;

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
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap, telemetry);
        turret = new Turret(hardwareMap);
        lift = new RobotLift(hardwareMap, telemetry);

        setAllianceColor(color);

        currentState = RobotState.NONE;

        stateStopwatch = new ElapsedTime();
        timeSinceShotReady = new ElapsedTime();
        shotPrepTime = new ElapsedTime();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Log.i(TAG, "Robot initialized");
    }

    public void start() {
        tryLoadPersistentState();
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

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public int getShotsTaken() {
        if (currentState != RobotState.SHOOT) {
            return 0;
        }
        return shotsTaken;
    }

    private void prepareToShoot() {
        final Pose2D robotPose = driveTrain.getRobotPose();
        turret.aimAtGoal(
                robotPose.getX(DistanceUnit.INCH) - allianceColor.goalPositionX,
                robotPose.getY(DistanceUnit.INCH) - allianceColor.goalPositionY,
                robotPose.getHeading(AngleUnit.RADIANS)
        );
        shooter.setRPMForGoal(
                Math.sqrt(
                        Math.pow(robotPose.getX(DistanceUnit.INCH) - allianceColor.goalPositionX, 2) +
                                Math.pow(robotPose.getY(DistanceUnit.INCH) - allianceColor.goalPositionY, 2)
                )
        );
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
                shooter.disengageKicker();
                intake.startIntake();
                break;

            case REVERSE_INTAKE:
                shooter.disengageKicker();
                intake.setPower(-1);

                break;

            case PRE_SHOOT:
                intake.startIntake();
                shooter.disengageKicker();
                break;

            case SHOOT:
                intake.stopIntake();
                shooter.disengageKicker();

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
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                break;

            case PARK:
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
        spindexer.updateSpindexer();

        switch (currentState) {
            case SHOOT:
                prepareToShoot();

                if (!isShotReady()) {
                    intake.stopIntake();

                    if (shotReady) {
                        shotsTaken++;
                        Log.d(TAG, "Shot #" + shotsTaken + " completed in " + timeSinceShotReady.milliseconds() + " millis");
                    }
                    shotReady = false;

                    break;
                }

                if (!shotReady) {
                    Log.d(TAG, "Ready to shoot after " + shotPrepTime.milliseconds() + " millis");
                    shotPrepTime.reset();
                    shotReady = true;
                    timeSinceShotReady.reset();
                }

                shooter.engageKicker();
                break;

            case PRE_SHOOT:
                prepareToShoot();
                break;

            case INTAKE:
                if (spindexer.hasAllArtifacts()) {
                    intake.stopIntake();
                }
                else {
                    intake.startIntake();
                }
                break;

            case REVERSE_INTAKE:
            case NONE:
                break;
        }

        spindexer.logInfo();
    }

    public boolean isShotReady() {
        // TODO check if a ball is loaded
        return currentState == RobotState.SHOOT && shooter.isReady() && turret.isReady() && spindexer.atTargetRotation();
    }

    /* Module-specific methods */

    public void setDrivePowers(double drive, double strafe, double turn) {
        driveTrain.setPower(drive, strafe, turn);
    }

    public void resetOdometry() {
        driveTrain.resetOdometry();
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

    public double getDrivePower() {
        return driveTrain.getDrivePower();
    }

    public double getStrafePower() {
        return driveTrain.getStrafePower();
    }

    public double getTurnPower() {
        return driveTrain.getTurnPower();
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
}