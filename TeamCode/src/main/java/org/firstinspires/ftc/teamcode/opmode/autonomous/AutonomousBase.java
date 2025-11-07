package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Config
public class AutonomousBase extends OpMode {
    /**
     * How long the robot typically takes to shoot every ball it can carry, in milliseconds
     */
    public static long SHOT_DURATION_MILLIS = 3000;

    /**
     * The pose tolerance used by {@link AutonomousStage} for the robot's x position, in inches
     */
    public static double POSE_TOLERANCE_X = 2;
    /**
     * The pose tolerance used by {@link AutonomousStage} for the robot's y position, in inches
     */
    public static double POSE_TOLERANCE_Y = 2;
    /**
     * The pose tolerance used by {@link AutonomousStage} for the robot's heading, in degrees
     */
    public static double POSE_TOLERANCE_HEADING = 2;

    /**
     * Represents a step carried out by the robot as part of
     * an autonomous sequence
     */
    public static class AutonomousStage {
        private final Pose begin;
        private final Pose end;
        private final Robot.RobotState robotState;
        private final Timing.Timer shotTimer;
        private boolean complete;

        /**
         * Creates an autonomous stage
         * @param start Where the robot should be at the beginning of the stage
         * @param end Where the robot should have moved to by the end of the stage
         * @param robotState What state the robot should be in by the end of the stage
         */
        public AutonomousStage(Pose start, Pose end, Robot.RobotState robotState) {
            this.begin = start;
            this.end = end;
            this.robotState = robotState;
            shotTimer = new Timing.Timer(SHOT_DURATION_MILLIS, TimeUnit.MILLISECONDS);
            complete = false;
        }

        /**
         * Gets this stage's starting pose
         * @return Where the robot should be before the stage starts
         */
        public Pose getStartPose() {
            return begin;
        }

        /**
         * Gets this stage's ending pose
         * @return Where the robot should have moved to by the time this stage is complete
         */
        public Pose getEndPose() {
            return end;
        }

        /**
         * Gets this stage's state for the robot
         * @return What state the robot should be in while this stage is active
         */
        public Robot.RobotState getRobotState() {
            return robotState;
        }

        /**
         * Checks if this state has been completed, assuming that this stage has already started
         * @param robot The robot running this stage
         * @param follower The follower moving the robot
         * @return {@code true} if the stage is complete, {@code false} otherwise
         */
        private boolean checkForCompletion(Robot robot, Follower follower) {
            if (!follower.atPose(end, POSE_TOLERANCE_X, POSE_TOLERANCE_Y, POSE_TOLERANCE_HEADING)) {
                // we aren't yet at our destination, so the state can't be complete
                return false;
            }

            switch (robotState) {
                case NONE:
                    // this state does nothing other than movement, we are complete
                    return true;
                case INTAKE:
                case REVERSE_INTAKE:
                    // these states are only useful when the robot is moving
                    return true;
                case SHOOT:
                    if (shotTimer.isTimerOn()) {
                        // we are currently shooting, wait until we're done
                        return shotTimer.done();
                    }

                    if (robot.isShotReady()) {
                        // we are ready to shoot, start the timer
                        shotTimer.start();
                    }
                    return false;
            }

            return true; // should never be reached
        }

        /**
         * Checks if this state has been completed
         * @param robot The robot running this stage
         * @param follower The follower moving the robot
         * @return {@code true} if the stage is complete, {@code false} otherwise
         */
        public boolean isStateComplete(Robot robot, Follower follower) {
            if (complete) {
                return true; // already done
            }

            complete = checkForCompletion(robot, follower);
            return complete;
        }

        /**
         * Starts this stage
         * @param robot The robot running this stage
         * @param follower The follower moving the robot
         */
        public void enterStage(Robot robot, Follower follower) {
            robot.setState(Robot.RobotState.NONE); // don't use excess power between stages
            follower.holdPoint(begin);
        }

        /**
         * Runs a cycle of the stage.  Does not loop the robot or follower
         * @param robot The robot running this stage
         * @param follower The follower moving the robot
         */
        public void loopStage(Robot robot, Follower follower) {
            if (follower.isBusy()) {
                return; // still moving
            }

            if (follower.atPose(begin, POSE_TOLERANCE_X, POSE_TOLERANCE_Y, POSE_TOLERANCE_HEADING)) {
                robot.setState(robotState);
                follower.holdPoint(end);
            }
        }
    }

    private final Robot.AllianceColor allianceColor;
    private final AutonomousStage[] stageSequence;
    private int currentStageIndex;

    private Robot robot;
    private Follower follower;

    public AutonomousBase(AutonomousStage[] stageSequence, Robot.AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        // add on a null stage as a buffer
        this.stageSequence = stageSequence;
        currentStageIndex = 0;
    }

    private boolean isAutoComplete() {
        return currentStageIndex >= stageSequence.length;
    }

    private AutonomousStage tryGetCurrentStage() {
        if (isAutoComplete()) {
            return null;
        }

        AutonomousStage currentStage = stageSequence[currentStageIndex];
        if (currentStage.isStateComplete(robot, follower)) {
            currentStageIndex++;
            if (currentStageIndex >= stageSequence.length) {
                return null; // we have just finished the auto
            }

            currentStage = stageSequence[currentStageIndex];
            currentStage.enterStage(robot, follower);
            return currentStage;
        }

        return currentStage;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry, allianceColor);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();
        robot.loopWithoutMovement(); // called last so auto aim turning isn't eaten by the follower

        if (isAutoComplete()) {
            // autonomous is complete; idle until time runs out
            return;
        }

        AutonomousStage currentStage = tryGetCurrentStage();
        if (currentStage == null) {
            return; // no more stages to run
        }

        currentStage.loopStage(robot, follower);
    }
}
