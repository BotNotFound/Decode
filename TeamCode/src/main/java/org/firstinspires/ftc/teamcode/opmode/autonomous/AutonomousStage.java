package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.concurrent.TimeUnit;

/**
 * Represents a step carried out by the robot as part of
 * an autonomous sequence
 */
@Config
public class AutonomousStage {
    /**
     * How long the robot typically takes to shoot every ball it can carry, in milliseconds
     */
    public static long MIN_SHOT_DURATION_MILLIS = 3000;
    public static long MAX_SHOT_DURATION_MILLIS = 4000;

    private final PathChain path;
    private final Robot.RobotState robotState;
    private final Timing.Timer minShotTimer;
    private final Timing.Timer maxShotTimer;

    private boolean complete;
    private int ballsHeldAtStart;

    /**
     * Creates an autonomous stage
     *
     * @param path       The path the robot will follow during the stage
     * @param robotState What state the robot should be in by the end of the stage
     */
    public AutonomousStage(PathChain path, Robot.RobotState robotState) {
        this.path = path;
        this.robotState = robotState;
        minShotTimer = new Timing.Timer(MIN_SHOT_DURATION_MILLIS, TimeUnit.MILLISECONDS);
        maxShotTimer = new Timing.Timer(MAX_SHOT_DURATION_MILLIS, TimeUnit.MILLISECONDS);
        complete = false;
    }

    /**
     * Gets this stage's ending pose
     *
     * @return The path the robot will follow during this stage
     */
    public PathChain getPath() {
        return path;
    }

    /**
     * Gets this stage's state for the robot
     *
     * @return What state the robot should be in while this stage is active
     */
    public Robot.RobotState getRobotState() {
        return robotState;
    }

    /**
     * Checks if this state has been completed, assuming that this stage has already started
     *
     * @param robot    The robot running this stage
     * @param follower The follower moving the robot
     * @return {@code true} if the stage is complete, {@code false} otherwise
     */
    private boolean checkForCompletion(Robot robot, Follower follower) {
        if (robotState != Robot.RobotState.SHOOT && follower.isBusy()) {
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
            case PRE_SHOOT:
                // this state is only a buffer for shoot, so if we are able to shoot,
                // we can end it
                return true;
            case SHOOT:
                if (minShotTimer.isTimerOn()) {
                    // we are currently shooting, wait until we're done
                    return maxShotTimer.done() || (minShotTimer.done() && robot.getShotsTaken() >= ballsHeldAtStart);
                }

                if (robot.isShotReady()) {
                    // we are ready to shoot, start the timer
                    minShotTimer.start();
                    maxShotTimer.start();
                }
                return false;
        }

        throw new IllegalStateException(); // should never be reached
    }

    /**
     * Checks if this state has been completed
     *
     * @param robot    The robot running this stage
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
     *
     * @param robot    The robot running this stage
     * @param follower The follower moving the robot
     */
    public void enterStage(Robot robot, Follower follower) {
        ballsHeldAtStart = robot.getHeldArtifactCount();
        robot.setState(robotState);

        follower.followPath(path);

        complete = false;
    }
}
