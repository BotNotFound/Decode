package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public abstract class AutonomousBase extends OpMode {
    private static final String TAG = "AutonomousBase";

    private final AllianceColor allianceColor;
    private final Pose startPose;
    private AutonomousStage[] stageSequence;
    private int currentStageIndex;

    protected Robot robot;
    protected Follower follower;

    public AutonomousBase(Pose startPose, AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        this.startPose = startPose;
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
            Log.i(TAG, "Completed stage " + currentStageIndex);
            currentStageIndex++;
            if (currentStageIndex >= stageSequence.length) {
                Log.i(TAG, "Completed autonomous sequence");
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
        currentStageIndex = 0;

        stageSequence = buildStageSequence();
        if (stageSequence == null) {
            stageSequence = new AutonomousStage[0];
        }

        Log.i(TAG, "Op mode initialized");
    }

    @Override
    public void start() {
        robot.start();
        if (stageSequence.length > 0) {
            stageSequence[0].enterStage(robot, follower);
        }
        follower.setStartingPose(startPose);
        Log.i(TAG, "Starting autonomous sequence");
    }

    /**
     * Creates the plan for the robot to execute during this autonomous
     *
     * @return The stage sequence to use
     * @implNote This method is called in init after {@link #robot} and {@link #follower}
     *         are initialized
     */
    protected abstract AutonomousStage[] buildStageSequence();

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
            robot.setState(Robot.RobotState.NONE);
            return;
        }

        telemetry.addData("Autonomous Stage", currentStageIndex);
    }

    @Override
    public void stop() {
        robot.savePersistentState();

        if (isAutoComplete()) {
            Log.i(TAG, "Op mode stopped after sequence was completed");
        }
        else {
            Log.i(TAG, "Op mode stopped at stage " + currentStageIndex);
        }
    }
}
