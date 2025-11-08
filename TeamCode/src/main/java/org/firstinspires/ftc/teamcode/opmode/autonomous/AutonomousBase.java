package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class AutonomousBase extends OpMode {
    private final Robot.AllianceColor allianceColor;
    private final Pose startPose;
    private final AutonomousStage[] stageSequence;
    private int currentStageIndex;

    private Robot robot;
    private Follower follower;

    public AutonomousBase(Pose startPose, AutonomousStage[] stageSequence, Robot.AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        this.startPose = startPose;
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
        follower.setStartingPose(startPose);
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
            robot.setState(Robot.RobotState.NONE);
            return; // no more stages to run
        }

        currentStage.loopStage(robot, follower);
    }

}
