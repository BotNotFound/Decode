package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public abstract class AutonomousBase extends OpMode {
    public static double MAX_INTAKE_MOTOR_POWER = 0.5;

    private final Robot.AllianceColor allianceColor;
    private final Pose startPose;
    private AutonomousStage[] stageSequence;
    private int currentStageIndex;

    protected Robot robot;
    protected Follower follower;

    public AutonomousBase(Pose startPose, Robot.AllianceColor allianceColor) {
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

        stageSequence = buildStageSequence();
        if (stageSequence == null) {
            stageSequence = new AutonomousStage[0];
        }
    }

    @Override
    public void start() {
        if (stageSequence.length > 0) {
            stageSequence[0].enterStage(robot, follower);
        }
    }

    /**
     * Creates the plan for the robot to execute during this autonomous
     * @return The stage sequence to use
     * @implNote This method is called in init after {@link #robot} and {@link #follower} are initialized
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

        if (robot.getState() == Robot.RobotState.INTAKE) {
            // limit motor power on intake
            final double curDrivePower = robot.getDrivePower();
            final double curStrafePower = robot.getStrafePower();
            final double curTurnPower = robot.getTurnPower();

            final double newDrivePower = Math.copySign(Math.min(Math.abs(curDrivePower), MAX_INTAKE_MOTOR_POWER), curDrivePower);
            final double newStrafePower = Math.copySign(Math.min(Math.abs(curStrafePower), MAX_INTAKE_MOTOR_POWER), curStrafePower);
            final double newTurnPower = Math.copySign(Math.min(Math.abs(curTurnPower), MAX_INTAKE_MOTOR_POWER), curTurnPower);

            robot.setDrivePowers(newDrivePower, newStrafePower, newTurnPower);
        }

        telemetry.addData("Autonomous Stage", currentStageIndex);
    }

}
