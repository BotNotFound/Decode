package org.firstinspires.ftc.teamcode.module;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RobotLift {
    public static final String LIFT_SERVO_NAME = "Kickstand";

    private final Servo liftServo;

    //TODO: Tune these values in dashboard
    public static double RAISED_ROBOT_POSITION = 0.2;

    public static double LOWERED_ROBOT_POSITION = 0.8;

    private boolean robotIsLifted;

    private final Telemetry telemetry;

    public RobotLift(HardwareMap hwMap, Telemetry telemetry) {
        liftServo = hwMap.get(Servo.class, LIFT_SERVO_NAME);

        liftServo.setPosition(LOWERED_ROBOT_POSITION);

        this.telemetry = telemetry;

        robotIsLifted = false;
    }

    public void raiseRobot() {
        liftServo.setPosition(RAISED_ROBOT_POSITION);
        robotIsLifted = true;
        telemetry.addData("Robot is ", " lifted up.");
    }

    public void lowerRobot() {
        liftServo.setPosition(LOWERED_ROBOT_POSITION);
        robotIsLifted = false;
        telemetry.addData("Robot is ", "on ground.");
    }


}