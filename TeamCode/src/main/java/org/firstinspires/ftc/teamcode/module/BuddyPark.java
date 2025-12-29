package org.firstinspires.ftc.teamcode.module;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class BuddyPark {

    public static final String BUDDY_PARK_SERVO_NAME = "Kicker";

    private final Servo buddyParkServo;

    //TODO: Tune these values in dashboard
    public static double RAISED_ROBOT_POSITION = 0.2;

    public static double LOWERED_ROBOT_POSITION = 0.8;

    private boolean robotIsLifted;

    private final Telemetry telemetry;

    public BuddyPark(HardwareMap hwMap, Telemetry telemetry) {
        buddyParkServo = hwMap.get(Servo.class, BUDDY_PARK_SERVO_NAME);

        buddyParkServo.setPosition(LOWERED_ROBOT_POSITION);

        this.telemetry = telemetry;

        robotIsLifted = false;
    }

    private void raiseRobot() {
        buddyParkServo.setPosition(RAISED_ROBOT_POSITION);
        robotIsLifted = true;
        telemetry.addData("Robot is ", " lifted up.");
    }

    private void lowerRobot() {
        buddyParkServo.setPosition(LOWERED_ROBOT_POSITION);
        robotIsLifted = false;
        telemetry.addData("Robot is ", "on ground.");
    }


}