package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class RobotLift {
    public static final String LIFT_SERVO_NAME = "Kickstand";

    private final Servo liftServo;

    public static double RAISED_ROBOT_POSITION = 0.292;

    public static double LOWERED_ROBOT_POSITION = 1;

    private final Telemetry telemetry;

    private boolean lifted;

    public RobotLift(HardwareMap hwMap, Telemetry telemetry) {
        liftServo = hwMap.get(Servo.class, LIFT_SERVO_NAME);

        this.telemetry = telemetry;

        lifted = false;
    }

    public void raiseRobot() {
        liftServo.setPosition(RAISED_ROBOT_POSITION);
        lifted = true;
    }

    public void lowerRobot() {
        liftServo.setPosition(LOWERED_ROBOT_POSITION);
        lifted = false;
    }

    public void logInfo() {
        telemetry.addData("Lift position", liftServo.getPosition());
        if (lifted) {
            telemetry.addLine("Robot is raised");
        }
        else {
            telemetry.addLine("Robot is lowered");
        }
    }
}