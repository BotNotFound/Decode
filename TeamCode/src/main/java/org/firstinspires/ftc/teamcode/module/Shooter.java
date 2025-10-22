package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
public class Shooter {
    public static final String LOWER_FLYWHEEL_MOTOR_NAME = "Lower flywheel";
    public static final String UPPER_FLYWHEEL_MOTOR_NAME = "Upper flywheel";
    public static final String KICKER_SERVO_NAME = "Kicker";

    public static double KICKER_IDLE_POSITION = 0.82;
    public static double KICKER_LAUNCH_POSITION = 0.35;

    private final DcMotorEx lowerFlywheelMotor;
    private final DcMotorEx upperFlywheelMotor;
    private final Servo kickerServo;

    private final PIDFController velocityPID;
    public static double kP = 0.001;
    public static double kD = 0.0002;
    public static double kF = 0.00026;

    public static int motorCPS = 28;

    public double defaultRPM = 3000;

    private final Telemetry telemetry;

    private final InterpLUT flywheelSpeeds;



    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        lowerFlywheelMotor = hardwareMap.get(DcMotorEx.class, LOWER_FLYWHEEL_MOTOR_NAME);
        upperFlywheelMotor = hardwareMap.get(DcMotorEx.class, UPPER_FLYWHEEL_MOTOR_NAME);
        kickerServo = hardwareMap.get(Servo.class, KICKER_SERVO_NAME);

        lowerFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        upperFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lowerFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kickerServo.setPosition(KICKER_IDLE_POSITION);

        velocityPID = new PIDFController(kP, 0, kD, kF);
        velocityPID.setTolerance(75);

        this.telemetry = telemetry;

        flywheelSpeeds = new InterpLUT();
        // the control points have to be in increasing order
        flywheelSpeeds.add(29.0, 2800);
        flywheelSpeeds.add(31.0, 3000);
        flywheelSpeeds.add(34.0, 3025);
        flywheelSpeeds.add(44.5, 3050);
        flywheelSpeeds.add(61.8, 3375);
        flywheelSpeeds.add(68.5, 3475);
        flywheelSpeeds.add(71.5, 3700);
        flywheelSpeeds.add(110,4050);
        flywheelSpeeds.add(130, 4300);
        flywheelSpeeds.createLUT();
    }

    public void setRPM(double rpm) {
        double actualRPM = upperFlywheelMotor.getVelocity() / motorCPS * 60;

        if(rpm > 0){
            double power = velocityPID.calculate(actualRPM, rpm);
            telemetry.addData("shooter power", power);

            lowerFlywheelMotor.setPower(power);
            upperFlywheelMotor.setPower(power);
        }
        else{
            lowerFlywheelMotor.setPower(0);
            upperFlywheelMotor.setPower(0);
        }

        telemetry.addData("Target RPM", rpm);
        telemetry.addData("Real RPM", actualRPM);
        telemetry.addData("Default RPM", defaultRPM);

    }

    public void setRPM(AprilTagPoseFtc tagPose) {
        if(tagPose != null){
            setRPM(flywheelSpeeds.get(tagPose.range));
        }
        else{
            setRPM(defaultRPM);
        }
    }

    public void editDefaultRPM(boolean increase) {
        if(increase){
            defaultRPM += 50;
        }
        else{
            defaultRPM -= 50;
        }
    }

    public void engageKicker() {
        kickerServo.setPosition(KICKER_LAUNCH_POSITION);

        telemetry.addData("Kicker Position", "engaged");
    }

    public void disengageKicker() {
        kickerServo.setPosition(KICKER_IDLE_POSITION);

        telemetry.addData("Kicker Position", "disengaged");
    }

    public boolean isReady() {
        return velocityPID.atSetPoint();
    }
}
