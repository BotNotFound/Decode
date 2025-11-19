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
    public static double KICKER_LAUNCH_POSITION = 0.3;

    private final DcMotorEx lowerFlywheelMotor;
    private final DcMotorEx upperFlywheelMotor;
    private final Servo kickerServo;

    private final PIDFController velocityPID;
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.000175;
    public static double tolerance = 100;

    public static int motorCPS = 28;

    public double defaultRPM = 2900;

    private boolean stickyRPM = false;
    private double stickyTargetRPM;

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

        lowerFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upperFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        kickerServo.setPosition(KICKER_IDLE_POSITION);

        velocityPID = new PIDFController(kP, kI, kD, kF);
        velocityPID.setTolerance(tolerance);

        this.telemetry = telemetry;

        flywheelSpeeds = new InterpLUT();
        // the control points have to be in increasing order
        flywheelSpeeds.add(30.0, 2700); // extrapolated lower bound
        flywheelSpeeds.add(39.0, 2900);
        flywheelSpeeds.add(70.0, 3200);
        flywheelSpeeds.add(82.0, 3400);
        flywheelSpeeds.add(96.5, 3700);
        flywheelSpeeds.add(120.0, 3950); // extrapolated upper bound
        flywheelSpeeds.createLUT();
    }

    public void setRPM(double rpm) {
        if (rpm == 0) {
            // no need for velocity control if we aren't spinning
            lowerFlywheelMotor.setPower(0);
            upperFlywheelMotor.setPower(0);
            return;
        }

        velocityPID.setTolerance(tolerance);
        velocityPID.setPIDF(kP, kI, kD, kF);
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

            stickyRPM = false;
        }

        telemetry.addData("Target RPM", rpm);
        telemetry.addData("Real RPM", actualRPM);
        telemetry.addData("Default RPM", defaultRPM);

    }

    public void setRPMForAprilTag(AprilTagPoseFtc tagPose) {
        if(tagPose != null){
            // clamps the range to the min/max for the interpLUT to avoid bound errors
            double measuredDistance = tagPose.range;
            measuredDistance = Math.min(120.0, Math.max(30.0, measuredDistance));

            // sets stickyRPM
            stickyRPM = true;
            stickyTargetRPM = flywheelSpeeds.get(measuredDistance);

            setRPM(stickyTargetRPM);
        } else if (stickyRPM) {
            setRPM(stickyTargetRPM);
        } else {
            setRPM(defaultRPM);
        }
    }

    public void increaseDefaultRPM() {
        defaultRPM += 50;
    }

    public void decreaseDefaultRPM() {
        defaultRPM -= 50;
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
