package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
public class Shooter {
    public static final String LEFT_FLYWHEEL_MOTOR_NAME = "Left flywheel";
    public static final String RIGHT_FLYWHEEL_MOTOR_NAME = "Right flywheel";
    public static final String KICKER_SERVO_NAME = "Kicker";

    public static final String HOOD_SERVO_NAME = "Hood";

    public static double KICKER_IDLE_POSITION = 0.3;
    public static double KICKER_LAUNCH_POSITION = 0.15;

    public static double FLYWHEEL_RPM_MULTIPLIER = 1;


    public static double HOOD_SERVO_MIN_POSITION = 0.15;

    public static double HOOD_SERVO_MAX_POSITION = 0.65;


    private final DcMotorEx leftFlywheelMotor;
    private final DcMotorEx rightFlywheelMotor;
    private final Servo kickerServo;
    private final Servo hoodServo;

    private final PIDFController velocityPID;
    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.000175;
    public static double tolerance = 100;

    /*
     * We currently use GoBilda 5203 6000 RPM motors to power our flywheel
     * (see https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder)
     */
    public static final double FLYWHEEL_ENCODER_RESOLUTION = 28;

    private boolean stickyRPM = false;
    private double stickyTargetRPM;

    private boolean kickerEngaged;

    private final Telemetry telemetry;

    private final InterpLUT flywheelSpeeds;

    private final InterpLUT hoodPositions;

    private final ElapsedTime timeSinceKickerEngaged;


    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFlywheelMotor = hardwareMap.get(DcMotorEx.class, LEFT_FLYWHEEL_MOTOR_NAME);
        rightFlywheelMotor = hardwareMap.get(DcMotorEx.class, RIGHT_FLYWHEEL_MOTOR_NAME);
        kickerServo = hardwareMap.get(Servo.class, KICKER_SERVO_NAME);
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        leftFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        kickerServo.setPosition(KICKER_IDLE_POSITION);
        hoodServo.setPosition(HOOD_SERVO_MIN_POSITION);

        velocityPID = new PIDFController(kP, kI, kD, kF);
        velocityPID.setTolerance(tolerance);

        this.telemetry = telemetry;

        flywheelSpeeds = new InterpLUT();
        // the control points have to be in increasing order
        flywheelSpeeds.add(0, 2000); // extrapolated lower bound
        flywheelSpeeds.add(46.79854459800701, 2425);
        flywheelSpeeds.add(68.78117471395568, 2500);
        flywheelSpeeds.add(86, 2750);

        flywheelSpeeds.add(103.68588631116245, 2975);
        flywheelSpeeds.add(106.47453701719283, 3100);
        flywheelSpeeds.add(139.90391077047641, 3700);
        flywheelSpeeds.add(152.20077845265857, 3900);
        flywheelSpeeds.add(160, 4030); // extrapolated upper bound
        flywheelSpeeds.add(203.646752982, 4589.09856718); // extrapolation
        flywheelSpeeds.createLUT();

        hoodPositions = new InterpLUT();
        /*idea behind hoodPositions interplut is so that we adjust the hood so we always hit the back of the goal at a low height.
        We map the rpm(x) to a hood position(y)
         */
        hoodPositions.add(-6000, 0.1);
        hoodPositions.add(0, HOOD_SERVO_MIN_POSITION);
        hoodPositions.add(2425, 0.151);
        hoodPositions.add(2500, 0.25);
        hoodPositions.add(2650, 0.3);
        hoodPositions.add(2975, 0.35);
        hoodPositions.add(3100, 0.4);
        hoodPositions.add(3700, 0.55);
        hoodPositions.add(3900, 0.575);
        hoodPositions.add(6000, HOOD_SERVO_MAX_POSITION); // extended max position
        hoodPositions.createLUT();


        timeSinceKickerEngaged = new ElapsedTime();
        kickerEngaged = false;

        // for FTC dashboard
        logRPM(0, 0);
    }

    public double getRPM() {
        return (leftFlywheelMotor.getVelocity() * 60.0) / FLYWHEEL_ENCODER_RESOLUTION;
    }

    public void setRPM(double rpm) {
        if (rpm == 0) {
            // no need for velocity control if we aren't spinning
            leftFlywheelMotor.setPower(0);
            rightFlywheelMotor.setPower(0);
            return;
        }

        velocityPID.setTolerance(tolerance);
        velocityPID.setPIDF(kP, kI, kD, kF);
        double actualRPM = getRPM();

        if (rpm > 0) {
            double power = velocityPID.calculate(actualRPM, rpm * FLYWHEEL_RPM_MULTIPLIER);

            leftFlywheelMotor.setPower(power);
            rightFlywheelMotor.setPower(power);
        }
        else {
            leftFlywheelMotor.setPower(0);
            rightFlywheelMotor.setPower(0);
        }
    }

    public void setRPMForGoal(double distanceToGoal) {
        // clamps the range to the min/max for the interpLUT to avoid bound errors
        distanceToGoal = Math.min(160.0, Math.max(30.0, distanceToGoal));

        //set stickyRPM
        stickyRPM = true;
        stickyTargetRPM = flywheelSpeeds.get(distanceToGoal);
        setRPM(stickyTargetRPM);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(Math.min(
            Math.max(position, HOOD_SERVO_MIN_POSITION),
            HOOD_SERVO_MAX_POSITION
        ));
    }

    public void adjustHood() {
        setHoodPosition(hoodPositions.get(stickyTargetRPM));
    }

    public void setRPMForAprilTag(AprilTagPoseFtc tagPose, double fallbackRPM) {
        if (tagPose != null) {
            // clamps the range to the min/max for the interpLUT to avoid bound errors
            double measuredDistance = tagPose.range;
            measuredDistance = Math.min(160.0, Math.max(30.0, measuredDistance));

            // sets stickyRPM
            stickyRPM = true;
            stickyTargetRPM = flywheelSpeeds.get(measuredDistance);

            setRPM(stickyTargetRPM);
        }
        else if (stickyRPM) {
            setRPM(stickyTargetRPM);
        }
        else {
            setRPM(fallbackRPM);
        }
    }

    public void engageKicker() {
        kickerServo.setPosition(KICKER_LAUNCH_POSITION);
        if (!kickerEngaged) {
            timeSinceKickerEngaged.reset();
        }
        kickerEngaged = true;
    }

    public void disengageKicker() {
        kickerServo.setPosition(KICKER_IDLE_POSITION);
        kickerEngaged = false;
    }

    public boolean isReady() {
        return velocityPID.atSetPoint();
    }

    private void logRPM(double target, double real) {
        telemetry.addData("Target RPM", target);
        telemetry.addData("Real RPM", real);
        telemetry.addData("Upper RPM Bound", target + tolerance);
        telemetry.addData("Lower RPM Bound", target - tolerance);
    }

    public void logInfo() {
        logRPM(velocityPID.getSetPoint(), getRPM());

        telemetry.addData("Flywheel power", leftFlywheelMotor.getPower());

        if (kickerEngaged) {
            telemetry.addLine("Kicker engaged");
        }
        else {
            telemetry.addLine("Kicker disengaged");
        }

    }
}
