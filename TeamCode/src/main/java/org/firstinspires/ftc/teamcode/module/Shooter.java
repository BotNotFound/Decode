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

    //TODO: tune these kicker position values using dashboard
    public static double KICKER_IDLE_POSITION = 0.4;
    public static double KICKER_LAUNCH_POSITION = 0;


    //TODO: tune these values using dashboard and for clamping servo positions
    public static double HOOD_SERVO_POSITION_LOWER_BOUND = 0.41;

    public static double HOOD_SERVO_POSITION_UPPER_BOUND = 0.67;


    private final DcMotorEx leftFlywheelMotor;
    private final DcMotorEx rightFlywheelMotor;
    private final Servo kickerServo;
    private final Servo hoodServo;

    //TODO: tune velocity PID
    private final PIDFController velocityPID;
    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.000175;
    public static double tolerance = 200;

    public static int motorCPS = 28;

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

        leftFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        kickerServo.setPosition(KICKER_IDLE_POSITION);
        hoodServo.setPosition(HOOD_SERVO_POSITION_LOWER_BOUND);

        velocityPID = new PIDFController(kP, kI, kD, kF);
        velocityPID.setTolerance(tolerance);

        this.telemetry = telemetry;

        //TODO: tune table (probably)
        flywheelSpeeds = new InterpLUT();
        // the control points have to be in increasing order
        flywheelSpeeds.add(28.0, 2500); // extrapolated lower bound
        flywheelSpeeds.add(31.3, 2600);
        flywheelSpeeds.add(36.1, 2675);
        flywheelSpeeds.add(47.0, 2825);
        flywheelSpeeds.add(55, 2900);
        flywheelSpeeds.add(62.3, 3050);
        flywheelSpeeds.add(74.5, 3125);
        flywheelSpeeds.add(84, 3400);
        flywheelSpeeds.add(93.4, 3525);
        flywheelSpeeds.add(101.3, 3675);
        flywheelSpeeds.add(121.5, 3950);
        flywheelSpeeds.add(130, 4050); // extrapolated upper bound
        flywheelSpeeds.createLUT();

        //TODO: tune hood interplut
        hoodPositions = new InterpLUT();
        /*idea behind hoodPositions interplut is so that we adjust the hood so we always hit the back of the goal at a low height.
        We map the rpm(x) to a hood position(y)
         */
        hoodPositions.add(41, 0.41);
        hoodPositions.add(61, 0.61);
        hoodPositions.add(67, 0.67);
        hoodPositions.add(69, 0.69);
        hoodPositions.createLUT();


        timeSinceKickerEngaged = new ElapsedTime();
        kickerEngaged = false;

        // for FTC dashboard
        logRPM(0, 0);
    }

    public double getRPM() {
        return rightFlywheelMotor.getVelocity() / motorCPS * 60;
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
            double power = velocityPID.calculate(actualRPM, rpm);

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
        distanceToGoal = Math.min(120.0, Math.max(30.0, distanceToGoal));

        //set stickyRPM
        stickyRPM = true;
        stickyTargetRPM = flywheelSpeeds.get(distanceToGoal);
        setRPM(stickyTargetRPM);
    }

    public void setHoodPosition(double position) {
        if (position < HOOD_SERVO_POSITION_LOWER_BOUND) {
            hoodServo.setPosition(HOOD_SERVO_POSITION_LOWER_BOUND);
        }
        hoodServo.setPosition(Math.min(position, HOOD_SERVO_POSITION_UPPER_BOUND));
    }

    public void adjustHood() {
        setHoodPosition(hoodPositions.get(getRPM()));
    }

    public void setRPMForAprilTag(AprilTagPoseFtc tagPose, double fallbackRPM) {
        if (tagPose != null) {
            // clamps the range to the min/max for the interpLUT to avoid bound errors
            double measuredDistance = tagPose.range;
            measuredDistance = Math.min(120.0, Math.max(30.0, measuredDistance));

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
