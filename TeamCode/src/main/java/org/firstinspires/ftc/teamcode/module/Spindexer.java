package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class Spindexer {

    private static final String SPINDEXER_SERVO_ONE = "Spindexer Servo 1";
    private static final String SPINDEXER_SERVO_TWO = "Spindexer Servo 2";
    private static final String SPINDEXER_SERVO_THREE = "Spindexer Servo 3";
    private static final String SPINDEXER_SERVO_FOUR = "Spindexer Servo 4";

    private static final String SENS_ORANGE_ENCODER = "spindexer encoder";

    private static final String FRONT_COLOR_SENSOR = "Front Color Sensor";

    private final CRServo spindexerServoOne;
    private final CRServo spindexerServoTwo;
    private final CRServo spindexerServoThree;
    private final CRServo spindexerServoFour;
    private final RevColorSensorV3 frontColorSensor;

    private final AnalogInput spindexerEncoder;

    private final Telemetry telemetry;

    public static double ARTIFACT_DISTANCE_THRESHOLD_CM = 2;

    private static double spindexerAngle;

    public static double offsetAngle = 0;

    public static double spindexerPower = 0.3;


    //was thinking of using timers to switch to the next slot
    public static double TIME_TO_SWITCH_TO_NEXT_EMPTY_SLOT = 0.3;

    public static double TIME_TO_SWITCH_TO_NEXT_TWO_EMPTY_SLOTS = 0.6;

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        spindexerServoOne = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_ONE);
        spindexerServoTwo = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_TWO);
        spindexerServoThree = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_THREE);
        spindexerServoFour = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_FOUR);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, FRONT_COLOR_SENSOR);

        spindexerEncoder = hardwareMap.get(AnalogInput.class, SENS_ORANGE_ENCODER);

        spindexerAngle = AngleUnit.normalizeDegrees((spindexerEncoder.getVoltage() - 0.043) / 3.1 * 360 + offsetAngle);


        this.telemetry = telemetry;

        //set all spindexer servo directions for spindexer to turn counterclockwise initially
        spindexerServoOne.setDirection(CRServo.Direction.FORWARD);
        spindexerServoTwo.setDirection(CRServo.Direction.REVERSE);
        spindexerServoThree.setDirection(CRServo.Direction.REVERSE);
        spindexerServoFour.setDirection(CRServo.Direction.FORWARD);


    }

    private boolean hasBall(RevColorSensorV3 sensor) {
        double dist = sensor.getDistance(DistanceUnit.CM);
        return !Double.isNaN(dist) && dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;

    }

    //sets the spindexer power, absolute value is there not to confuse servo directions
    public void setSpindexerPower(double power) {
        spindexerServoOne.setPower(Math.abs(power));
        spindexerServoTwo.setPower(Math.abs(power));
        spindexerServoThree.setPower(Math.abs(power));
        spindexerServoFour.setPower(Math.abs(power));
    }

    public void runSpindexer() {
        spindexerServoOne.setPower(spindexerPower);
        spindexerServoTwo.setPower(spindexerPower);
        spindexerServoThree.setPower(spindexerPower);
        spindexerServoFour.setPower(spindexerPower);
    }

    //spindexer rotates counterclockwise
    public void rotateCounterClockWise() {
        spindexerServoOne.setDirection(CRServo.Direction.FORWARD);
        spindexerServoTwo.setDirection(CRServo.Direction.REVERSE);
        spindexerServoThree.setDirection(CRServo.Direction.REVERSE);
        spindexerServoFour.setDirection(CRServo.Direction.FORWARD);
    }

    //spindexer rotates clockwise
    public void rotateClockWise() {
        spindexerServoOne.setDirection(CRServo.Direction.REVERSE);
        spindexerServoTwo.setDirection(CRServo.Direction.FORWARD);
        spindexerServoThree.setDirection(CRServo.Direction.FORWARD);
        spindexerServoFour.setDirection(CRServo.Direction.REVERSE);
    }

    public void rotateToEmptySlot() {


    }

    public void zeroSpindexer() {

    }

    public void stopSpindexer() {
        spindexerServoOne.setPower(0);
        spindexerServoTwo.setPower(0);
        spindexerServoThree.setPower(0);
        spindexerServoFour.setPower(0);
    }


}
