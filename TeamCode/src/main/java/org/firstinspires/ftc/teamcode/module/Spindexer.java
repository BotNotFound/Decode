package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
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

    private static final String FRONT_COLOR_SENSOR= "Front Color Sensor";

    private final CRServo spindexerServoOne;
    private final CRServo spindexerServoTwo;
    private final CRServo spindexerServoThree;
    private final CRServo spindexerServoFour;
    private final RevColorSensorV3 frontColorSensor;

    private final AnalogInput spindexerEncoder;

    private final Telemetry telemetry;

    //TODO: Tune the artifact distance threshold for the color sensor so it detects reliably
    public static double ARTIFACT_DISTANCE_THRESHOLD_CM = 2;

    private static double spindexerAngle;

    //TODO: Tune the below angles and power values
    public static double offsetAngle = 0;


    public static double spindexerPower = 0.3;

    public static double FIRST_SLOT_ANGLE = 0;

    public static double SECOND_SLOT_ANGLE = 120;

    public static double THIRD_SLOT_ANGLE = 240;

    //TODO: Tune the tolerance value
    public static double ANGLE_TOLERANCE_DEGREES = 1;

    //TODO: Tune the PIDF  values for the spindexer
    private final PIDFController spindexerController;

    public static double kP = 0.1;
    public static double kI = 0.01;
    public static double kD = 0;
    public static double kF = 0;
    public static double tolerance = 200;

    private static final int MAX_ARTIFACT_COUNT = 3;

    private int artifactCount = 0;



    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry){
        spindexerServoOne= hardwareMap.get(CRServo.class, SPINDEXER_SERVO_ONE);
        spindexerServoTwo = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_TWO);
        spindexerServoThree = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_THREE);
        spindexerServoFour = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_FOUR);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, FRONT_COLOR_SENSOR);

        spindexerEncoder = hardwareMap.get(AnalogInput.class, SENS_ORANGE_ENCODER);

        spindexerAngle = AngleUnit.normalizeDegrees((spindexerEncoder.getVoltage()-0.043)/3.1*360 + offsetAngle);

        spindexerController = new PIDFController(kP,kI, kD, kF);
        spindexerController.setTolerance(tolerance);







        this.telemetry = telemetry;

        //set all spindexer servo directions for spindexer to turn counterclockwise initially
        spindexerServoOne.setDirection(CRServo.Direction.FORWARD);
        spindexerServoTwo.setDirection(CRServo.Direction.REVERSE);
        spindexerServoThree.setDirection(CRServo.Direction.REVERSE);
        spindexerServoFour.setDirection(CRServo.Direction.FORWARD);


    }

    private boolean detectsBall(RevColorSensorV3 sensor) {
        double dist = sensor.getDistance(DistanceUnit.CM);
        return !Double.isNaN(dist) && dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;

    }

    //sets the spindexer power, absolute value is there not to confuse servo directions
    public void setSpindexerPower(double power){
        spindexerServoOne.setPower(Math.abs(power));
        spindexerServoTwo.setPower(Math.abs(power));
        spindexerServoThree.setPower(Math.abs(power));
        spindexerServoFour.setPower(Math.abs(power));
    }

    public void runSpindexer(){
        spindexerServoOne.setPower(spindexerPower);
        spindexerServoTwo.setPower(spindexerPower);
        spindexerServoThree.setPower(spindexerPower);
        spindexerServoFour.setPower(spindexerPower);
    }

    //spindexer rotates counterclockwise
    public void setRotationCounterClockWise(){
        spindexerServoOne.setDirection(CRServo.Direction.FORWARD);
        spindexerServoTwo.setDirection(CRServo.Direction.REVERSE);
        spindexerServoThree.setDirection(CRServo.Direction.REVERSE);
        spindexerServoFour.setDirection(CRServo.Direction.FORWARD);
    }

    //spindexer rotates clockwise
    public void setRotationClockWise(){
        spindexerServoOne.setDirection(CRServo.Direction.REVERSE);
        spindexerServoTwo.setDirection(CRServo.Direction.FORWARD);
        spindexerServoThree.setDirection(CRServo.Direction.FORWARD);
        spindexerServoFour.setDirection(CRServo.Direction.REVERSE);
    }

    public void rotateToAngle(double angle){
        spindexerController.setTolerance(tolerance);
        spindexerController.setPIDF(kP, kI, kD, kF);
        double power = spindexerController.calculate(getSpindexerAngle(), angle);
        double angleDifference = AngleUnit.normalizeDegrees(angle-getSpindexerAngle());
        if(angleDifference < 0){
            setRotationClockWise();
        }else if(angleDifference > 0){
            setRotationCounterClockWise();
        }
        setSpindexerPower(power);

        telemetry.addData("Current spindexer angle, ", getSpindexerAngle());

    }

    public void rotateByEmptySlots(){
        //for now we're just doing the case where we intake them all continuously and there are no issues but that does need to be changed
        if(getArtifactCount() == 0 && (AngleUnit.normalizeDegrees(getSpindexerAngle()-ArtifactLocation.SLOT_ONE.angle)< ANGLE_TOLERANCE_DEGREES) && detectsBall(frontColorSensor)) {
            artifactCount++;
            ArtifactLocation.SLOT_ONE.hasBall = true;
            rotateToAngle(ArtifactLocation.SLOT_TWO.angle);
        }
        else if(getArtifactCount() == 1 && (AngleUnit.normalizeDegrees(getSpindexerAngle()-ArtifactLocation.SLOT_TWO.angle)< ANGLE_TOLERANCE_DEGREES) && detectsBall(frontColorSensor)) {
            artifactCount++;
            ArtifactLocation.SLOT_TWO.hasBall = true;
            rotateToAngle(ArtifactLocation.SLOT_THREE.angle);
        }
        else if(getArtifactCount() == 2 && (AngleUnit.normalizeDegrees(getSpindexerAngle()-ArtifactLocation.SLOT_THREE.angle)< ANGLE_TOLERANCE_DEGREES) &&detectsBall(frontColorSensor)) {
            artifactCount++;
            ArtifactLocation.SLOT_THREE.hasBall = true;
        }
    }

    public void zeroSpindexer(){
        if(!(AngleUnit.normalizeDegrees(getSpindexerAngle()-ArtifactLocation.SLOT_ONE.angle)< ANGLE_TOLERANCE_DEGREES)){
            rotateToAngle(ArtifactLocation.SLOT_ONE.angle);
        }
    }



    public void stopSpindexer(){
        setSpindexerPower(0);
    }

    public int getArtifactCount(){
        return artifactCount;
    }

    public void resetArtifactCount(){
        artifactCount = 0;
    }

    public void printAngle(){
        spindexerAngle = AngleUnit.normalizeDegrees((spindexerEncoder.getVoltage()-0.043)/3.1*360 + offsetAngle);
        telemetry.addData("Spindexer Angle(degrees): ", spindexerAngle);

    }

    private double getSpindexerAngle(){
        return AngleUnit.normalizeDegrees((spindexerEncoder.getVoltage()-0.043)/3.1*360 + offsetAngle);
    }






}
