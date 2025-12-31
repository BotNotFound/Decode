package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquIDController;


@Config
public class Spindexer {

    private static final String SPINDEXER_SERVO_ONE = "Spindexer 1";
    private static final String SPINDEXER_SERVO_TWO = "Spindexer 2";
    private static final String SPINDEXER_SERVO_THREE = "Spindexer 3";
    private static final String SPINDEXER_SERVO_FOUR = "Spindexer 4";

    private static final String SENS_ORANGE_ENCODER = "Spindexer Encoder";

    private static final String FRONT_COLOR_SENSOR = "Front Color Sensor";

    private final CRServo spindexerServoOne;
    private final CRServo spindexerServoTwo;
    private final CRServo spindexerServoThree;
    private final CRServo spindexerServoFour;
    private final RevColorSensorV3 frontColorSensor;

    private final AnalogInput spindexerEncoder;

    private final Telemetry telemetry;

    private final boolean[] ballDetections;
    private ArtifactLocation curFrontLocation;

    //TODO: Tune the artifact distance threshold for the color sensor so it detects reliably
    public static double ARTIFACT_DISTANCE_THRESHOLD_CM = 2;

    //TODO: Tune the below angles and power values
    public static double offsetAngle = 0;

    // TODO tune PIDF values + tolerance
    private final SquIDController spindexerController;

    public static double kP = 0.005;
    public static double tolerance = 1;

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        ballDetections = new boolean[ArtifactLocation.values().length];
        curFrontLocation = null;

        spindexerServoOne = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_ONE);
        spindexerServoTwo = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_TWO);
        spindexerServoThree = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_THREE);
        spindexerServoFour = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_FOUR);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, FRONT_COLOR_SENSOR);

        spindexerEncoder = hardwareMap.get(AnalogInput.class, SENS_ORANGE_ENCODER);

        spindexerController = new SquIDController(kP);
        spindexerController.setTolerance(tolerance);

        // set all servo directions for spindexer to turn counterclockwise
        spindexerServoOne.setDirection(CRServo.Direction.FORWARD);
        spindexerServoTwo.setDirection(CRServo.Direction.REVERSE);
        spindexerServoThree.setDirection(CRServo.Direction.REVERSE);
        spindexerServoFour.setDirection(CRServo.Direction.FORWARD);
    }

    private void updateDetectionFromSensor() {
        if (!spindexerController.atTarget() || curFrontLocation == null) {
            return; // not at an artifact location; color sensor won't give the right data
        }

        double dist = frontColorSensor.getDistance(DistanceUnit.CM);
        ballDetections[curFrontLocation.index] = dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;
    }

    public boolean hasArtifact(ArtifactLocation location) {
        updateDetectionFromSensor();
        return ballDetections[location.index];
    }

    public boolean hasAllArtifacts() {
        updateDetectionFromSensor();
        for (boolean ballDetection : ballDetections) {
            if (!ballDetection) {
                return false;
            }
        }
        return true;
    }

    public int getArtifactCount() {
        updateDetectionFromSensor();
        int artifacts = 0;
        for (boolean detection : ballDetections) {
            if (detection) {
                artifacts++;
            }
        }
        return artifacts;
    }

    public void setSpindexerPower(double power) {
        spindexerServoOne.setPower(power);
        spindexerServoTwo.setPower(power);
        spindexerServoThree.setPower(power);
        spindexerServoFour.setPower(power);
    }

    /**
     * Given 3 values, returns the one with the lowest magnitude.
     *
     * @return The value with the lowest magnitude
     */
    private static double signedMin(double a, double b, double c) {
        if (Math.abs(a) <= Math.abs(b) && Math.abs(a) <= Math.abs(c)) {
            return a;
        }
        if (Math.abs(b) <= Math.abs(a) && Math.abs(b) <= Math.abs(c)) {
            return b;
        }
        return c;
    }

    /**
     * Calculates the shortest displacement of {@code angle1} relative to {@code angle2}. This is
     * similar to a simple {@code angle1 - angle2}, with the exception that the calculated
     * displacement will not be unnecessarily large when the angles are across the normalization
     * boundary (e.g. {@code angle1 = 170} and {@code angle2 = -175})
     *
     * @param angle1 The angle to get the displacement of
     * @param angle2 The 'source' angle -- displacement is calculated relative to this value
     * @param unit   The unit of the given angles
     * @return The shortest displacement of {@code angle1} relative to {@code angle2}
     */
    private static double getShortestDisplacement(double angle1, double angle2, AngleUnit unit) {
        final double ROTATION;
        switch (unit) {
            case DEGREES:
                ROTATION = 360;
                break;

            case RADIANS:
            default:
                ROTATION = 2.0 * Math.PI;
                break;
        }

        angle1 = unit.normalize(angle1);
        angle2 = unit.normalize(angle2);

        return signedMin(
                angle1 - angle2,
                angle1 + ROTATION - angle2,
                angle1 - angle2 - ROTATION
        );
    }

    /**
     * Calculates the closest angle to {@code closeToAngle} that is equivalent to {@code angle}
     *
     * @param angle        The angle to transform
     * @param closeToAngle The target angle -- the calculated angle will be as close as possible to
     *                     this
     * @param unit         The unit both angles are in
     * @return The closest angle to {@code closeToAngle} that is equivalent to {@code angle}
     */
    private static double closestEquivalentAngle(double angle, double closeToAngle, AngleUnit unit) {
        return closeToAngle + getShortestDisplacement(angle, closeToAngle, unit);
    }

    public void rotateToAngle(double angle) {
        curFrontLocation = null;
        spindexerController.setTarget(angle);
        updateSpindexer();
    }

    public void updateSpindexer() {
        spindexerController.setP(kP);
        spindexerController.setTolerance(tolerance);

        final double curEquivAngle = closestEquivalentAngle(getAngle(), spindexerController.getTarget(), AngleUnit.DEGREES);
        setSpindexerPower(spindexerController.calculate(curEquivAngle));
        updateDetectionFromSensor();
    }

    public void rotateLocationToFront(ArtifactLocation location) {
        rotateToAngle(location.angle);
        curFrontLocation = location;
    }

    public boolean atTargetRotation() {
        return spindexerController.atTarget();
    }

    public void rotateToNextSlot() {
        if (curFrontLocation == null) {
            return;
        }

        rotateLocationToFront(curFrontLocation.getNextLocation());
    }

    private double getAngle() {
        // see https://docs.sensorangerobotics.com/encoder/#analog-usage
        return AngleUnit.normalizeDegrees((spindexerEncoder.getVoltage() - 0.043) / 3.1 * 360 + offsetAngle);
    }

    public void logInfo() {
        telemetry.addData("Spindexer Angle (degrees)", getAngle());
        telemetry.addData("Spindexer Target Angle (degrees)", spindexerController.getTarget());
        telemetry.addData("Spindexer Close Angle (degrees)", closestEquivalentAngle(getAngle(), spindexerController.getTarget(), AngleUnit.DEGREES));
    }
}
