package org.firstinspires.ftc.teamcode.module;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*If we ever consider mild sorting efforts and tracking whether or not we have three artifacts inside
 * we'll use this class
 */
@Config
public class ArtifactTracker {
    public static final String BACK_COLOR_SENSOR_NAME = "ColorSensorBack";
    public static final String MIDDLE_COLOR_SENSOR_NAME = "ColorSensorMiddle";
    public static final String FRONT_COLOR_SENSOR_NAME = "ColorSensorFront";
    public static final String INDICATOR_LIGHT_NAME = "LED Light";
    public static final int MAX_ARTIFACT_COUNT = 3;

    public enum ArtifactLocation {
        FRONT(0, FRONT_COLOR_SENSOR_NAME),
        MIDDLE(1, MIDDLE_COLOR_SENSOR_NAME),
        BACK(2, BACK_COLOR_SENSOR_NAME);

        public final int index;
        public final String hardwareName;

        ArtifactLocation(int index, String hardwareName) {
            this.index = index;
            this.hardwareName = hardwareName;
        }
    }

    private final RevColorSensorV3[] colorSensors;

    private final Servo indicatorLight;


    public static double ARTIFACT_DISTANCE_THRESHOLD_CM = 5;

    private final Telemetry telemetry;


    public ArtifactTracker(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        colorSensors = new RevColorSensorV3[ArtifactLocation.values().length];
        for (ArtifactLocation location : ArtifactLocation.values()) {
            colorSensors[location.index] = hardwareMap.get(RevColorSensorV3.class, location.hardwareName);
        }

        indicatorLight = hardwareMap.get(Servo.class, INDICATOR_LIGHT_NAME);
    }

    private boolean hasBall(RevColorSensorV3 sensor) {
        double dist = sensor.getDistance(DistanceUnit.CM);
        return !Double.isNaN(dist) && dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;

    }


    public boolean hasBall(ArtifactLocation location) {
        return hasBall(colorSensors[location.index]);
    }

    private void setLEDYellow() {
        indicatorLight.setPosition(0.388);
    }

    private void setLEDRed() {
        indicatorLight.setPosition(0.277);
    }

    private void setLEDGreen() {
        indicatorLight.setPosition(0.500);
    }

    public void updateLED() {
        switch (getArtifactCount()) {
            case 0:
                indicatorLight.setPosition(0); // off
                break;
            case 1:
                setLEDRed();
                break;
            case 2:
                setLEDYellow();
                break;
            case 3:
            default:
                setLEDGreen();
                break;
        }
    }


    public int getArtifactCount() {
        int count = 0;
        for (RevColorSensorV3 sensor : colorSensors) {
            if (hasBall(sensor)) {
                count++;
            }
        }
        return count;
    }

    public boolean hasAllArtifacts() {
        return getArtifactCount() == MAX_ARTIFACT_COUNT;
    }

    public void reportDetections() {
        telemetry.addData(
                "Front Artifact",
                hasBall(ArtifactLocation.FRONT) ? "Detected" : "Not Detected"
        );
        telemetry.addData(
                "Middle Artifact",
                hasBall(ArtifactLocation.MIDDLE) ? "Detected" : "Not Detected"
        );
        telemetry.addData(
                "Back Artifact",
                hasBall(ArtifactLocation.BACK) ? "Detected" : "Not Detected"
        );
        updateLED();
    }
}