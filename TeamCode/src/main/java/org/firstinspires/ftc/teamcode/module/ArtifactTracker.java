package org.firstinspires.ftc.teamcode.module;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IndicatorColorValues;


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

    /* Contains the colors to for each amount of balls we can have at once.
     * The color's index is the number of held balls it will represent
     */
    public static double[] INDICATOR_COLORS = {
            IndicatorColorValues.OFF,
            IndicatorColorValues.VIOLET,
            IndicatorColorValues.BLUE,
            IndicatorColorValues.GREEN,
    };


    private final RevColorSensorV3[] colorSensors;

    private final Servo indicatorLight;


    public static double ARTIFACT_DISTANCE_THRESHOLD_CM = 6;

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

    public void updateLED() {
        indicatorLight.setPosition(
                INDICATOR_COLORS[Math.min(getArtifactCount(), INDICATOR_COLORS.length)]
        );
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