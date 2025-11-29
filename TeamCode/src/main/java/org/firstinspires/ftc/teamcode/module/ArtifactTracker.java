package org.firstinspires.ftc.teamcode.module;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.lights.RGBIndicator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*If we ever consider mild sorting efforts and tracking whether or not we have three artifacts inside
 * we'll use this class
 */
@Config
public class ArtifactTracker {
    public static final String FAR_COLOR_SENSOR_NAME = "ColorSensorFar";
    public static final String MIDDLE_COLOR_SENSOR_NAME = "ColorSensorMiddle";
    public static final String NEAR_COLOR_SENSOR_NAME = "ColorSensorNear";
    public static final String INDICATOR_LIGHT_NAME = "LED Light";
    public static final int MAX_ARTIFACT_COUNT = 3;

    public enum ArtifactLocation {
        NEAR(0, NEAR_COLOR_SENSOR_NAME),
        MIDDLE(1, MIDDLE_COLOR_SENSOR_NAME),
        FAR(2, FAR_COLOR_SENSOR_NAME);

        public final int index;
        public final String hardwareName;

        private ArtifactLocation(int index, String hardwareName) {
            this.index = index;
            this.hardwareName = hardwareName;
        }
    }

    private final RevColorSensorV3[] colorSensors;

    private final RGBIndicator indicatorLight;


    public static final double ARTIFACT_DISTANCE_THRESHOLD_CM = 5;

    private final Telemetry telemetry;


    public ArtifactTracker(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        colorSensors = new RevColorSensorV3[ArtifactLocation.values().length];
        for (ArtifactLocation location : ArtifactLocation.values()) {
            colorSensors[location.index] = hardwareMap.get(RevColorSensorV3.class, location.hardwareName);
        }

        indicatorLight = hardwareMap.get(RGBIndicator.class, INDICATOR_LIGHT_NAME);
    }

    private boolean hasBall(RevColorSensorV3 sensor) {
        double dist = sensor.getDistance(DistanceUnit.CM);
        return !Double.isNaN(dist) && dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;

    }

    public boolean hasBall(ArtifactLocation location) {
        return hasBall(colorSensors[location.index]);
    }

    public boolean hasSomeArtifacts() {
        return numArtifacts() > 0 && numArtifacts() < MAX_ARTIFACT_COUNT;

    }

    public void setLEDViolet() {
        telemetry.addLine("CAREFUL!!  YOU HAVE TOO MANY ARTIFACTS!! REVERSE INTAKE NOW!!");
        indicatorLight.update(0.722);
    }

    public void setLEDYellow() {
        telemetry.addLine("ROBOT HAS SOME BUT NOT ALL ARTIFACTS!!  KEEP INTAKING!!");
        indicatorLight.update(0.388);
    }

    public void setLEDRed() {
        telemetry.addLine("THE ROBOT IS EMPTY!!  START INTAKING!!");
        indicatorLight.update(0.277);
    }

    public void setLEDGreen() {
        telemetry.addLine("THE ROBOT IS FULL OF ARTIFACTS!! GO SHOOT NOW!!");
        indicatorLight.update(0.500);
    }


    public int numArtifacts() {
        int count = 0;
        for (RevColorSensorV3 sensor : colorSensors) {
            if (hasBall(sensor)) {
                count++;
            }
        }
        return count;
    }

    public boolean hasAllArtifacts() {
        return numArtifacts() == MAX_ARTIFACT_COUNT;
    }

    public void reportDetections() {
        telemetry.addData(
                "Near Artifact",
                hasBall(ArtifactLocation.NEAR) ? "Detected" : "Not Detected"
        );
        telemetry.addData(
                "Middle Artifact",
                hasBall(ArtifactLocation.MIDDLE) ? "Detected" : "Not Detected"
        );
        telemetry.addData(
                "Far Artifact",
                hasBall(ArtifactLocation.FAR) ? "Detected" : "Not Detected"
        );
        telemetry.update();
    }
}