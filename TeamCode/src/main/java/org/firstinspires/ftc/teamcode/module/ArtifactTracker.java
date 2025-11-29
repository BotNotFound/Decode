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
    public static final String COLOR_SENSOR_FAR = "ColorSensorFar";
    public static final String COLOR_SENSOR_MIDDLE = "ColorSensorMiddle";
    public static final String COLOR_SENSOR_NEAR = "ColorSensorNear";

    public enum ArtifactLocation {
        NEAR(0, COLOR_SENSOR_NEAR),
        MIDDLE(1, COLOR_SENSOR_MIDDLE),
        FAR(2, COLOR_SENSOR_FAR);

        public final int index;
        public final String hardwareName;

        private ArtifactLocation(int index, String hardwareName) {
            this.index = index;
            this.hardwareName = hardwareName;
        }
    }

    private final RevColorSensorV3[] colorSensors;

    private final Servo LED;


    public static final double ARTIFACT_DISTANCE_THRESHOLD_CM = 5;

    private final Telemetry telemetry;


    public ArtifactTracker(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        colorSensors = new RevColorSensorV3[ArtifactLocation.values().length];
        for (ArtifactLocation location : ArtifactLocation.values()) {
            colorSensors[location.index] = hardwareMap.get(RevColorSensorV3.class, location.hardwareName);
        }

        LED = hardwareMap.get(Servo.class, "LED Light");


    }

    private boolean hasBall(RevColorSensorV3 sensor) {
        double dist = sensor.getDistance(DistanceUnit.CM);
        return !Double.isNaN(dist) && dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;

    }

    public boolean hasBall(ArtifactLocation location) {
        return hasBall(colorSensors[location.index]);
    }

    public boolean hasSomeArtifacts() {
        return numArtifacts() > 0 && numArtifacts() < 3;

    }

    public void setLEDViolet() {
        telemetry.addData("CAREFUL!! ", "YOU HAVE TOO MANY ARTIFACTS!! REVERSE INTAKE NOW!!");
        LED.setPosition(0.722);
    }

    public void setLEDYellow() {
        telemetry.addData("ROBOT HAS SOME BUT NOT ALL ARTIFACTS!! ", "KEEP INTAKING!!");
        LED.setPosition(0.388);
    }

    public void setLEDRed() {
        telemetry.addData("THE ROBOT IS EMPTY!! ", "START INTAKING!!");
        LED.setPosition(0.277);
    }

    public void setLEDGreen() {
        telemetry.addData("THE ROBOT IS FULL OF ARTIFACTS!!", "GO SHOOT NOW!!");
        LED.setPosition(0.500);
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

    public void reportDetections() {
        if (hasBall(ArtifactLocation.NEAR)) {
            telemetry.addData("Artifact ", "detected at the front of the robot");
        }
        else {
            telemetry.addData("Artifact ", "NOT detected at the front of the robot");
        }
        if (hasBall(ArtifactLocation.MIDDLE)) {
            telemetry.addData("Artifact ", "detected in the middle of the robot");
        }
        else {
            telemetry.addData("Artifact ", "NOT detected in the middle of the robot");
        }
        if (hasBall(ArtifactLocation.FAR)) {
            telemetry.addData("Artifact ", "detected at the back of the robot");
        }
        else {
            telemetry.addData("Artifact ", "NOT detected at the back of the robot");
        }
        telemetry.update();
    }

}