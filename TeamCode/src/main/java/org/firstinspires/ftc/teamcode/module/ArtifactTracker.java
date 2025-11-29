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

    private final RevColorSensorV3 colorSensorNear;
    private final RevColorSensorV3 colorSensorMiddle;
    private final RevColorSensorV3 colorSensorFar;

    private final Servo LED;


    public static final double ARTIFACT_DISTANCE_THRESHOLD_CM = 5;

    private final Telemetry telemetry;


    public ArtifactTracker(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        colorSensorNear = hardwareMap.get(RevColorSensorV3.class, COLOR_SENSOR_NEAR);
        colorSensorMiddle = hardwareMap.get(RevColorSensorV3.class, COLOR_SENSOR_MIDDLE);
        colorSensorFar = hardwareMap.get(RevColorSensorV3.class, COLOR_SENSOR_FAR);
        LED = hardwareMap.get(Servo.class, "LED Light");


    }

    private boolean hasBall(RevColorSensorV3 sensor) {
        double dist = sensor.getDistance(DistanceUnit.CM);
        return !Double.isNaN(dist) && dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;

    }

    public boolean hasAllArtifacts() {
        return (numArtifacts() == 3) || (hasBall(colorSensorNear) &&
                hasBall(colorSensorMiddle) && hasBall(colorSensorFar));
    }

    public boolean hasNoArtifacts() {
        return (numArtifacts() == 0) || (!hasBall(colorSensorNear) &&
                !hasBall(colorSensorMiddle) && !hasBall(colorSensorFar));

    }

    public boolean hasSomeArtifacts() {
        return numArtifacts() > 0 && numArtifacts() < 3;

    }
    /*This is because when we have four artifacts the fourth ball get pushed up the hood and the color sensor at
     * the back will think its too far. This is the only cases I know for when there are 4 artifacts in our robot.
     */
    //Not needed actually if we're intaking two at the front it will think there's too many
    /*public boolean tooManyArtifacts(){
        return hasBall(colorSensorNear) && hasBall(colorSensorMiddle) && !hasBall(colorSensorFar);
    }*/

    public void setLEDViolet() {
        telemetry.addData("CAREFUL!! ", "YOU HAVE TOO MANY ARTIFACTS!! REVERSE INTAKE NOW!!");
        telemetry.update();
        LED.setPosition(0.722);
    }

    public void setLEDYellow() {
        telemetry.addData("ROBOT HAS SOME BUT NOT ALL ARTIFACTS!! ", "KEEP INTAKING!!");
        telemetry.update();
        LED.setPosition(0.388);
    }

    public void setLEDRed() {
        telemetry.addData("THE ROBOT IS EMPTY!! ", "START INTAKING!!");
        telemetry.update();
        LED.setPosition(0.277);
    }

    public void setLEDGreen() {
        telemetry.addData("THE ROBOT IS FULL OF ARTIFACTS!!", "GO SHOOT NOW!!");
        telemetry.update();
        LED.setPosition(0.500);
    }


    public int numArtifacts() {
        int count = 0;
        if (hasBall(colorSensorNear)) {
            count++;
        }
        if (hasBall(colorSensorMiddle)) {
            count++;
        }
        if (hasBall(colorSensorFar)) {
            count++;
        }
        return count;
    }

    public void reportDetections() {
        if (hasBall(colorSensorNear)) {
            telemetry.addData("Artifact ", "detected at the front of the robot");
        }
        else {
            telemetry.addData("Artifact ", "NOT detected at the front of the robot");
        }
        if (hasBall(colorSensorMiddle)) {
            telemetry.addData("Artifact ", "detected in the middle of the robot");
        }
        else {
            telemetry.addData("Artifact ", "NOT detected in the middle of the robot");
        }
        if (hasBall(colorSensorFar)) {
            telemetry.addData("Artifact ", "detected at the back of the robot");
        }
        else {
            telemetry.addData("Artifact ", "NOT detected at the back of the robot");
        }
        telemetry.update();
    }

}