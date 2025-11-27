package org.firstinspires.ftc.teamcode.module;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.module.Intake;
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
    

    private final Telemetry telemetry;
    
    




    public ArtifactTracker(HardwareMap hwMap, Telemtry tele){
        telemetry = tele;

        colorSensorNear= hwMap.get(RevColorSensorV3.class, COLOR_SENSOR_NEAR);
        colorSensorMiddle= hwMap.get(RevColorSensorV3.class, COLOR_SENSOR_MIDDLE);
        colorSensorFar= hwMap.get(RevColorSensorV3.class, COLOR_SENSOR_FAR);
        LED = hwMap.get(Servo.class, "LED Light");


    }
    private boolean hasBall(RevColorSensorV3 sensor) {
        return sensor.getDistance(DistanceUnit.CM) <= 5;
    }
    public boolean hasAllArtifacts(){
        //return hasBall(colorSensorNear) && hasBall(colorSensorMiddle) && hasBall(colorSensorFar);
        if(numArtifacts()!=3){
            return false;
        }
        tele.addData("Robot ", "has all artifacts");
        LED.setPosition(0.500);
        return true;
    }
    public boolean hasNoArtifacts(){
        //return !hasBall(colorSensorNear) && !hasBall(colorSensorMiddle) && !hasBall(colorSensorFar);
        if(numArtifacts()!= 0){
            return false;
        }
        tele.addData("Robot ", "has no artifacts");
        LED.setPosition(0.277);
        return true;
    }

    public boolean hasSomeArtifacts(){
        if(numArtifacts()>0 && numArtifacts()<3){

            if(colorSensorNear.hasBall()){
                tele.addData("Artifact ", "detected at the front of the robot");
            }
            else{
                tele.addData("Artifact ", "NOT detected at the front of the robot");
            }
            if(colorSensorMiddle.hasBall()){
                tele.addData("Artifact ", "detected in the middle of the robot");
            }
            else{
                tele.addData("Artifact ", "NOT detected in the middle of the robot");
            }
            if(colorSensorFar.hasBall()){
                tele.addData("Artifact ", "detected at the back of the robot");
            }
            else{
                tele.addData("Artifact ", "NOT detected at the back of the robot");
            }
            LED.setPosition(0.388);
            return true;
        }
        return false;
    }
    
    public boolean tooManyArtifacts(){

        if((!colorSensorNear.hasBall() && !colorSensorFar.hasBall())
        || (colorSensorNear.hasBall() && !colorSensorFar.hasBall())){
            LED.setPosition(0.722);
            return true;
        }
        return false; 

    }
    
    public int numArtifacts(){
        int count = 0;
        if(colorSensorNear.hasBall()){
            count++;
        }
        if(colorSensorMiddle.hasBall()){
            count++;
        }
        if(colorSensorFar.hasBall()){
            count++;
        }
        return count;
    }
    
    //if the second the third one detect a artifact then stop the transfer 
    //if the second and third one have an artifact 
}