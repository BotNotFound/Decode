package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

// as of 10/10/25 there is no second servo for transfer
@Config
public class Transfer{

    private final CRServo servo1;
    // private final CRServo servo2;

    public static final String SERVO_ONE = "Servo1";
    // public static final String SERVO_TWO = "Servo2";

    //adjust the servo power as needed for the transfer.
    public static double POWER_VALUE = 1.0;


    public Transfer(HardwareMap hwMap){
        servo1= hwMap.get(CRServo.class, SERVO_ONE);
        // servo2= hwMap.get(CRServo.class, SERVO_TWO);

    }

    public void setPower(double power){
        servo1.setPower(power);
        // servo2.setPower(power);

    }

    public void startTransfer(){
        servo1.setPower(POWER_VALUE);
        // servo2.setPower(POWER_VALUE);
    }


    public void stopTransfer(){
        servo1.setPower(0.0);
        // servo2.setPower(0.0);
    }
    //this method is useful if color sensor detects wrong artifact
    public void reverseTransfer(){
        servo1.setPower(-POWER_VALUE);
        // servo2.setPower(-POWER_VALUE);
    }


}
