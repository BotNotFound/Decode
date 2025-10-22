package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// as of 10/10/25 there is no second servo for transfer
@Config
public class Transfer {

    private final CRServo servo1;

    public static final String SERVO_ONE = "Servo1";

    //adjust the servo power as needed for the transfer.
    public static double POWER_VALUE = -1.0;

    private final Telemetry telemetry;


    public Transfer(HardwareMap hwMap, Telemetry telemetry) {
        servo1= hwMap.get(CRServo.class, SERVO_ONE);

        this.telemetry = telemetry;
    }

    public void startTransfer() {
        servo1.setPower(POWER_VALUE);
        telemetry.addData("Transfer: ", POWER_VALUE);
    }


    public void stopTransfer() {
        servo1.setPower(0.0);
        telemetry.addData("Transfer: ", 0.0);
    }
    //this method is useful if color sensor detects wrong artifact
    public void reverseTransfer() {
        servo1.setPower(-POWER_VALUE);
        telemetry.addData("Transfer: ", -POWER_VALUE);
    }


}
