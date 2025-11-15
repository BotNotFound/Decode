package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// as of 10/10/25 there is no second servo for transfer
@Config
public class Transfer {

    private final CRServo transferServo;

    public static final String TRANSFER_SERVO_NAME = "Transfer";

    //adjust the servo power as needed for the transfer.
    public static double POWER_VALUE = -1.0;

    private final Telemetry telemetry;


    public Transfer(HardwareMap hwMap, Telemetry telemetry) {
        transferServo = hwMap.get(CRServo.class, TRANSFER_SERVO_NAME);

        this.telemetry = telemetry;
    }

    public void startTransfer() {
        transferServo.setPower(POWER_VALUE);
        telemetry.addData("Transfer: ", POWER_VALUE);
    }


    public void stopTransfer() {
        transferServo.setPower(0.0);
        telemetry.addData("Transfer: ", 0.0);
    }
    //this method is useful if color sensor detects wrong artifact
    public void reverseTransfer() {
        transferServo.setPower(-POWER_VALUE);
        telemetry.addData("Transfer: ", -POWER_VALUE);
    }


}
