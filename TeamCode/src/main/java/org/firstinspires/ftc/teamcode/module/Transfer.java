package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Transfer {

    private final CRServo transferServo;
    private final CRServo transferServo2;

    public static final String TRANSFER_SERVO_NAME = "Transfer";
    public static final String TRANSFER2_SERVO_NAME = "Transfer 2";

    //adjust the servo power as needed for the transfer.
    public static double POWER_VALUE = 1.0;


    public Transfer(HardwareMap hwMap) {
        transferServo = hwMap.get(CRServo.class, TRANSFER_SERVO_NAME);
        transferServo2 = hwMap.get(CRServo.class, TRANSFER2_SERVO_NAME);

        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setTransferPower(double power) {
        transferServo.setPower(power);
        transferServo2.setPower(power);
    }

    public void startTransfer() {
        setTransferPower(POWER_VALUE);
    }

    public void stopTransfer() {
        setTransferPower(0.0);
    }

    //this method is useful if color sensor detects wrong artifact
    public void reverseTransfer() {
        setTransferPower(-POWER_VALUE);
    }


}
