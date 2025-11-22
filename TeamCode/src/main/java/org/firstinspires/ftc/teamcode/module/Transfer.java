package org.firstinspires.ftc.teamcode.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// as of 10/10/25 there is no second servo for transfer
@Config
public class Transfer {

    private final CRServo transferServo;
    private final CRServo transferServo2;

    public static final String TRANSFER_SERVO_NAME = "Transfer";
    public static final String TRANSFER2_SERVO_NAME = "Transfer 2";

    //adjust the servo power as needed for the transfer.
    public static double POWER_VALUE = 0.2;

    private final Telemetry telemetry;


    public Transfer(HardwareMap hwMap, Telemetry telemetry) {
        transferServo = hwMap.get(CRServo.class, TRANSFER_SERVO_NAME);
        transferServo2 = hwMap.get(CRServo.class, TRANSFER2_SERVO_NAME);

        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo2.setDirection(DcMotorSimple.Direction.FORWARD);

        this.telemetry = telemetry;
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
        telemetry.addData("Transfer: ", "Stopped");
    }

    //this method is useful if color sensor detects wrong artifact
    public void reverseTransfer() {
        setTransferPower(-POWER_VALUE);
    }


}
