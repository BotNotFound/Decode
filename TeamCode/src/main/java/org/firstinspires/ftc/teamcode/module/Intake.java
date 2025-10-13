package org.firstinspires.ftc.teamcode.module;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake{
    public static final String INTAKE_MOTOR_NAME = "Intake";
    private final DcMotor intakeMotor;
    private final Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);

        this.telemetry = telemetry;
    }
    public void setPower(double power){
        intakeMotor.setPower(power);

        telemetry.addData("Intake: ", power);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);

        telemetry.addData("Intake: ", "stopped");
    }

}
