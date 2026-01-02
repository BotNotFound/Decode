package org.firstinspires.ftc.teamcode.module;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Indicator {
    public static final String INDICATOR_LIGHT_NAME = "LED Light";
    private final Servo indicatorLight;
    public Indicator(HardwareMap hardwareMap){ indicatorLight = hardwareMap.get(Servo.class, INDICATOR_LIGHT_NAME); }

    public void setPosition(double position){ indicatorLight.setPosition(position); }


}
