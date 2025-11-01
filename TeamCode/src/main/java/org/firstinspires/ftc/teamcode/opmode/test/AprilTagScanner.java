package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class AprilTagScanner extends OpMode {
    private AprilTagProcessor processor;

    @Override
    public void init() {
        processor = AprilTagProcessor.easyCreateWithDefaults();
    }

    @Override
    public void loop() {

    }
}
