package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import java.util.UUID;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Record New Autonomous")
public class Recorder extends TeleOp {
    private OpModeRecord.Recorder recorder;

    @Override
    public void start() {
        super.start();
        recorder = new OpModeRecord.Recorder(robot.getAllianceColor());
    }

    @Override
    public void loop() {
        super.loop();
        recorder.recordCycle(gamepad1, gamepad2);
    }

    @Override
    public void stop() {
        super.stop();
        Replayer.Hub.registerRecord(UUID.randomUUID().toString(), recorder.finishRecording());
    }
}
