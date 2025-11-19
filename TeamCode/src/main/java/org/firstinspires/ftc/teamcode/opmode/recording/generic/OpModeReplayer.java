package org.firstinspires.ftc.teamcode.opmode.recording.generic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.Map;

public class OpModeReplayer extends OpMode {
    private final OpModeRecording recording;
    private final OpMode opMode;

    public OpModeReplayer(OpModeRecording recording) {
        this.recording = recording;
        this.opMode = recording.createOpMode();
    }

    private static void registerRecordingAsOpMode(OpModeManager manager, String name, OpModeRecording recording) {
        manager.register(
                new OpModeMeta.Builder()
                        .setName(name)
                        .setGroup("-RECORDING-")
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .build(),
                new OpModeReplayer(recording)
        );
    }

    @OpModeRegistrar
    public static void registerRecordings(OpModeManager manager) {
        for (Map.Entry<String, OpModeRecording> recording : OpModeRecording.loadRecordings().entrySet()) {
            registerRecordingAsOpMode(manager, recording.getKey(), recording.getValue());
        }
    }

    @Override
    public void init() {
        opMode.init();
    }

    @Override
    public void init_loop() {
        opMode.init_loop();
    }

    @Override
    public void start() {
        opMode.start();
        recording.restartRecording();
    }

    @Override
    public void loop() {
        OpModeRecording.State state = recording.getCurrentState();
        if (state != null && getRuntime() >= state.getTimestamp()) {
            state.setupState(gamepad1, gamepad2);
            recording.advanceState();
        }

        opMode.loop();
    }

    @Override
    public void stop() {
        opMode.stop();
    }
}
