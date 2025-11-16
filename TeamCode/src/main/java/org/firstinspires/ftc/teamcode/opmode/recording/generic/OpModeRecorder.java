package org.firstinspires.ftc.teamcode.opmode.recording.generic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OpModeRecorder extends OpMode {
    private static final String RECORD_NAME_AVAILABLE_CHARS =
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890 ";

    private final OpMode opMode;
    private OpModeRecording.Builder recordingBuilder;

    private StringBuilder recordingName;
    private int recordNameCurIdx;
    private boolean nameFinalized;

    public OpModeRecorder(Class<? extends OpMode> opModeClass) {
        opMode = OpModeRecording.createOpMode(opModeClass);
    }

    private static int rotateInt(int cur, int offset, int max) {
        int next = cur + offset;
        while (next >= max) {
            next -= max;
        }
        while (next < 0) {
            next += max;
        }
        return next;
    }

    @Override
    public void init() {
        recordingName = new StringBuilder();
        nameFinalized = false;
    }

    @Override
    public void init_loop() {
        if (nameFinalized) {
            opMode.init_loop();
            return;
        }

        while (recordNameCurIdx > recordingName.length()) {
            recordNameCurIdx -= recordingName.length();
        }
        while (recordNameCurIdx < 0) {
            recordNameCurIdx += recordingName.length();
        }
        if (recordNameCurIdx == recordingName.length()) {
            recordingName.append(RECORD_NAME_AVAILABLE_CHARS.charAt(0));
        }

        final int curCharIdx = RECORD_NAME_AVAILABLE_CHARS.indexOf(recordingName.charAt(recordNameCurIdx));
        if (gamepad1.dpadUpWasPressed()) {
            recordingName.setCharAt(
                    recordNameCurIdx,
                    RECORD_NAME_AVAILABLE_CHARS.charAt(
                            rotateInt(curCharIdx, 1, RECORD_NAME_AVAILABLE_CHARS.length())
                    )
            );
        }
        if (gamepad1.dpadDownWasPressed()) {
            recordingName.setCharAt(
                    recordNameCurIdx,
                    RECORD_NAME_AVAILABLE_CHARS.charAt(
                            rotateInt(curCharIdx, -1, RECORD_NAME_AVAILABLE_CHARS.length())
                    )
            );
        }

        if (gamepad1.dpadRightWasPressed()) {
            recordNameCurIdx = rotateInt(recordNameCurIdx, 1, recordingName.length());
        }
        if (gamepad1.dpadLeftWasPressed()) {
            recordNameCurIdx = rotateInt(recordNameCurIdx, -1, recordingName.length());
        }

        telemetry.addLine("Record Name [DPAD to change, A to submit]:");
        telemetry.addLine(recordingName.toString());

        if (gamepad1.aWasPressed()) {
            nameFinalized = true;
            opMode.init();
        }
    }

    @Override
    public void start() {
        if (!nameFinalized) {
            nameFinalized = true;
            opMode.init();
        }
        opMode.start();
        recordingBuilder = new OpModeRecording.Builder(opMode.getClass());
    }

    @Override
    public void loop() {
        opMode.loop();
        recordingBuilder.addState(gamepad1, gamepad2, getRuntime());
    }

    @Override
    public void stop() {
        opMode.stop();

        if (recordingBuilder != null) {
            OpModeRecording recording = recordingBuilder.build();
            OpModeReplayer.registerRecording(recordingName.toString(), recording);
            recordingBuilder = null;
        }
    }
}
