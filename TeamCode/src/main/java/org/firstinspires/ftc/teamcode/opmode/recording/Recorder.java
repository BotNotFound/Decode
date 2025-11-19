package org.firstinspires.ftc.teamcode.opmode.recording;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

@Autonomous(name = "Record New Autonomous")
public class Recorder extends TeleOp {
    private OpModeRecord.Recorder recorder = null;

    private static final String RECORD_NAME_AVAILABLE_CHARS =
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890 ";
    private StringBuilder recordName;
    private int recordNameCurIdx;
    private boolean nameFinalized;

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
        recordName = new StringBuilder();
        nameFinalized = false;
    }

    @Override
    public void init_loop() {
        if (nameFinalized) {
            super.init_loop();
            return;
        }

        while (recordNameCurIdx > recordName.length()) {
            recordNameCurIdx -= recordName.length();
        }
        while (recordNameCurIdx < 0) {
            recordNameCurIdx += recordName.length();
        }
        if (recordNameCurIdx == recordName.length()) {
            recordName.append(RECORD_NAME_AVAILABLE_CHARS.charAt(0));
        }

        final int curCharIdx = RECORD_NAME_AVAILABLE_CHARS.indexOf(recordName.charAt(recordNameCurIdx));
        if (gamepad1.dpadUpWasPressed()) {
            recordName.setCharAt(
                    recordNameCurIdx,
                    RECORD_NAME_AVAILABLE_CHARS.charAt(
                            rotateInt(curCharIdx, 1, RECORD_NAME_AVAILABLE_CHARS.length())
                    )
            );
        }
        if (gamepad1.dpadDownWasPressed()) {
            recordName.setCharAt(
                recordNameCurIdx,
                RECORD_NAME_AVAILABLE_CHARS.charAt(
                        rotateInt(curCharIdx, -1, RECORD_NAME_AVAILABLE_CHARS.length())
                )
            );
        }

        if (gamepad1.dpadRightWasPressed()) {
            recordNameCurIdx = rotateInt(recordNameCurIdx, 1, recordName.length() + 1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            recordNameCurIdx = rotateInt(recordNameCurIdx, -1, recordName.length());
        }

        telemetry.addLine("Record Name [DPAD to change, A to submit]:");
        telemetry.addLine(recordName.toString());

        if (gamepad1.aWasPressed()) {
            nameFinalized = true;
            super.init();
        }
    }

    @Override
    public void start() {
        if (!nameFinalized) {
            nameFinalized = true;
            super.init();
        }
        super.start();
        recorder = new OpModeRecord.Recorder(robot.getAllianceColor());
    }

    @Override
    public void loop() {
        super.loop();
        recorder.recordCycle(gamepad1, gamepad2, getRuntime());
    }

    @Override
    public void stop() {
        super.stop();

        if (recorder != null) {
            Replayer.Hub.registerRecord(recordName.toString(), recorder.finishRecording());
            recorder = null;
        }
    }
}
