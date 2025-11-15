package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

@Autonomous(name = "Record New Autonomous")
public class Recorder extends TeleOp {
    private OpModeRecord.Recorder recorder;

    private static final String RECORD_NAME_AVAILABLE_CHARS =
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890 ";
    private StringBuilder recordName;
    private int recordNameCurIdx;

    private static int rotateInt(int cur, int offset, int max) {
        int next = cur + offset;
        while (cur >= max) {
            cur -= max;
        }
        while (cur < 0) {
            cur += max;
        }
        return next;
    }

    @Override
    public void init() {
        super.init();
        recordName = new StringBuilder();
    }

    @Override
    public void init_loop() {
        super.init_loop();

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
            recordNameCurIdx = rotateInt(recordNameCurIdx, 1, recordName.length());
        }
        if (gamepad1.dpadLeftWasPressed()) {
            recordNameCurIdx = rotateInt(recordNameCurIdx, -1, recordName.length());
        }

        telemetry.addLine("Record Name [use DPAD to change]:");
        telemetry.addLine(recordName.toString());
    }

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
        Replayer.Hub.registerRecord(recordName.toString(), recorder.finishRecording());
    }
}
