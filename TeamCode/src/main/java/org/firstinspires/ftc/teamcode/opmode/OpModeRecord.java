package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;

import java.io.Serializable;
import java.util.ArrayList;

public class OpModeRecord implements Serializable {
    private static final long serialVersionUID = 1L;

    private final Robot.AllianceColor allianceColor;
    private final OpModeState[] record;
    private int curState;
    protected OpModeRecord(Robot.AllianceColor allianceColor, OpModeState[] record) {
        this.allianceColor = allianceColor;
        this.record = record;
        this.curState = 0;
    }

    public Robot.AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void restartRecord() {
        curState = 0;
    }

    public boolean isRecordComplete() {
        return curState >= record.length;
    }

    public OpModeState getCurrentState() {
        if (curState >= record.length) {
            return null;
        }
        return record[curState];
    }

    public void moveToNextState() {
        curState++;
    }

    public static class OpModeState implements Serializable {
        private static final long serialVersionUID = 1L;

        private byte[] gamepad1Bytes;
        private byte[] gamepad2Bytes;

        public OpModeState() {}
        public OpModeState(Gamepad gamepad1, Gamepad gamepad2) {
            this.gamepad1Bytes = gamepad1.toByteArray();
            this.gamepad2Bytes = gamepad2.toByteArray();
        }

        public void setupState(Gamepad gamepad1, Gamepad gamepad2) {
            gamepad1.fromByteArray(gamepad1Bytes);
            gamepad2.fromByteArray(gamepad2Bytes);
        }
    }

    public static class Recorder {
        private static final OpModeState[] RECORD_ARRAY_TYPE = new OpModeState[0];

        private final Robot.AllianceColor allianceColor;
        private final ArrayList<OpModeState> record;

        public Recorder(Robot.AllianceColor allianceColor) {
            this.allianceColor = allianceColor;
            this.record = new ArrayList<>();
        }

        public void recordCycle(Gamepad gamepad1, Gamepad gamepad2) {
            record.add(new OpModeState(gamepad1, gamepad2));
        }

        public OpModeRecord finishRecording() {
            return new OpModeRecord(allianceColor, record.toArray(RECORD_ARRAY_TYPE));
        }
    }
}
