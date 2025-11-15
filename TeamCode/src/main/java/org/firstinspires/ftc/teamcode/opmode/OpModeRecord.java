package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;

import java.io.Serializable;
import java.util.ArrayList;

public class OpModeRecord {
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

    public static final class SerializedRecord implements Serializable {
        private static final long serialVersionUID = 1L;

        private Robot.AllianceColor allianceColor;
        private SerializedState[] record;
        public SerializedRecord() {}

        private SerializedRecord(Robot.AllianceColor allianceColor, SerializedState[] record) {
            this.allianceColor = allianceColor;
            this.record = record;
        }

        public static SerializedRecord serializeRecord(OpModeRecord record) {
            if (record == null || record.record == null) {
                throw new IllegalArgumentException("cannot serialize uninitialized record!");
            }
            final SerializedState[] states = new SerializedState[record.record.length];
            for (int i = 0; i < states.length; i++) {
                states[i] = SerializedState.serializeState(record.record[i]);
            }
            return new SerializedRecord(record.allianceColor, states);
        }

        public static OpModeRecord deserializeRecord(SerializedRecord record) {
            if (record == null || record.record == null) {
                throw new IllegalArgumentException("cannot deserialize uninitialized record!");
            }
            final OpModeState[] states = new OpModeState[record.record.length];
            for (int i = 0; i < states.length; i++) {
                states[i] = SerializedState.deserializeState(record.record[i]);
            }
            return new OpModeRecord(record.allianceColor, states);
        }

        private static final class SerializedState implements Serializable {
            private static final long serialVersionUID = 1L;

            private byte[] gamepad1;
            private byte[] gamepad2;
            public SerializedState() {}

            private SerializedState(byte[] gamepad1, byte[] gamepad2) {
                this.gamepad1 = gamepad1;
                this.gamepad2 = gamepad2;
            }

            public static SerializedState serializeState(OpModeState state) {
                if (state == null || state.gamepad1 == null || state.gamepad2 == null) {
                    throw new IllegalArgumentException("cannot serialize uninitialized state!");
                }

                return new SerializedState(
                        state.getGamepad1().toByteArray(),
                        state.getGamepad2().toByteArray()
                );
            }

            public static OpModeState deserializeState(SerializedState state) {
                if (state == null || state.gamepad1 == null || state.gamepad2 == null) {
                    throw new IllegalArgumentException("cannot deserialize uninitialized state!");
                }

                final OpModeState deserialized = new OpModeState();
                deserialized.gamepad1.fromByteArray(state.gamepad1);
                deserialized.gamepad2.fromByteArray(state.gamepad2);
                return deserialized;
            }
        }
    }

    public static class OpModeState {
        private final Gamepad gamepad1;

        private final Gamepad gamepad2;

        public OpModeState() {
            this.gamepad1 = new Gamepad();
            this.gamepad2 = new Gamepad();
        }
        public OpModeState(Gamepad gamepad1, Gamepad gamepad2) {
            this();
            this.gamepad1.copy(gamepad1);
            this.gamepad2.copy(gamepad2);
        }

        public Gamepad getGamepad1() {
            return gamepad1;
        }

        public Gamepad getGamepad2() {
            return gamepad2;
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
