package org.firstinspires.ftc.teamcode.opmode.recording.generic;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serial;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Map;

public class OpModeRecording implements Serializable {
    @Serial
    private static final long serialVersionUID = 1L;

    private static final String TAG = OpModeRecording.class.getSimpleName();

    public static final File RECORDINGS_FOLDER = Environment.getExternalStorageDirectory();
    public static final String RECORDINGS_FILE_EXTENSION = ".gor";

    public static <T extends OpMode> T createOpMode(Class<T> opModeClass) {
        try {
            return opModeClass.getConstructor().newInstance();
        } catch (Throwable th) {
            throw new IllegalArgumentException(
                    "Cannot construct op mode of type " + opModeClass.getName(),
                    th
            );
        }
    }

    private static String toRecordingFilename(String recordName) {
        return RECORDINGS_FOLDER.getPath() + recordName + RECORDINGS_FILE_EXTENSION;
    }

    public static void saveRecording(String recordName, OpModeRecording record) {
        final String filename = toRecordingFilename(recordName);
        try (FileOutputStream fileOutputStream = new FileOutputStream(filename);
             ObjectOutputStream objectOutputStream = new ObjectOutputStream(fileOutputStream)) {
            objectOutputStream.writeObject(record);
        }
        catch (IOException e) {
            Log.e(TAG, e.toString());
        }
    }

    public static Map<String, OpModeRecording> loadRecordings() {
        final File[] recordFiles = RECORDINGS_FOLDER.listFiles((dir, name) -> name.endsWith(RECORDINGS_FILE_EXTENSION));
        final Hashtable<String, OpModeRecording> records = new Hashtable<>();
        if (recordFiles == null) {
            return records; // no files
        }

        for (File recordFile : recordFiles) {
            if (!recordFile.isFile()) {
                continue;
            }
            if (!recordFile.canRead()) {
                continue;
            }

            final String recordFilename = recordFile.getName();
            final String recordName = recordFilename.substring(0, recordFilename.length() - RECORDINGS_FILE_EXTENSION.length());
            final OpModeRecording record;
            try (FileInputStream fileInputStream = new FileInputStream(recordFile);
                 ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream)) {
                record = (OpModeRecording) objectInputStream.readObject();
            }
            catch (Exception e) {
                Log.e(TAG, e.toString());
                continue;
            }

            records.put(recordName, record);
        }

        return records;
    }

    private final Class<? extends OpMode> opModeClass;
    private final State[] states;
    private int curStateIdx;

    private OpModeRecording(Class<? extends OpMode> opModeClass, State[] states) {
        this.opModeClass = opModeClass;
        this.states = states;
        curStateIdx = 0;
    }

    public OpMode createOpMode() {
        try {
            return createOpMode(opModeClass);
        }
        catch (IllegalArgumentException e) {
            throw new IllegalStateException(
                    "Cannot construct op mode of type " + opModeClass.getName(),
                    e.getCause()
            );
        }
    }

    public void restartRecording() {
        curStateIdx = 0;
    }

    public void advanceState() {
        if (curStateIdx < states.length) {
            curStateIdx++;
        }
    }

    public State getCurrentState() {
        if (curStateIdx >= states.length) {
            return null;
        }
        return states[curStateIdx];
    }

    public static class State implements Serializable {
        @Serial
        private static final long serialVersionUID = 1L;

        private final byte[] gamepad1Bytes;
        private final byte[] gamepad2Bytes;
        private final double timestamp;

        public State(Gamepad gamepad1, Gamepad gamepad2, double timestamp) {
            this.gamepad1Bytes = gamepad1.toByteArray();
            this.gamepad2Bytes = gamepad2.toByteArray();
            this.timestamp = timestamp;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public void setupState(Gamepad gamepad1, Gamepad gamepad2) {
            gamepad1.fromByteArray(gamepad1Bytes);
            gamepad2.fromByteArray(gamepad2Bytes);
        }
    }

    public static class Builder {
        private static final State[] STATE_ARRAY_TYPE = new State[0];

        private final ArrayList<State> states;
        private final Class<? extends OpMode> opModeClass;

        public Builder(Class<? extends OpMode> opModeClass) {
            this.opModeClass = opModeClass;
            states = new ArrayList<>();
        }

        public void addState(Gamepad gamepad1, Gamepad gamepad2, double timestamp) {
            states.add(new State(gamepad1, gamepad2, timestamp));
        }

        public OpModeRecording build() {
            return new OpModeRecording(opModeClass, states.toArray(STATE_ARRAY_TYPE));
        }
    }
}
