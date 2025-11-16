package org.firstinspires.ftc.teamcode.opmode.recording;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Robot;

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

public class OpModeRecord implements Serializable {
    @Serial
    private static final long serialVersionUID = 1L;
    private static final String TAG = OpModeRecord.class.getSimpleName();

    public static final File RECORDINGS_FOLDER = Environment.getExternalStorageDirectory();
    public static final String RECORDINGS_FILE_EXTENSION = ".omr";

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

    private static String toRecordFilename(String recordName) {
        return RECORDINGS_FOLDER.getPath() + recordName + RECORDINGS_FILE_EXTENSION;
    }

    public static void saveRecord(String recordName, OpModeRecord record) {
        final String filename = toRecordFilename(recordName);
        try (FileOutputStream fileOutputStream = new FileOutputStream(filename);
             ObjectOutputStream objectOutputStream = new ObjectOutputStream(fileOutputStream)) {
            objectOutputStream.writeObject(record);
        }
        catch (IOException e) {
            Log.e(TAG, e.toString());
        }
    }

    public static Map<String, OpModeRecord> loadRecords() {
        final File[] recordFiles = RECORDINGS_FOLDER.listFiles((dir, name) -> name.endsWith(RECORDINGS_FILE_EXTENSION));
        final Hashtable<String, OpModeRecord> records = new Hashtable<>();
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
            final OpModeRecord record;
            try (FileInputStream fileInputStream = new FileInputStream(recordFile);
                 ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream)) {
                record = (OpModeRecord) objectInputStream.readObject();
            }
            catch (Exception e) {
                Log.e(TAG, e.toString());
                continue;
            }

            records.put(recordName, record);
        }

        return records;
    }

    private static OpModeManager curManager;

    @OpModeRegistrar
    public static void registerRecords(OpModeManager manager) {
        curManager = manager;
        final Map<String, OpModeRecord> records = loadRecords();
        for (Map.Entry<String, OpModeRecord> recordEntry : records.entrySet()) {
            registerRecordAsOpMode(manager, recordEntry.getKey(), recordEntry.getValue());
        }
    }

    private static void registerRecordAsOpMode(OpModeManager manager, String recordName, OpModeRecord record) {
        manager.register(
                new OpModeMeta.Builder()
                        .setName(recordName)
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .setGroup("-RECORDED-")
                        .build(),
                new Replayer(record)
        );
    }

    public static void registerRecordAsOpMode(String recordName, OpModeRecord record) {
        if (curManager == null) {
            throw new IllegalStateException("No OpModeManager has been initialized!");
        }
        registerRecordAsOpMode(curManager, recordName, record);
    }

    public static class OpModeState implements Serializable {
        @Serial
        private static final long serialVersionUID = 1L;

        private final byte[] gamepad1Bytes;
        private final byte[] gamepad2Bytes;
        private final double timestamp;

        public OpModeState(Gamepad gamepad1, Gamepad gamepad2, double timestamp) {
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

    public static class Recorder {
        private static final OpModeState[] RECORD_ARRAY_TYPE = new OpModeState[0];

        private final Robot.AllianceColor allianceColor;
        private final ArrayList<OpModeState> record;

        public Recorder(Robot.AllianceColor allianceColor) {
            this.allianceColor = allianceColor;
            this.record = new ArrayList<>();
        }

        public void recordCycle(Gamepad gamepad1, Gamepad gamepad2, double timestamp) {
            record.add(new OpModeState(gamepad1, gamepad2, timestamp));
        }

        public OpModeRecord finishRecording() {
            return new OpModeRecord(allianceColor, record.toArray(RECORD_ARRAY_TYPE));
        }
    }
}
