package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import java.util.Hashtable;

public class Replayer extends TeleOp {
    private final OpModeRecord record;

    protected Replayer(OpModeRecord record) {
        this.record = record;
    }

    @Override
    public void start() {
        super.start();
        robot.setAllianceColor(record.getAllianceColor());
        record.restartRecord();
    }

    @Override
    public void loop() {
        if (!record.isRecordComplete()) {
            OpModeRecord.OpModeState state = record.getCurrentState();
            gamepad1.copy(state.getGamepad1());
            gamepad2.copy(state.getGamepad2());
            record.moveToNextState();
        }

        super.loop();
    }

    @Autonomous(name = "Replay Recorded Autos")
    public static class Hub extends OpMode {
        private static final Hashtable<String, OpModeRecord> records = new Hashtable<>();

        public static void registerRecord(String name, OpModeRecord record) {
            records.put(name, record);
        }

        // this is a field instead of a base class because we need to recreate it whenever
        // a new opmode is recorded
        private SelectableOpMode opMode;

        @Override
        public void init() {
            opMode = new SelectableOpMode("Select a record to replay", s -> {
                records.forEach((name, record) -> {
                    s.add(name, () -> new Replayer(record));
                });
            }) {};
            opMode.gamepad1 = gamepad1;
            opMode.gamepad2 = gamepad2;
            opMode.telemetry = telemetry;
            opMode.hardwareMap = hardwareMap;
            opMode.init();
        }

        @Override
        public void init_loop() {
            opMode.init_loop();
        }

        @Override
        public void start() {
            opMode.start();
        }

        @Override
        public void loop() {
            opMode.loop();
        }

        @Override
        public void stop() {
            opMode.stop();
        }
    }
}
