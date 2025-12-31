package org.firstinspires.ftc.teamcode.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.List;

public abstract class RawDeviceController<T extends HardwareDevice> extends OpMode {
    private final Class<? extends T> deviceClass;
    private List<T> devices;
    private int curDeviceIndex;

    private void setDeviceIndexSafe(int newIndex) {
        while (newIndex >= devices.size()) {
            newIndex -= devices.size();
        }
        while (newIndex < 0) {
            newIndex += devices.size();
        }
        curDeviceIndex = newIndex;
    }

    @SuppressWarnings("unchecked")
    public RawDeviceController(Class<? extends T> deviceClass) {
        this.deviceClass = deviceClass;
    }

    @Override
    public void init() {
        devices = hardwareMap.getAll(deviceClass);
        curDeviceIndex = 0;
    }

    @Override
    public void loop() {
        if (devices.isEmpty()) {
            telemetry.addLine("No devices available!");
            devices = hardwareMap.getAll(deviceClass);
            return;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            setDeviceIndexSafe(curDeviceIndex - 1);
        }
        if (gamepad1.dpadRightWasPressed()) {
            setDeviceIndexSafe(curDeviceIndex + 1);
        }
        final T device = devices.get(curDeviceIndex);

        telemetry.addData("Current Device Index", curDeviceIndex);
        telemetry.addData("Current Device Name", device.getDeviceName());
        telemetry.addData("Current Device Connection", device.getConnectionInfo());
        telemetry.addLine("[Use D-pad left & right to change device index]");

        logDeviceData(device);

        telemetry.addLine("[Hold A to control device]");

        if (gamepad1.a) {
            controlDevice(device);
        }
    }

    protected abstract void controlDevice(T device);

    protected abstract void logDeviceData(T device);
}
