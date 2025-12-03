package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

@Autonomous(group = "test")
public class IdentifyCameras extends OpMode {
    private final ArrayList<WebcamName> cameras = new ArrayList<>();

    private void logCameraInfo(WebcamName camera) {
        telemetry.addLine(camera.getDeviceName()
                + "\n\tConnection info: " + camera.getConnectionInfo()
                + "\n\tManufacturer: " + camera.getManufacturer().name()
                + "\n\tCamera Modes:"
        );
        for (CameraCharacteristics.CameraMode mode : camera.getCameraCharacteristics().getAllCameraModes()) {
            telemetry.addLine("\t "
                    + (mode.isDefaultSize ? "[default] " : "")
                    + mode
            );
        }
    }

    private void logAllCameras() {
        for (WebcamName camera : cameras) {
            logCameraInfo(camera);
        }
        telemetry.update();
    }

    @Override
    public void init_loop() {
        logAllCameras();
    }

    @Override
    public void loop() {
        logAllCameras();
    }

    @Override
    public void init() {

        for (HardwareDevice device : hardwareMap) {
            if (device instanceof WebcamName) {
                cameras.add((WebcamName) device);
            }
        }
    }
}
