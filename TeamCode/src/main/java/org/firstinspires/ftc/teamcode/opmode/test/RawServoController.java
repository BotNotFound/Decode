package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.util.List;

@TeleOp(group = "rawcontroller")
public class RawServoController extends OpMode {
    private List<ServoController> controllers;
    private int curControllerIdx;
    private int curServoPort;
    private double basePosition;


    private static int rotateInt(int value, int minInclusive, int maxExclusive) {
        if (minInclusive == maxExclusive) {
            return minInclusive;
        }

        final int range = maxExclusive - minInclusive;
        while (value >= maxExclusive) {
            value -= range;
        }
        while (value < minInclusive) {
            value += range;
        }
        return value;
    }

    private void setNewPortSafe(int newPort) {
        curServoPort = rotateInt(
                newPort,
                LynxConstants.INITIAL_SERVO_PORT,
                LynxConstants.INITIAL_SERVO_PORT + LynxConstants.NUMBER_OF_SERVO_CHANNELS
        );
    }

    private void setNewControllerSafe(int newControllerIdx) {
        curControllerIdx = rotateInt(
                newControllerIdx,
                0,
                controllers.size()
        );
        setNewPortSafe(0);
    }

    @Override
    public void init() {
        controllers = hardwareMap.getAll(ServoController.class);
        curControllerIdx = 0;
        curServoPort = 0;
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            setNewControllerSafe(curControllerIdx + 1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            setNewControllerSafe(curControllerIdx - 1);
        }

        if (gamepad1.dpadRightWasPressed()) {
            setNewPortSafe(curServoPort + 1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            setNewPortSafe(curServoPort - 1);
        }

        final ServoController controller = controllers.get(curControllerIdx);
        final double newPosition = gamepad1.left_stick_y;

        telemetry.addData("Current Servo Controller Index", curControllerIdx);
        telemetry.addData("Current Servo Controller", controller.getDeviceName());
        telemetry.addLine("Use dpad up + down to cycle controllers");
        telemetry.addData("Current Servo Port", curServoPort);
        telemetry.addLine("Use dpad left + right to cycle ports");
        telemetry.addData("Current Servo Position", controller.getServoPosition(curServoPort));
        telemetry.addLine("Use left stick y while a is pressed to set position");
        telemetry.addLine("Use left stick y while b is pressed to move position");

        if (gamepad1.a) {
            controller.setServoPosition(curServoPort, newPosition);
        }

        if (gamepad1.bWasPressed()) {
            basePosition = controller.getServoPosition(curServoPort);
        }
        if (gamepad1.b) {
            controller.setServoPosition(curServoPort, basePosition + newPosition);
        }
    }
}
