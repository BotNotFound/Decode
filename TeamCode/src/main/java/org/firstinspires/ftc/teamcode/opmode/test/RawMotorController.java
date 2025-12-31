package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.util.List;

@TeleOp(group = "rawcontroller")
public class RawMotorController extends OpMode {
    private List<DcMotorController> controllers;
    private int curControllerIdx;
    private int curMotorPort;


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
        curMotorPort = rotateInt(
                newPort,
                LynxConstants.INITIAL_MOTOR_PORT,
                LynxConstants.INITIAL_MOTOR_PORT + LynxConstants.NUMBER_OF_MOTORS
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
        controllers = hardwareMap.getAll(DcMotorController.class);
        curControllerIdx = 0;
        curMotorPort = 0;
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
            setNewPortSafe(curMotorPort + 1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            setNewPortSafe(curMotorPort - 1);
        }

        final DcMotorController controller = controllers.get(curControllerIdx);
        final double newPower = gamepad1.left_stick_y;

        telemetry.addData("Current Motor Controller Index", curControllerIdx);
        telemetry.addData("Current Motor Controller", controller.getDeviceName());
        telemetry.addLine("Use dpad up + down to cycle controllers");
        telemetry.addData("Current Motor Port", curMotorPort);
        telemetry.addLine("Use dpad left + right to cycle ports");
        telemetry.addData("Current Motor Power", controller.getMotorPower(curMotorPort));
        telemetry.addLine("Use left stick y while a is pressed to set power");

        if (gamepad1.aWasPressed()) {
            controller.setMotorMode(curMotorPort, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.a) {
            controller.setMotorPower(curMotorPort, newPower);
        }
    }
}
