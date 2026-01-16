package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.controller.PIDController;

public class PositionalPIDFController extends PIDController {
    private double feedforwardThreshold = 1;

    public PositionalPIDFController(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd);
        setF(kf);
    }

    @Override
    public double calculate(double pv) {
        final double pidPower = super.calculate(pv);
        final double error = getPositionError();
        if (Math.abs(error) < feedforwardThreshold) {
            return pidPower;
        }
        return pidPower + getF() * Math.signum(error);
    }

    public double getFeedforwardThreshold() {
        return feedforwardThreshold;
    }

    public void setFeedforwardThreshold(double kfErrorThreshold) {
        this.feedforwardThreshold = kfErrorThreshold;
    }
}
