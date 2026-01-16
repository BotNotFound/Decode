package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.controller.PIDController;

public class PositionalPIDFController extends PIDController {
    private double feedforwardThreshold = 1;
    private double posF;

    public PositionalPIDFController(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd);
        posF = kf;
    }

    @Override
    public double calculate(double pv) {
        final double pidPower = super.calculate(pv);
        final double error = getSetPoint() - pv;
        if (Math.abs(error) < feedforwardThreshold) {
            return pidPower;
        }
        return pidPower + posF * -Math.signum(error);
    }

    public double getFeedforwardThreshold() {
        return feedforwardThreshold;
    }

    public void setFeedforwardThreshold(double kfErrorThreshold) {
        this.feedforwardThreshold = kfErrorThreshold;
    }

    @Override
    public void setF(double kf) {
        posF = kf;
    }

    @Override
    public double getF() {
        return posF;
    }

    @Override
    public void setPIDF(double kp, double ki, double kd, double kf) {
        super.setPIDF(kp, ki, kd, 0);
        setF(kf);
    }
}
