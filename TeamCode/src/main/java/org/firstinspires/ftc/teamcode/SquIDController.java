package org.firstinspires.ftc.teamcode;

public class SquIDController {
    private double kP;
    private double tolerance;
    private double target;
    private double curError;

    public SquIDController(double kP) {
        this.kP = kP;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setTarget(double targetValue) {
        this.target = targetValue;
    }

    public boolean atTarget() {
        return Math.abs(curError) <= tolerance;
    }

    public double calculate(double measuredValue) {
        curError = target - measuredValue;
        if (atTarget()) {
            return 0.0;
        }

        return Math.copySign(kP * Math.sqrt(Math.abs(curError)), curError);
    }
}
