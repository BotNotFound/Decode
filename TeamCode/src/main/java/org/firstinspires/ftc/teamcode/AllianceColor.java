package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public enum AllianceColor {
    RED(24, 144, 144),
    BLUE(20, 0, 144);

    public final int targetAprilTagID;
    public final double goalPositionX;
    public final double goalPositionY;

    AllianceColor(int targetAprilTagID, double goalPositionX, double goalPositionY) {
        this.targetAprilTagID = targetAprilTagID;
        this.goalPositionX = goalPositionX;
        this.goalPositionY = goalPositionY;
    }


    @NonNull
    @Override
    public String toString() {
        switch (this) {
            case RED:
                return "Red Alliance";
            case BLUE:
                return "Blue Alliance";
            default:
                return super.toString();
        }
    }
}
