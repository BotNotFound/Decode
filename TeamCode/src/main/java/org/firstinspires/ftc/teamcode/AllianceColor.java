package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public enum AllianceColor {
    // TODO test goal angles
    RED(24, 45),
    BLUE(20, 135);

    public final int targetAprilTagID;
    public final double goalAngle;

    AllianceColor(int targetAprilTagID, double goalAngle) {
        this.targetAprilTagID = targetAprilTagID;
        this.goalAngle = goalAngle;
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
