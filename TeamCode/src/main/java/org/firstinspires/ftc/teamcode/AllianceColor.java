package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public enum AllianceColor {
    RED(24),
    BLUE(20);

    public final int targetAprilTagID;

    AllianceColor(int targetAprilTagID) {
        this.targetAprilTagID = targetAprilTagID;
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
