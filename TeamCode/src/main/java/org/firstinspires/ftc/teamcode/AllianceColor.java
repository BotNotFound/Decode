package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public enum AllianceColor {
    RED(24, 35, 0.15),
    BLUE(20, 135, -0.15);

    public final int targetAprilTagID;
    public final double tagAimOffsetZero;
    public final double tagAimOffsetMultiplier;

    AllianceColor(int targetAprilTagID, double tagAimOffsetZero, double tagAimOffsetMultiplier) {
        this.targetAprilTagID = targetAprilTagID;
        this.tagAimOffsetZero = tagAimOffsetZero;
        this.tagAimOffsetMultiplier = tagAimOffsetMultiplier;
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
