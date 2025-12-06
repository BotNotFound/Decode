package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public enum AllianceColor {
    RED(24, 35),
    BLUE(20, 145);

    public final int targetAprilTagID;
    public final double tagAimOffsetZero;

    AllianceColor(int targetAprilTagID, double tagAimOffsetZero) {
        this.targetAprilTagID = targetAprilTagID;
        this.tagAimOffsetZero = tagAimOffsetZero;
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
