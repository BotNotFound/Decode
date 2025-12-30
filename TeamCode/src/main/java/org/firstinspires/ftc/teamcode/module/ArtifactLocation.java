package org.firstinspires.ftc.teamcode.module;


public enum ArtifactLocation {
    SLOT_ONE(0, Spindexer.FIRST_SLOT_ANGLE, "Slot One"),
    SLOT_TWO(1, Spindexer.SECOND_SLOT_ANGLE, "Slot Two"),
    SLOT_THREE(2, Spindexer.THIRD_SLOT_ANGLE, "Slot Three");


    public final int index;
    public final String hardwareName;

    public final double angle;

    //check how to store colors for the artifact location, should store either purple or green

    ArtifactLocation(int index, double angle, String hardwareName) {
        this.index = index;
        this.angle = angle;
        this.hardwareName = hardwareName;
    }

    public ArtifactLocation getNextLocation() {
        switch (this) {
            case SLOT_ONE:
                return SLOT_TWO;
            case SLOT_TWO:
                return SLOT_THREE;
            case SLOT_THREE:
                return SLOT_ONE;
        }
        throw new IllegalStateException();
    }
}
