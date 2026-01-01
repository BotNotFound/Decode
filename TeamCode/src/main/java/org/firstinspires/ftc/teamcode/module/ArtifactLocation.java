package org.firstinspires.ftc.teamcode.module;


public enum ArtifactLocation {
    SLOT_ONE(0, "Slot One"),
    SLOT_TWO(120, "Slot Two"),
    SLOT_THREE(240, "Slot Three");


    public final String hardwareName;

    public final double angle;

    //check how to store colors for the artifact location, should store either purple or green

    ArtifactLocation(double angle, String hardwareName) {
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

    public ArtifactLocation getPreviousLocation() {
        switch (this) {
            case SLOT_ONE:
                return SLOT_THREE;
            case SLOT_TWO:
                return SLOT_ONE;
            case SLOT_THREE:
                return SLOT_TWO;
        }
        throw new IllegalStateException();
    }
}
