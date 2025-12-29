package org.firstinspires.ftc.teamcode.module;


public enum ArtifactLocation {
    SLOT_ONE(1, Spindexer.FIRST_SLOT_ANGLE, "Slot One", false),
    SLOT_TWO(2, Spindexer.SECOND_SLOT_ANGLE, "Slot Two", false),
    SLOT_THREE(3,Spindexer.THIRD_SLOT_ANGLE, "Slot Three", false);


    public final int index;
    public final String hardwareName;

    public final double angle;

    public boolean hasBall;



    //check how to store colors for the artifact location, should store either purple or green

    ArtifactLocation(int index, double angle, String hardwareName, boolean hasBall) {
        this.index = index;
        this.angle = angle;
        this.hardwareName = hardwareName;
        this.hasBall = hasBall;
    }


}
