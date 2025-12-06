package org.firstinspires.ftc.teamcode.module;

public enum ArtifactLocation {
    FRONT(0, ArtifactTracker.FRONT_COLOR_SENSOR_NAME),
    MIDDLE(1, ArtifactTracker.MIDDLE_COLOR_SENSOR_NAME),
    BACK(2, ArtifactTracker.BACK_COLOR_SENSOR_NAME);

    public final int index;
    public final String hardwareName;

    ArtifactLocation(int index, String hardwareName) {
        this.index = index;
        this.hardwareName = hardwareName;
    }
}
