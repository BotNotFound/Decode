package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public enum Motif {

    // PPG is Purple-Purple-Green
    // GPP is Green-Purple-Purple
    // PGP is Purple-Green-Purple

    PPG(23),
    GPP(21),
    PGP(22);

    public final int motifID;

    Motif(int motifID) {
        this.motifID = motifID;
    }

    @NonNull
    @Override
    public String toString() {
        switch (this) {
            case PPG:
                return "Motif ID = 23";
            case GPP:
                return "GPP ID = 21";
            case PGP:
                return "PGP ID = 22";
            default:
                return super.toString();
        }
    }








}
