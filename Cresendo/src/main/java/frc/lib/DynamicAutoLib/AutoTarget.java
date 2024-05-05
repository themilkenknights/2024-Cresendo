package frc.lib.DynamicAutoLib;

import edu.wpi.first.math.geometry.Translation2d;

public abstract class AutoTarget {
    Translation2d position;

    public AutoTarget(Translation2d position) {
        this.position = position;

    }
}
