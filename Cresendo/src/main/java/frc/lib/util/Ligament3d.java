// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class Ligament3d {
    private double Length = 0;

    public double getLength() {
        return Length;
    }
    private Rotation3d Rotation = new Rotation3d();
    
    public Rotation3d getRotation() {
        return Rotation;
    }
    public void setRotation(Rotation3d angle) {
        Rotation = angle;
    }

    public Ligament3d(double length, Rotation3d angle) {
        Length = length;
        Rotation = angle;
    }

}
