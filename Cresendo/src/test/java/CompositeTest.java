// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.DynamicAutoLib.CompositeZone;
import edu.wpi.first.wpilibj.DriverStation;



/** Add your docs here. */
public class CompositeTest {
    @Test
    public void FlipTestE(){
        ArrayList<Ellipse2d> array = new ArrayList<>();
        array.add(new Ellipse2d(new Pose2d(), 1));
        CompositeZone x = new CompositeZone(array,new ArrayList<>());


        boolean a = x.isInZone(new Pose2d(1,0,new Rotation2d()));
        x.setAlliance(DriverStation.Alliance.Red);
        assertNotEquals(a,x.isInZone(new Pose2d(1,0,new Rotation2d())));
    }

    @Test
    public void FlipTestR(){
        ArrayList<Rectangle2d> array = new ArrayList<>();
        array.add(new Rectangle2d(new Pose2d(), 2,2));
        CompositeZone x = new CompositeZone(new ArrayList<>(),array);



        boolean a = x.isInZone(new Pose2d(1,0,new Rotation2d()));

        x.setAlliance(DriverStation.Alliance.Red);
        assertNotEquals(a,x.isInZone(new Pose2d(1,0,new Rotation2d())));
    }

    @Test
    public void MultiItems(){
        ArrayList<Rectangle2d> array = new ArrayList<>(0);
        array.add(new Rectangle2d(new Pose2d(), 0.01,0.01));
        array.add(new Rectangle2d(new Pose2d(new Translation2d(0, 1),new Rotation2d()), 2,2));

        ArrayList<Ellipse2d> array2 = new ArrayList<>(0);
        array2.add(new Ellipse2d(new Pose2d(),5));


        CompositeZone x = new CompositeZone(array2,new ArrayList<>());//array);

        boolean a = x.isInZone(new Pose2d(1,0,new Rotation2d()));

        x.setAlliance(DriverStation.Alliance.Red);
        assertNotEquals(a,x.isInZone(new Pose2d(1,01,new Rotation2d())));
    }


    
}
