// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation3d;


/** Add your docs here. */
public class ForwardK {
    ArrayList<Ligament3d> nodes;
    Translation3d base = new Translation3d();
    public ForwardK(Translation3d base,ArrayList<Ligament3d> nodes) {
        this.nodes = nodes;
        this.base = base;
    }

    public ForwardK(Translation3d base,Ligament3d... nodes) {
        this.nodes = new ArrayList<>(Arrays.asList(nodes));
        this.base = base;
    }

    public void addNode(Ligament3d node){
        nodes.add(node);
    }
    public Translation3d getFinalPoint(){
        Translation3d endPoint = base;
        for (Ligament3d node : nodes) {
            endPoint = endPoint.plus(new Translation3d(node.getLength(), node.getRotation()));
        }
        return endPoint;
    }

    public static Translation3d getFinalPoint(Translation3d base, ArrayList<Ligament3d> nodes){
        Translation3d endPoint = base;
        for (Ligament3d node : nodes) {
            endPoint = endPoint.plus(new Translation3d(node.getLength(), node.getRotation()));
        }
        return endPoint;
    }

}
