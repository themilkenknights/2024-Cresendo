package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;



public class Align{
 // private static final PathConstraints pathConstraints = new PathConstraints(2, 2, 540, 540);
    public static Command AMPAlign(){
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Amp aligner"));
    }

    public static Command RightHPAlign(){
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("HP a aligner"));
    }

    public static Command LeftHPAlign(){
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("HP b aligner"));
    }
 }