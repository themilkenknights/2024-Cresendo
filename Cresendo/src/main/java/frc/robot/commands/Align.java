package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;



public class Align{
  private static final PathConstraints pathConstraints = new PathConstraints(2, 2, 540, 540);
    public static Command AMPAlign(){
      return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Amp aligner"),pathConstraints);
    }

    public static Command RightHPAlign(){
      return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("HP a aligner"),pathConstraints);
    }

    public static Command LeftHPAlign(){
      return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("HP b aligner"),pathConstraints);
    }
 }