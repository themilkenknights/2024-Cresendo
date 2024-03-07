package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;

public class AprilTagCommand extends Command {
  boolean seen = false;
  CommandSwerveDrivetrain drivetrain;

  public AprilTagCommand(CommandSwerveDrivetrain sw) {
    drivetrain = sw;
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-knights") != false) {
      seen = true;

      // use network tables to get limelight botpose (array: x,y,z,roll,pitch, yaw,latency)
      double[] botpose =
          NetworkTableInstance.getDefault()
              .getTable("limelight-knights")
              .getEntry("botpose_wpiblue")
              .getDoubleArray(new double[7]);
      Pose2d visionMeasurement2d =
          new Pose2d(botpose[0], botpose[1], drivetrain.getRotation3d().toRotation2d());
      SmartDashboard.putNumberArray("vision", botpose);

      drivetrain.addVisionMeasurement(
          visionMeasurement2d, Timer.getFPGATimestamp() - (botpose[6] / 1000.0));
    }
  }

  @Override
  public boolean isFinished() {
    return seen;
  }
}
