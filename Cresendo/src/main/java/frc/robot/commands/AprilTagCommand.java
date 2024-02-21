package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CommandSwerveDrivetrain;
//import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;

public class AprilTagCommand {

    public static Command UpdateVision(CommandSwerveDrivetrain sw) {
        return new InstantCommand(() -> {
            if (
                LimelightHelpers.getTV("limelight-knights")!=false){
                //NetworkTableInstance.getDefault().getTable("limelight-knights").getEntry("tv").getDouble(0) != 0) {
                // use network tables to get limelight botpose (array: x,y,z,roll,pitch, yaw,latency)
                double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-knights")
                        .getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
                // Pose3d visionMeasurement3d = new Pose3d();
                Pose2d visionMeasurement2d = new Pose2d(botpose[0], botpose[1], sw.getRotation3d().toRotation2d());
                SmartDashboard.putNumberArray("vision", botpose);
                // Compute the robot's field-relative position from botpose

                // visionMeasurement3d = new Pose3d((botpose[0]),(botpose[1]),(botpose[2]), new
                // Rotation3d());
                // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
                // visionMeasurement2d = (new Pose2d(0,0,new
                // Rotation2d(getGyroYaw().getDegrees())));

                sw.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp() - (botpose[6] / 1000.0));
            }
            ;
        });

    }

}
