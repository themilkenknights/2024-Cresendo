package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;


public class AprilTagCommand extends Command {
    boolean seen = false;
    Field2d feild = new Field2d();
    Timer timer = new Timer();

    CommandSwerveDrivetrain drivetrain;
    public AprilTagCommand(CommandSwerveDrivetrain sw) {

        seen = false;
        drivetrain=sw;
        SmartDashboard.putData(feild);
        
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute(){
        
        if (LimelightHelpers.getTV("limelight-knights")!=false){
                seen = true;
  
                // use network tables to get limelight botpose (array: x,y,z,roll,pitch, yaw,latency)
                double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-knights")
                        .getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
                Pose2d visionMeasurement2d = new Pose2d(botpose[0], botpose[1], new Rotation2d(3));
                SmartDashboard.putNumberArray("vision", botpose);
                //drivetrain.addVisionMeasurement(visionMeasurement2d, (Timer.getFPGATimestamp() - (botpose[6] / 1000.0))*1000);
                //drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill);
                

                drivetrain.addVisionMeasurement(visionMeasurement2d, (Timer.getFPGATimestamp() - (botpose[6] / 1000.0)),VecBuilder.fill(.7,.7,9999999));
            //feild.setRobotPose(visionMeasurement2d);

     }
     SmartDashboard.putBoolean("Tag", seen);

    }
    @Override
    public boolean isFinished(){
        return (seen&timer.hasElapsed(1));
    }
}