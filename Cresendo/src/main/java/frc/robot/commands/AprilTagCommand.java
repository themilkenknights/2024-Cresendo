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
    String llname = "limelight-knights";

    CommandSwerveDrivetrain drivetrain;

    public AprilTagCommand(CommandSwerveDrivetrain sw) {

        seen = false;
        drivetrain = sw;
        SmartDashboard.putData(feild);

    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV(llname) != false) {
            seen = true;

            boolean doRejectUpdate = false;
            LimelightHelpers.SetRobotOrientation(llname, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0,
                    0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llname);
            if (Math.abs(drivetrain.getPigeon2().getRate()) > 720) // if our angular velocity is greater than 720
                                                                   // degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                drivetrain.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds, VecBuilder.fill(.6, .6, 9999999));
            }

        }
        SmartDashboard.putBoolean("Tag", seen);

    }

    @Override
    public boolean isFinished() {
        return (seen & timer.hasElapsed(1));
    }
}