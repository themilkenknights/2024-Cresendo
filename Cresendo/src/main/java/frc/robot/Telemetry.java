package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    private final Field2d field2d = new Field2d();
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;

    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    //private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    //private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    ShuffleboardTab swerveShuffleboardTab = Shuffleboard.getTab("swerve");

    /* Robot speeds for general checking */
  /*  private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();
    */

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
      
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        field2d.setRobotPose(pose);
        SmartDashboard.putData(field2d);

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        swerveShuffleboardTab.addDouble("speed",velocities::getNorm);
        swerveShuffleboardTab.addDouble("velocity x",velocities::getX);
        swerveShuffleboardTab.addDouble("velocity y",velocities::getY);
        swerveShuffleboardTab.addDouble("OdometryPeriod",()->{return state.OdometryPeriod;});
       //swerveShuffleboardTab.addDoubleArray("speeds",()->{return state.ModuleStates[0].;});
        /* Telemeterize the module's states */
        /*for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            swerveShuffleboardTab.addDouble("Module " + i+"Angle", ()->state.ModuleStates[0].angle.getDegrees());
            swerveShuffleboardTab.addDouble("Module " + i+ "Speed", ()->state.ModuleStates[0].speedMetersPerSecond);
        }*/
        swerveShuffleboardTab.addDouble("Module " + 0+"Angle", ()->state.ModuleStates[0].angle.getDegrees());
        swerveShuffleboardTab.addDouble("Module " + 0+ "Speed", ()->state.ModuleStates[0].speedMetersPerSecond);
        swerveShuffleboardTab.addDouble("Module " + 1+"Angle", ()->state.ModuleStates[1].angle.getDegrees());
        swerveShuffleboardTab.addDouble("Module " + 1+ "Speed", ()->state.ModuleStates[1].speedMetersPerSecond);
        swerveShuffleboardTab.addDouble("Module " + 2+"Angle", ()->state.ModuleStates[2].angle.getDegrees());
        swerveShuffleboardTab.addDouble("Module " + 2+ "Speed", ()->state.ModuleStates[2].speedMetersPerSecond);
        swerveShuffleboardTab.addDouble("Module " + 3+"Angle", ()->state.ModuleStates[3].angle.getDegrees());
        swerveShuffleboardTab.addDouble("Module " + 3+ "Speed", ()->state.ModuleStates[3].speedMetersPerSecond);
       
        publisher.set(state.ModuleStates);
    }
}