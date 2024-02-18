package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
//import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
//import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator SwerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Field2d field;
    public Pose2d simOdometryPose = new Pose2d();

    public Swerve() {

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "rio");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(3, 0.1, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,
        // getGyroYaw(), getModulePositions());
        SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(),
                getModulePositions(), new Pose2d());
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        ShuffleboardTab tab = Shuffleboard.getTab("swerve");
        
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                -rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        if (Robot.isSimulation()) {
            // SwerveModuleState[] measuredStates =
            // new SwerveModuleState[] {
            // swerveModuleStates[0], swerveModuleStates[0], swerveModuleStates[0],
            // swerveModuleStates[0]
            // };
            // ChassisSpeeds speeds =
            // Constants.Swerve.swerveKinematics.toChassisSpeeds(measuredStates);
            simOdometryPose = simOdometryPose.transformBy(new Transform2d(translation.getX(),
                    translation.getY(),
                    new Rotation2d(rotation)).times(0.02));
            /*
             * simOdometryPose.exp(
             * speeds.vxMetersPerSecond * .02,
             * speeds.vyMetersPerSecond * .02,
             * speeds.omegaRadiansPerSecond * .02));
             */

        }
    }

    private DoubleSupplier Velecity(SwerveModule module){
        return ()->module.getState().speedMetersPerSecond;
    }
    private DoubleSupplier Angle(SwerveModule module){
        return ()->module.getState().angle.getDegrees();
    }
    @Override
    public void initSendable(SendableBuilder builder){
        ShuffleboardTab tab = Shuffleboard.getTab("swerve");

        ShuffleboardLayout lay0 = tab.getLayout("Module "+mSwerveMods[0].moduleNumber, BuiltInLayouts.kGrid);
        lay0.addDouble("Mod 0 Angle",Angle(mSwerveMods[0]));
        lay0.addDouble("Mod 0  Velocity",Velecity(mSwerveMods[0]));


        ShuffleboardLayout lay1 = tab.getLayout("Module "+mSwerveMods[1].moduleNumber, BuiltInLayouts.kGrid);
        lay1.addDouble("Mod 1 Angle",Angle(mSwerveMods[1]));
        lay1.addDouble("Mod 1 Velocity",Velecity(mSwerveMods[1]));

        ShuffleboardLayout lay2 = tab.getLayout("Module "+mSwerveMods[2].moduleNumber, BuiltInLayouts.kGrid);
        lay2.addDouble("Mod 2 Angle",Angle(mSwerveMods[2]));
        lay2.addDouble("Mod 2nVelocity",Velecity(mSwerveMods[2]));

        ShuffleboardLayout lay3 = tab.getLayout("Module "+mSwerveMods[1].moduleNumber, BuiltInLayouts.kGrid);
        lay3.addDouble("Mod 3 Angle",Angle(mSwerveMods[3]));
        lay3.addDouble("Mod 3 Velocity",Velecity(mSwerveMods[3]));

     }        /*for (SwerveModule mod : mSwerveMods) {
            ShuffleboardLayout layout0 = tab.getLayout("Mod " + mod.moduleNumber, BuiltInLayouts.kList);
            builder.addDoubleProperty("Mod " + (mod.moduleNumber) + " CANcoder", new DoubleSupplier(){
                @Override
                public Double getValueAsDouble(){
                    return mod.getCANcoder().getDegrees();
                }
            }
            );
            layout0.add("Mod " + (mod.moduleNumber) + " Angle", mod.getPosition().angle.getDegrees());
            layout0.add("Mod " + (mod.moduleNumber) + " Velocity", mod.getState().speedMetersPerSecond);
        }*/
    

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        robotRelativeSpeeds.omegaRadiansPerSecond = -robotRelativeSpeeds.omegaRadiansPerSecond;
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        if (Robot.isSimulation()) {

            simOdometryPose = simOdometryPose.transformBy(new Transform2d(
                    targetSpeeds.vxMetersPerSecond,
                    targetSpeeds.vyMetersPerSecond,

                    new Rotation2d(targetSpeeds.omegaRadiansPerSecond)));
        }
        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return SwerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        SwerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        SwerveDrivePoseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        SwerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        SwerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {

        SwerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());
        if (Robot.isSimulation()) {
            field.setRobotPose(simOdometryPose);
        } else {
            field.setRobotPose(SwerveDrivePoseEstimator.getEstimatedPosition());
        }

        /*
         * ShuffleboardLayout layout0 =
         * tab.getLayout(mSwerveMods[0].moduleNumber+" the module",
         * BuiltInLayouts.kList);
         * layout0.add("Mod " + mSwerveMods[0]moduleNumber + " CANcoder",
         * mSwerveMods[0].getCANcoder().getDegrees());
         * layout0.add("Mod " + mSwerveMods[0].moduleNumber + " Angle",
         * mSwerveMods[0].getPosition().angle.getDegrees());
         * layout0.add("Mod " + mSwerveMods[0].moduleNumber + " Velocity",
         * mSwerveMods[0].getState().speedMetersPerSecond);
         */

    }

    @Override
    public void simulationPeriodic() {/*
                                       * //SwerveDrivePoseEstimator.update(new Rotation2d(),getModulePositions());
                                       * SwerveModuleState[] measuredStates =
                                       * new SwerveModuleState[] {
                                       * mSwerveMods[0].getState(), mSwerveMods[1].getState(),
                                       * mSwerveMods[2].getState(), mSwerveMods[3].getState()
                                       * };
                                       * // SwerveModulePosition[] mSwerveModulePositions =
                                       * // new SwerveModulePosition[] {
                                       * // mSwerveMods[0].getPosition(), mSwerveMods[1].getPosition(),
                                       * mSwerveMods[2].getPosition(), mSwerveMods[3].getPosition()
                                       * // };
                                       * ChassisSpeeds speeds =
                                       * Constants.Swerve.swerveKinematics.toChassisSpeeds(measuredStates);
                                       * simOdometryPose =
                                       * simOdometryPose.exp(
                                       * new Twist2d(
                                       * speeds.vxMetersPerSecond * .02,
                                       * speeds.vyMetersPerSecond * .02,
                                       * speeds.omegaRadiansPerSecond * .02));
                                       */
        // SwerveDrivePoseEstimator.resetPosition(simOdometryPose.getRotation(),mSwerveModulePositions,simOdometryPose);
        // setPose(getPose().transformBy(new Transform2d()));
    }
}