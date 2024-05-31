package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.limits;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    // pp telem
    private static final DoubleArrayPublisher posePub = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("/PathPlanner/currentPose").publish();
    private static final DoubleArrayPublisher targetPub = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("/PathPlanner/currentPose").publish();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public Field2d field2d = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
    // new SwerveRequest.SysIdSwerveRotation();
    // private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new
    // SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
     * new SysIdRoutine.Config(
     * null,
     * Volts.of(4),
     * null,
     * (state) -> SignalLogger.writeString("state", state.toString())),
     * new SysIdRoutine.Mechanism(
     * (volts) -> setControl(RotationCharacterization.withVolts(volts)),
     * null,
     * this));
     * private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
     * new SysIdRoutine.Config(
     * null,
     * Volts.of(7),
     * null,
     * (state) -> SignalLogger.writeString("state", state.toString())),
     * new SysIdRoutine.Mechanism(
     * (volts) -> setControl(SteerCharacterization.withVolts(volts)),
     * null,
     * this));
     * 
     * /* Change this to the sysid routine you want to test
     */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        updateFalconSettings();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        updateFalconSettings();
    }

    public void updateFalconSettings() {
        for (int i = 0; i < 4; i++) {
            getModule(i).getDriveMotor().getConfigurator().apply(limits.DriveLimits);
            getModule(i).getSteerMotor().getConfigurator().apply(limits.TurnLimits);
        }

    }
    public Command goBreak(){
        
        return runOnce(()->{
            for (int i = 0; i < 4; i++) {
                getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
                getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
            }
        });
    }

    public Command goCoast(){
        
        return runOnce(()->{
            for (int i = 0; i < 4; i++) {
                getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
                getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
            }
        });
    }
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        PathPlannerLogging.setLogCurrentPoseCallback((pose)->posePub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()}));
        PathPlannerLogging.setLogTargetPoseCallback((pose)->targetPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()}));
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose.transformBy(new Transform2d(new Translation2d(0.88265/2,0), new Rotation2d())), // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
                                            () -> {
                                                // Boolean supplier that controls when the path will be mirrored for the red alliance
                                                // This will flip the path being followed to the red side of the field.
                                                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                                  
                                                var alliance = DriverStation.getAlliance();
                                                if (alliance.isPresent()) {
                                                  return alliance.get() == DriverStation.Alliance.Red;
                                                }
                                                return false;
                                            },
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}