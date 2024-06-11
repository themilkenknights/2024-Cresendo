package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intakes;

public class turnnshoot extends SequentialCommandGroup {
    private final PIDController turningPID = new PIDController(0.1, 0, 0);
    private double targetHeading = 0;
    private double TargetAngularVelocity = 0;
    private Translation2d cor = new Translation2d();

    private SwerveRequest request(boolean shoot) {
        if (shoot) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                targetHeading = -40;
                cor = new Translation2d(0.250825, -0.250825);
            } else {
                targetHeading = -160;
                cor = new Translation2d(0.250825, 0.250825);
            }
        } else {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                targetHeading = -60;
                cor = new Translation2d(0.250825, -0.250825);
            } else {
                targetHeading = -120;
                cor = new Translation2d(0.250825, 0.250825);
            }
        }
        turningPID.setSetpoint(targetHeading);
        TargetAngularVelocity = turningPID.calculate(m_drive.getState().Pose.getRotation().getDegrees());
        return new SwerveRequest.RobotCentric().withCenterOfRotation(cor).withRotationalRate(TargetAngularVelocity);

    }

    private final CommandSwerveDrivetrain m_drive;
    private final Intakes m_intakes;

    public turnnshoot(CommandSwerveDrivetrain drivetrain, Intakes intake) {
        Shuffleboard.getTab("intakes").add(turningPID);
        turningPID.setTolerance(5);// degrees
        m_drive = drivetrain;
        m_intakes = intake;
        addCommands(new ParallelDeadlineGroup(m_drive.applyRequest(() -> request(true)).until(turningPID::atSetpoint)
                .andThen(m_drive.runOnce(() -> m_drive.setControl(new SwerveRequest.SwerveDriveBrake())),
                        intake.goUp())),
                m_intakes.Shoot());
    }

    public turnnshoot(CommandSwerveDrivetrain drivetrain, Intakes intake, boolean AutoIntake, boolean AutoAlign,
            boolean GoBack) {
        Shuffleboard.getTab("intakes").add(turningPID);
        turningPID.setTolerance(5);// degrees
        m_drive = drivetrain;
        m_intakes = intake;
        if (AutoAlign) {
            addCommands(Align.RightHPAlign());
        }
        if (AutoIntake) {
            addCommands(m_intakes.AutoHPin());
        }

        addCommands(new ParallelDeadlineGroup(m_drive.applyRequest(() -> request(true)).until(turningPID::atSetpoint)
                .andThen(m_drive.runOnce(() -> m_drive.setControl(new SwerveRequest.SwerveDriveBrake())),
                        intake.goUp())),
                m_intakes.Shoot());
        if (GoBack) {
            addCommands(new InstantCommand(() -> turningPID.setSetpoint(0)),
                    m_drive.applyRequest(() -> request(false)).until(turningPID::atSetpoint)
                            .andThen(m_drive.runOnce(() -> m_drive.setControl(new SwerveRequest.SwerveDriveBrake()))));
        }
    }
}
