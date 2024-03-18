// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intakes;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //private CommandSwerveDrivetrain SwerveStuff = TunerConstants.DriveTrain;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Save outputs to a new log // old path LogFileUtil.addPathSuffix(logPath, "_sim")
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Position
    Logger.recordOutput("pose",m_robotContainer.drivetrain.getPose());

    // Note stuff
    Logger.recordOutput("note inside",m_robotContainer.s_Intakes.getFrontIR());

    // Elevator
    Logger.recordOutput("elevator setpoint",m_robotContainer.s_Intakes.getElevator().getController().getSetpoint().position);
    Logger.recordOutput("elevator measurement",m_robotContainer.s_Intakes.getElevator().getMeasurement());

    // Intake
    //Logger.recordOutput("intake rotation velocity",m_robotContainer.s_Intakes);


    // Climber
    Logger.recordOutput("climber setpoint",m_robotContainer.s_Climb.getController().getSetpoint());
    Logger.recordOutput("climber measurement",m_robotContainer.s_Climb.getMeasurement());

    // Limelight
    Logger.recordOutput("april tag detected",LimelightHelpers.getTV("limelight-knights"));

    
    //Logger.recordOutput("pose",m_robotContainer.drivetrain.getPose());
    // add update with estimatedPosition, + Timer.getFPGATimeStamp()
    //SwerveStuff.addVisionMeasurement(SwerveStuff.getPose()  , Timer.getFPGATimestamp());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    m_robotContainer.reEnable();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
