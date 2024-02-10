package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;
@SuppressWarnings("unused")
public class IntakeElevator extends ProfiledPIDSubsystem {
  private final TalonFX LeftElevator = new TalonFX(Constants.ElevatorLeftCANID);
  private static final TrapezoidProfile.Constraints ProfileConstraints = new TrapezoidProfile.Constraints(MetersPerSecond.of(4),MetersPerSecondPerSecond.of(1));
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0,0.43,2.83,0.07);
  /** The shooter subsystem for the robot. */
  public IntakeElevator() {
    super(new ProfiledPIDController(0.1, 0, 0,ProfileConstraints));
  
    setGoal(0);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = elevatorFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    LeftElevator.setVoltage(output + feedforward);
  }


  @Override
  public double getMeasurement() {
    return LeftElevator.getPosition().getValueAsDouble();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

}