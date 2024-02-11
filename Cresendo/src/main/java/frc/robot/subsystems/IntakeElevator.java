package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;

public class IntakeElevator extends ProfiledPIDSubsystem {
  private final TalonFX LeftElevator = new TalonFX(Constants.ElevatorLeftCANID);
  private static final TrapezoidProfile.Constraints ProfileConstraints = new TrapezoidProfile.Constraints(MetersPerSecond.of(4),MetersPerSecondPerSecond.of(1));
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0,0.43,2.83,0.07);

  public static enum Positions {
    GROUND,HP,AMP
  }
  /**
  * DO NOT CALL DIRECTLY. Use Intakes subsystem insted
   */
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
  public Command setDown(){
    return runOnce(()->setGoal(0));
  }

   public Command setUp(){
    //TODO: set properly
    return runOnce(()->setGoal(20));
  }

  public Command setHeight(Positions height){
    return runOnce(()->{
      switch (height) {
        case GROUND:
          setGoal(0);
          break;
        case AMP:
          setGoal(10);//TODO: set properly
          break;
        case HP:
          setGoal(15); ;//TODO: set properly
          break;
      }
    });
  }
}