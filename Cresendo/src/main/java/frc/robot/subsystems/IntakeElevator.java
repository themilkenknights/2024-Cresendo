package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeElevator extends ProfiledPIDSubsystem {

  private static final double spoolsize = 0.5 * Math.PI;
  private static final double reduction = 9;

  private static double inchestorotations(double inches) {
    return spoolsize * inches * reduction;
  }

  private final TalonFX LeftElevator = new TalonFX(Constants.ElevatorLeftCANID);
  private final TalonFX RightElevator = new TalonFX(Constants.ElevatorRightCANID);

  private static final TrapezoidProfile.Constraints ProfileConstraints = new TrapezoidProfile.Constraints(inchestorotations(4), (inchestorotations(1)));
  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0.43, 2.83, 0.07);

  private static final ElevatorSim sim = new
  ElevatorSim(DCMotor.getFalcon500(1), reduction, 6, spoolsize/37, 0, 500, false,
   0);
  public static enum Positions {
    GROUND, HP, AMP
  }

  /*
   * private static class stage {
   * public double kg,ks,kv,setpoint;
   * public stage(double setpoint,double kg,double ks,double kv){
   * this.ks = ks;
   * this.kg = kg;
   * this.kv = kv;
   * this.setpoint = setpoint;
   * }
   * }
   * private static final stage[] feedforwardSteps = {new stage(0, 0, 0, 0)};
   */

  /**
   * DO NOT CALL DIRECTLY. Use Intakes subsystem insted
   */
  public IntakeElevator() {
    super(new ProfiledPIDController(0.5, 0, 0, ProfileConstraints));
    RightElevator.setInverted(true);
    RightElevator.setControl(new Follower(LeftElevator.getDeviceID(), false));
    LeftElevator.setPosition(0);

    setGoal(0);
  }

  public void initSendable(SendableBuilder builder) {
    // Shuffleboard.getTab("intakes").add("ElevatorPID",m_controller);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    if (getMeasurement() > inchestorotations(25.25)) {// TODO: set all of the stage encoder poses for feed forward
                                                      // properly, and put
      // in kv,kg,ks, etc
      // note: this may need to be moved
      elevatorFeedforward = new ElevatorFeedforward(0, 0.43, 2.83, 0.07);
    } else if (getMeasurement() > inchestorotations(14)) {
      // in kv,kg,ks, etc
      // note: this may need to be moved
      elevatorFeedforward = new ElevatorFeedforward(0, 0.43, 2.83, 0.07);
    } else if (getMeasurement() > inchestorotations(8)) {
      elevatorFeedforward = new ElevatorFeedforward(0, 0.43, 2.83, 0.07);
    } else {
      elevatorFeedforward = new ElevatorFeedforward(0, 0.43, 2.83, 0.07);
    }

    double feedforward = elevatorFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    LeftElevator.setVoltage(output + feedforward);

  }

  @Override
  public double getMeasurement() {
    return (Robot.isReal())?LeftElevator.getPosition().getValueAsDouble():m_controller.getSetpoint().position;
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public Command setDown() {
    return runOnce(() -> setGoal(1));
  }

  public Command setUp() {
    // TODO: set properly
    return runOnce(() -> setGoal(inchestorotations(25)));
  }

  @Override
  public void simulationPeriodic() {
    sim.setInput((LeftElevator.getMotorVoltage().getValueAsDouble()));

    sim.update(0.02);
    LeftElevator.setPosition(sim.getPositionMeters());
  }

  public Command gotoHeight(Positions height) {
    // sequence: first, set based off of the height param, then only finish when it
    // reaches the setpoint
    switch (height) {
      case GROUND:
      return new ProfiledPIDCommand(m_controller, this::getMeasurement, 0, this::useOutput,this).until(this.m_controller::atGoal);
      case AMP:
        return new ProfiledPIDCommand(m_controller, this::getMeasurement, inchestorotations(20), this::useOutput,this).until(this.m_controller::atGoal);
      case HP:
       return new ProfiledPIDCommand(m_controller, this::getMeasurement, inchestorotations(25), this::useOutput,this).until(this.m_controller::atGoal);
       default:
       return new ProfiledPIDCommand(m_controller, this::getMeasurement, 0, this::useOutput,this);

  }

}
}
