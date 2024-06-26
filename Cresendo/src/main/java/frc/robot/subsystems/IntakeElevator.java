package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.limits;

public class IntakeElevator extends ProfiledPIDSubsystem {
  // intake constants
  private static final double kP = 1.3;
  private static final double kI = 0;
  private static final double kD = 0;

  private static final double HPSetpoint = 3.6;
  private static final double AMPSetpoint = 3.1;
  private static final double AUTOSetpoint = 3.35;
  private static final double spoolsize = 0.5 * Math.PI;
  private static final double reduction = 25;

  private static double inchestorotations(double inches) {
    return spoolsize * inches * reduction;
  }
  
  private final TalonFX LeftElevator = new TalonFX(Constants.ElevatorLeftCANID);
  private final TalonFX RightElevator = new TalonFX(Constants.ElevatorRightCANID);
  private static final TrapezoidProfile.Constraints ProfileConstraints = new TrapezoidProfile.Constraints(
      inchestorotations(50), (inchestorotations(35)));
  // private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0,
  // 0.43, 2.83, 0.07);

  private static final ElevatorSim sim = new ElevatorSim(DCMotor.getFalcon500(1), reduction, 6, spoolsize, 0, 500,
      false,
      0);

  public static enum Positions {
    GROUND, HP, AMP, STOW, AUTO
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
    super(new ProfiledPIDController(kP, kI, kD, ProfileConstraints));

    //current limits
    RightElevator.getConfigurator().apply(limits.ElevatorLimits);
    LeftElevator.getConfigurator().apply(limits.ElevatorLimits);

    //follow
    RightElevator.setControl(new Follower(LeftElevator.getDeviceID(), false));
    //zero
    LeftElevator.setPosition(0);

    //tolerence
    m_controller.setTolerance(5);

    //defulting
    setDefaultCommand(new ProfiledPIDCommand(m_controller, this::getMeasurement, m_controller::getGoal, this::useOutput, this));
    setGoal(0);
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    Shuffleboard.getTab("intakes").add("ElevatorPID", m_controller);
    Shuffleboard.getTab("intakes").add("Elevator Motor", this.LeftElevator);
    Shuffleboard.getTab("intakes").addDouble("Encoder", this::getMeasurement).withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -10, "max", 1000, "Orientation", "VERTICAL"));
    
    Shuffleboard.getTab("intakes").addDouble("goal",()->m_controller.getSetpoint().position);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint

    // double feedforward = elevatorFeedforward.calculate(setpoint.position,
    // setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    LeftElevator.setVoltage(output);// + feedforward);

  }

  @Override
  public double getMeasurement() {
    return LeftElevator.getPosition().getValueAsDouble();// (Robot.isReal()) ?
                                                         // LeftElevator.getPosition().getValueAsDouble() :
                                                         // m_controller.getSetpoint().position;
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public Command setDown() {
    return runOnce(() -> setGoal(0));
  }

  public Command setAMP() {
    return runOnce(() -> setGoal(inchestorotations(AMPSetpoint)));
  }

  public Command STOW() {
    return runOnce(() -> setGoal(inchestorotations(0)));
  }

  @Override
  public void simulationPeriodic() {
    sim.setInput((LeftElevator.getMotorVoltage().getValueAsDouble()));

    sim.update(0.02);
    // LeftElevator.setPosition(sim.getPositionMeters());
    LeftElevator.setPosition(m_controller.getSetpoint().position);
  }

  public Command gotoHeight(Positions height) {
    // sequence: first, set based off of the height param, then only finish when it
    // reaches the setpoint
    switch (height) {
      case GROUND:
        return new ProfiledPIDCommand(m_controller, this::getMeasurement, 0, this::useOutput, this)
            .until(this.m_controller::atGoal);
      case AMP:
        return new ProfiledPIDCommand(m_controller, this::getMeasurement, inchestorotations(AMPSetpoint), this::useOutput, this)
            .until(this.m_controller::atGoal);
      case HP:
        return new ProfiledPIDCommand(m_controller, this::getMeasurement, inchestorotations(HPSetpoint), this::useOutput, this)
            .until(this.m_controller::atGoal);
      case AUTO:
        return new ProfiledPIDCommand(m_controller, this::getMeasurement, inchestorotations(AUTOSetpoint), this::useOutput, this)
            .until(this.m_controller::atGoal);
      default:
        return new ProfiledPIDCommand(m_controller, this::getMeasurement, 0, this::useOutput, this);

    }

  }

  public void onReEnable() {

    LeftElevator.setPosition(0);
    this.m_controller.reset(0);
    setGoal(0);
    sim.setState(0, 0);

  }
}
