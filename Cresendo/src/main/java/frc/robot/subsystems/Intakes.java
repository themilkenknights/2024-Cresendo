package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.Constants;
import frc.robot.subsystems.IntakeElevator.Positions;

public class Intakes extends SubsystemBase {

  public static enum state {
    FWD, REV, SLOW, OFF
  }

  private final IntakeElevator intakeElevator;
  private TalonFX topIntake = new TalonFX(Constants.topIntakeCANID);
  private TalonFX bottomIntake = new TalonFX(Constants.bottomIntakeIntakeCANID);
  private DigitalInput backIR = new DigitalInput(Constants.backIRPORT);
  private DigitalInput frontIR = new DigitalInput(Constants.frontIRPORT);

  /** Setup the Intakes. */
  public Intakes() {
    intakeElevator = new IntakeElevator();
  }

  public Command setTopIntakeState(Intakes.state intakeState) {

    if (intakeState == state.FWD) {
      return runOnce(() -> topIntake.set(1));
    } else if (intakeState == state.REV) {
      return runOnce(() -> topIntake.set(-1));
    } else {
      return runOnce(() -> topIntake.set(0));
    }
  }

  public Command setBottomIntakeState(Intakes.state intakeState) {

    if (intakeState == state.FWD) {
      return runOnce(() -> bottomIntake.set(1));
    } else {
      return runOnce(() -> bottomIntake.set(0));
    }
  }

  public boolean getBackIR() {
    return !(backIR.get());
  }

  public boolean getFrontIR() {
    return !(frontIR.get());
  }

  public boolean getNotFrontIR() {
    return (frontIR.get());
  }

  public Command HPin() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(Positions.HP),TopIntakeByBeambreak(),intakeElevator.gotoHeight(Positions.GROUND));
  }

  public Command TopIntakeByBeambreak() {
    return new SequentialCommandGroup(setTopIntakeState(state.FWD), waitUntil(this::getBackIR),
        setTopIntakeState(state.OFF));
  }

  public Command TopOutakeByBeambreak() {
    return new SequentialCommandGroup(setTopIntakeState(state.FWD), waitUntil(this::getNotFrontIR), waitSeconds(0.25),
        setTopIntakeState(state.OFF));
  }

 

  public Command GroundPickUP() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.GROUND),
        setBottomIntakeState(state.FWD),
        waitSeconds(3),
        setBottomIntakeState(state.OFF));
    // TODO: integrate sensor(some code writen in comment bellow, trigger still
    // needs to be implemented/added)

    /*
     * return new
     * SequentialCommandGroup(intakeElevator.setHeight(IntakeElevator.Positions.
     * GROUND),
     * setBottomIntakeState(state.ON),
     * Commands.waitUntil(this::getBeambreak),
     * setBottomIntakeState(state.OFF));
     * 
     */
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    ShuffleboardTab tab = Shuffleboard.getTab("intakes");
    tab.add("ElevatorPID", intakeElevator.getController());
    tab.add("Elevator", intakeElevator);
    tab.add(frontIR);
    tab.add(backIR);
    super.initSendable(builder);
    intakeElevator.initSendable(builder);

    tab.add("TopIntake", topIntake);
    tab.add("BottomIntake", bottomIntake);

    ShuffleboardLayout commandlLayout = tab.getLayout("IntakeCommands", BuiltInLayouts.kGrid);
    commandlLayout.withSize(2, 4);
    commandlLayout.add("out", TopOutakeByBeambreak());
    commandlLayout.add("in", TopIntakeByBeambreak());
    commandlLayout.add("Ground", GroundPickUP());
    commandlLayout.add("HP", HPin());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
