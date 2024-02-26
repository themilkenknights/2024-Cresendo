package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.Constants;
import frc.robot.subsystems.IntakeElevator.Positions;

public class Intakes extends SubsystemBase {

  public static enum state {
    GROUND, HP, OUT,OFF
  }

  private Mechanism2d mech = new Mechanism2d(18,10,new Color8Bit(Color.kBlueViolet));
  private final IntakeElevator intakeElevator;
  private TalonFX topIntake = new TalonFX(Constants.topIntakeCANID);
  private TalonFX midIntake = new TalonFX(Constants.midIntakeCANID);
  private TalonFX bottomIntake = new TalonFX(Constants.bottomIntakeIntakeCANID);
  private DigitalInput backIR = new DigitalInput(Constants.backIRPORT);
  private DigitalInput frontIR = new DigitalInput(Constants.frontIRPORT);

  /** Setup the Intakes. */
  public Intakes(IntakeElevator elevator) {
    this.intakeElevator = elevator;
  }

  public Command setTopIntakeState(Intakes.state intakeState) {

    if (intakeState == state.GROUND) {
      return runOnce(() -> {
        topIntake.set(1);
        midIntake.set(-1);
      });
    } else if (intakeState == state.OUT) {
      return runOnce(() -> {
        topIntake.set(-1);
        midIntake.set(-1);
      });
    }
    else if (intakeState == state.HP) {
      return runOnce(() -> {
        topIntake.set(1);
        midIntake.set(1);
      });
    } else {
      return runOnce(() -> topIntake.set(0));
    }
  }

  public Command setBottomIntakeState(Intakes.state intakeState) {

    if (intakeState == state.GROUND) {
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
    return new SequentialCommandGroup(setTopIntakeState(state.HP), waitUntil(this::getBackIR),
        setTopIntakeState(state.OFF));
  }

  public Command TopOutakeByBeambreak() {
    return new SequentialCommandGroup(setTopIntakeState(state.OUT), waitUntil(this::getNotFrontIR), waitSeconds(0.25),
        setTopIntakeState(state.OFF));
  }

 
public Command AmpOuttake(){
  return new SequentialCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.AMP),TopIntakeByBeambreak());
}
public IntakeElevator getElevator(){
  return this.intakeElevator;
}
public Command GoDown(){
  return new ParallelCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.GROUND));
}
  public Command GroundPickUP() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.GROUND),
        setBottomIntakeState(state.GROUND),setTopIntakeState(state.GROUND),
        waitUntil(this::getFrontIR),
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
    tab.add("Mech",mech);
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
    intakeElevator.periodic();
    mech.getRoot("root",0,0).setPosition(0, (intakeElevator.getMeasurement()/9)/(Math.PI*0.5));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
