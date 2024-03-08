package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeElevator.Positions;

public class Intakes extends SubsystemBase {

  public static enum state {
    GROUND, HP, OUT, OFF, GROUNDOUT
  }

  private Mechanism2d mech = new Mechanism2d(18 / 39.37, 30 / 39.37);// ,new Color8Bit(Color.kBlueViolet));
  private MechanismLigament2d stage = mech.getRoot("Elevator", 18 / 39.37, 0.1)
      .append(new MechanismLigament2d("Stage", 10 / 39.37, 90));
  private final IntakeElevator intakeElevator;
  private TalonFX topIntake = new TalonFX(Constants.topIntakeCANID);
  private TalonFX midIntake = new TalonFX(Constants.midIntakeCANID);
  private TalonFX bottomIntake = new TalonFX(Constants.bottomIntakeIntakeCANID);
  private DigitalInput frontIR = new DigitalInput(Constants.frontIRPORT);

  /** Setup the Intakes. */
  public Intakes(IntakeElevator elevator) {
    this.intakeElevator = elevator;
    final ParallelCommandGroup defultCommandGroup = new ParallelCommandGroup(setBottomIntakeState(state.OFF),
        setTopIntakeState(state.OFF));
    defultCommandGroup.addRequirements(this);
    setDefaultCommand(defultCommandGroup);
    stage.setColor(new Color8Bit(Color.kSilver));
    stage.append(new MechanismLigament2d("Intake", -0.31, 90));
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
    } else if (intakeState == state.HP) {
      return runOnce(() -> {
        topIntake.set(1);
        midIntake.set(1);
      });
    } else {
      return new InstantCommand(() -> {
        topIntake.set(0);
        midIntake.set(0);
      });
    }
  }

  public Command setBottomIntakeState(Intakes.state intakeState) {

    if (intakeState == state.GROUND) {
      return runOnce(() -> bottomIntake.set(1));
    }
else if (intakeState == state.GROUNDOUT) {
      return runOnce(() -> {
        bottomIntake.set(-1);
      });}
else {
      return new InstantCommand(() -> bottomIntake.set(0));
    }
  }

  public boolean getFrontIR() {
    return !(frontIR.get());
  }

  public boolean getNotFrontIR() {
    return (frontIR.get());
  }

  public Command HPin() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(Positions.HP), TopIntakeByBeambreak(),
        intakeElevator.gotoHeight(Positions.GROUND)).withName("HP");
  }

  public Command TopIntakeByBeambreak() {
    return new SequentialCommandGroup(setTopIntakeState(state.HP), waitUntil(this::getFrontIR),waitSeconds(1),//TODO: tune time
        setTopIntakeState(state.OFF));
  }

  public Command TopOutakeByBeambreak() {
    return new SequentialCommandGroup(setTopIntakeState(state.OUT), waitUntil(this::getNotFrontIR), waitSeconds(0.25),//TODO: tune time
        setTopIntakeState(state.OFF));
  }

  public Command AmpOuttake() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.AMP), TopIntakeByBeambreak())
        .withName("Amp Outtake");
  }

  public Command goUp() {
    return new ParallelCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.AMP));
  }

  public IntakeElevator getElevator() {
    return this.intakeElevator;
  }

  public Command GoDown() {
    return new ParallelCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.GROUND));
  }

  public Command GroundPickUP() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(IntakeElevator.Positions.GROUND),
        setBottomIntakeState(state.GROUND), setTopIntakeState(state.GROUND),
        waitUntil(this::getFrontIR),waitSeconds(1),
        setBottomIntakeState(state.OFF)).withName("GroundPickup");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    ShuffleboardTab tab = Shuffleboard.getTab("intakes");
    //tab.add("ElevatorPID", intakeElevator.getController());
    //tab.add("Elevator", intakeElevator);
    tab.add(frontIR);
    tab.add("Mech", mech);
    super.initSendable(builder);
    intakeElevator.initSendable(builder);

    tab.add("TopIntake", topIntake);
    tab.add("other", midIntake);
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
   //intakeElevator.periodic();
    stage.setLength((((intakeElevator.getMeasurement() / 18) / (Math.PI * 0.5)) / 40));// +0.3);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
