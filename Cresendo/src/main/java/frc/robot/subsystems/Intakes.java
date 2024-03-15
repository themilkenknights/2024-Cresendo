package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeElevator.Positions;

public class Intakes extends SubsystemBase {

  // leds
  private AddressableLED leds = new AddressableLED(Constants.ledPORT);
  // Making the buffer with length 60
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(90);
  private boolean ledOveride = false;
  String ledState = "None";
  

  public static enum state {
    GROUND, OUT, IN, INFORFUCKSSAKE, INHP, HP, GROUNDOUT, OFF
  }

  ShuffleboardTab tab = Shuffleboard.getTab("intakes");
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

    // leds
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);

    leds.start();
  }

  private final double elevatorspeed = .6;
  private final double groundspeed = 1;
  private final double waittime = 0.055;
  private final double waittimeGround = 0.1;

  public Command Flashorange(){
    return new RepeatCommand(new SequentialCommandGroup(new RunCommand(()->{
      ledOveride=true;
      ledState = "orange";
    }).withTimeout(0.25),waitSeconds(0.25)));
  }

  public Command FlashBlue(){
    return new RepeatCommand(new SequentialCommandGroup(new RunCommand(()->{
      ledOveride=true;
      ledState = "BLUE";
    }).withTimeout(0.25),waitSeconds(0.25)));
  }


  public Command setTopIntakeState(Intakes.state intakeState) {

    if (intakeState == state.GROUND) {
      return runOnce(() -> {
        topIntake.set(elevatorspeed);
        midIntake.set(elevatorspeed);
      });
    } else if (intakeState == state.OUT) {
      return runOnce(() -> {
        topIntake.set(-elevatorspeed);
        midIntake.set(elevatorspeed);
      });
    } else if (intakeState == state.IN) {
      return runOnce(() -> {
        topIntake.set(-elevatorspeed);
        midIntake.set(-elevatorspeed);
      });
    } else if (intakeState == state.INFORFUCKSSAKE) {
      return runOnce(() -> {
        topIntake.set(.1);
        midIntake.set(-.1);
      });

    } else if (intakeState == state.INHP) {
      return runOnce(() -> {
        topIntake.set(elevatorspeed);
        midIntake.set(-elevatorspeed);
      });

    } else if (intakeState == state.HP) {
      return runOnce(() -> {
        topIntake.set(elevatorspeed);
        midIntake.set(elevatorspeed);
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
      return runOnce(() -> bottomIntake.set(groundspeed));
    } else if (intakeState == state.GROUNDOUT) {
      return runOnce(() -> {
        bottomIntake.set(-groundspeed);
      });
    } else {
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
    return new SequentialCommandGroup(setTopIntakeState(state.INHP), setBottomIntakeState(state.OFF),
        TopIntakeByBeambreak());
  
  }

  public Command AutoHPin() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(Positions.GROUND),setTopIntakeState(state.INHP), setBottomIntakeState(state.OFF),
        TopIntakeByBeambreak());
  
  }

  public Command TopIntakeByBeambreak() {
    return new SequentialCommandGroup(setTopIntakeState(state.INHP), waitUntil(this::getFrontIR), waitSeconds(waittime),
        setTopIntakeState(state.OFF));
  }

  public Command TopOutakeByBeambreak() {
    return new SequentialCommandGroup(setTopIntakeState(state.OUT), waitUntil(this::getNotFrontIR),
        waitSeconds(waittime),
        setTopIntakeState(state.OFF));
  }

  public Command AmpOuttake() {
    return new SequentialCommandGroup(setTopIntakeState(state.OUT), setBottomIntakeState(state.OFF),
        TopOutakeByBeambreak(), waitSeconds(0.1))
        .withName("Amp Outtake");
  }

  public Command AutoAmpOuttake() {
    return new SequentialCommandGroup(intakeElevator.gotoHeight(Positions.AMP),
        TopOutakeByBeambreak(),intakeElevator.STOW())
        .withName("Amp Outtake");
  }

  public Command goUp() {
    return new ParallelCommandGroup(setTopIntakeState(state.OFF), setBottomIntakeState(state.OFF),
        intakeElevator.gotoHeight(IntakeElevator.Positions.HP));
  }

  public Command GoDown() {
    return new ParallelCommandGroup(setTopIntakeState(state.OFF), setBottomIntakeState(state.OFF),
        intakeElevator.gotoHeight(IntakeElevator.Positions.GROUND));
  }

  public IntakeElevator getElevator() {
    return this.intakeElevator;
  }

  public Command AutoGroundPickUP() {
    return new SequentialCommandGroup(setTopIntakeState(state.OFF), setBottomIntakeState(state.OFF),
        intakeElevator.gotoHeight(IntakeElevator.Positions.GROUND),
        setBottomIntakeState(state.GROUND), setTopIntakeState(state.GROUND),
        waitUntil(this::getFrontIR), waitSeconds(waittimeGround),
        setBottomIntakeState(state.OFF)).withName("GroundPickup");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    ShuffleboardTab tab = Shuffleboard.getTab("intakes");
    // tab.add("ElevatorPID", intakeElevator.getController());
    // tab.add("Elevator", intakeElevator);
    tab.add(frontIR);
    tab.add("Mech", mech);
    super.initSendable(builder);
    intakeElevator.initSendable(builder);
    tab.addString("LED", () -> ledState);
    tab.add("TopIntake", topIntake);
    tab.add("other", midIntake);
    tab.add("BottomIntake", bottomIntake);

    ShuffleboardLayout commandlLayout = tab.getLayout("IntakeCommands", BuiltInLayouts.kGrid);
    commandlLayout.withSize(2, 4);
    commandlLayout.add("out", TopOutakeByBeambreak());
    commandlLayout.add("in", TopIntakeByBeambreak());
    commandlLayout.add("Ground", AutoGroundPickUP());
    commandlLayout.add("HP", HPin());

  }

  public void onReEnable() {
    setBottomIntakeState(state.OFF).schedule();
    setTopIntakeState(state.OFF).schedule();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intakeElevator.periodic();

    stage.setLength((intakeElevator.getMeasurement() / Math.PI) / 40);// +0.3);

    if (!ledOveride) {
      if (!getFrontIR()) {

        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red

          ledBuffer.setRGB(i, 255, 0, 0);
          ledState = "RED";
        }
      } else {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          ledBuffer.setRGB(i, 0, 255, 0);
          ledState = "GREEN";
        }

      }
      
    }else if (ledState == "orange"){
      ledOveride = false;
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 165, 0);
      }
    }else{
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 0, 0, 255);
        ledOveride = false;
      }
    }

    leds.setData(ledBuffer);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
