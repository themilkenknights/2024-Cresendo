package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class Intakes extends SubsystemBase {


    public static enum state{
      ON,SLOW,OFF
    }

    private final IntakeElevator intakeElevator;
    private TalonFX topIntake = new TalonFX(Constants.topIntakeCANID);
    private TalonFX bottomIntake = new TalonFX(Constants.bottomIntakeIntakeCANID);
    private DigitalInput beambreak = new DigitalInput(Constants.beambreakPORT);

  /** Setup the Intakes. */
  public Intakes() {
      intakeElevator=new IntakeElevator();
  }
  public Command setTopIntakeState(Intakes.state intakeState){
      
    if(intakeState == state.ON){
      return runOnce(()->topIntake.set(1));
    }else{
      return runOnce(()->topIntake.set(0));
    }
  }

    public Command setBottomIntakeState(Intakes.state intakeState){
      
    if(intakeState == state.ON){
      return runOnce(()->bottomIntake.set(1));
    }else{
      return runOnce(()->bottomIntake.set(0));
    }
  }

  public boolean getBeambreak(){
    return beambreak.get();
  }
  public Command GroundPickUP(){

    return new SequentialCommandGroup(intakeElevator.setHeight(IntakeElevator.Positions.GROUND),
    setBottomIntakeState(state.ON),
    Commands.waitSeconds(3),
    setBottomIntakeState(state.OFF));
    //TODO: integrate sensor(some code writen in comment bellow, trigger still needs to be implemented/added)
  

    /*return new SequentialCommandGroup(intakeElevator.setHeight(IntakeElevator.Positions.GROUND),
    setBottomIntakeState(state.ON),
    Commands.waitUntil(this::getBeambreak),
    setBottomIntakeState(state.OFF));

    */
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
