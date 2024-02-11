package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
@SuppressWarnings("unused")
public class Intakes extends SubsystemBase {
    private final IntakeElevator intakeElevator;
    private TalonFX topIntake = new TalonFX(Constants.topIntakeCANID);
    private TalonFX bottomIntake = new TalonFX(Constants.bottomIntakeIntakeCANID);


  /** Setup the Intakes. */
  public Intakes() {
      intakeElevator=new IntakeElevator();
  }
  public Command setTopIntakeState(boolean state){
    if(state==true){
      return runOnce(()->topIntake.set(1));
    }else{
      return runOnce(()->topIntake.set(0));
    }
  }

    public Command setBottomIntakeState(boolean state){
    if(state==true){
      return runOnce(()->bottomIntake.set(1));
    }else{
      return runOnce(()->bottomIntake.set(0));
    }
  }

  public Command GroundPickUP(){
    return new SequentialCommandGroup(intakeElevator.setHeight(IntakeElevator.Positions.GROUND),setBottomIntakeState(true),Commands.waitSeconds(3),setBottomIntakeState(false));
    //TODO: integrate sensor
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
