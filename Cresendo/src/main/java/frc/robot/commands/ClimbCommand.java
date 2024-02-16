package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.Positions;


public class ClimbCommand extends SequentialCommandGroup {
  private final Climb m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param goDown suplier for going down
   */
  public ClimbCommand(Climb subsystem,BooleanSupplier goDown) {
    m_subsystem = subsystem;
    
      addRequirements(subsystem);
       addCommands(m_subsystem.unlockClimb(),
       new ParallelRaceGroup(new SequentialCommandGroup(m_subsystem.goToClimberPosition(Positions.TOP),Commands.run(()->{}).until(goDown)),
       Commands.run(()->{}).until(goDown) ),
       m_subsystem.goToClimberPosition(Positions.BOTTOM),m_subsystem.lockClimb());
  }

}
