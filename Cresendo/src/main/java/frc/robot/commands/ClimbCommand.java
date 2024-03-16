package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.Positions;

public class ClimbCommand extends SequentialCommandGroup {
  private final Climb m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param goDown    suplier for going down
   */
  public ClimbCommand(Climb subsystem, Boolean goDown) {
    m_subsystem = subsystem;

    addRequirements(subsystem);
    if (!goDown) {
      addCommands(m_subsystem.unlockClimb(), m_subsystem.goToClimberPosition(Positions.TOP));
    } else {
      addCommands(m_subsystem.goToClimberPosition(Positions.BOTTOM), m_subsystem.lockClimb());
    }
  }

}
