package frc.lib.DynamicAutoLib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class DynamicAuto {
    ArrayList<AutoTask> objectives = new ArrayList<>();

    public DynamicAuto(ArrayList<AutoTask> objectives) {
        this.objectives = objectives;
    }
    public Command getAutoCommand(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        for (AutoTask autoTask : objectives) {
            command.addCommands(autoTask.getCommand().until(autoTask::isFinished));
        }

        return command;
    }
}
