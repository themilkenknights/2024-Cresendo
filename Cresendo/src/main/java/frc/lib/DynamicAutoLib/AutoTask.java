package frc.lib.DynamicAutoLib;

import edu.wpi.first.wpilibj2.command.Command;


/** Add your docs here. */
public abstract class AutoTask {
    // public static enum taskType{
    //     PICKUP,SCORE,GAMETASK
    // }
    public abstract Command getCommand();

    public abstract boolean isFinished();







}
