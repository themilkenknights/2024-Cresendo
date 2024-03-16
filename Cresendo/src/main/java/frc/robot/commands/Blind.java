package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class Blind extends Command{
    DoubleSupplier heading;
    public Blind(DoubleSupplier heading){
        this.heading = heading;
    }
    @Override
    public void execute() {
        if (!(heading.getAsDouble()>80)&!(heading.getAsDouble()<260)){
        LimelightHelpers.setLEDMode_ForceBlink("limelight-knights");
        }else{
            LimelightHelpers.setLEDMode_ForceOff("limelight-knights");
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
