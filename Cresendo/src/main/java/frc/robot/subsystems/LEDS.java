package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LEDS extends SubsystemBase {
    BooleanSupplier beambreak;

    LEDPattern beambreakPattern = null;

    LEDPattern green = LEDPattern.gradient(Color.kGreen,Color.kLightGreen,Color.kGreen,Color.kLightGreen).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(15));
    LEDPattern red = LEDPattern.gradient(Color.kRed,Color.kDarkRed,Color.kRed,Color.kHotPink).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(15));
    LEDPattern orange = LEDPattern.gradient(Color.kOrange,Color.kDarkOrange,Color.kCoral).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(5)).blink(Units.Seconds.of(0.4));
    LEDPattern blue = LEDPattern.gradient(Color.kBlue,Color.kAliceBlue,Color.kAqua,Color.kAzure,Color.kSkyBlue).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(5)).blink(Units.Seconds.of(0.4));
    
    @Override
    public void periodic() {
        
        if(beambreak.getAsBoolean()){
            beambreakPattern = green;
        }else{
            beambreakPattern = red;
        }
        if(RobotState.isDisabled()){
            beambreakPattern = beambreakPattern.atBrightness(Units.Percent.of(30));
        }else{
            beambreakPattern = beambreakPattern.atBrightness(Units.Percent.of(100));
        }

        if(ledState=="orange"){
            orange.overlayOn(beambreakPattern).applyTo(ledBuffer);
            
        }else if(ledState=="blue"){
            blue.overlayOn(beambreakPattern).applyTo(ledBuffer);
        }else{
            beambreakPattern.applyTo(ledBuffer);
        }
          leds.setData(ledBuffer);
    }
    public LEDS(BooleanSupplier beambreak) {
        this.beambreak = beambreak;

        // leds
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);

        leds.start();
    }
  public Command FlashOrange(){
   return  runEnd(()->    ledState="orange",()->ledState="default");
  }

  public Command FlashBlue(){
    return  runEnd(()->    ledState = "blue",()->ledState="default");
  }
    // leds
    private AddressableLED leds = new AddressableLED(Constants.ledPORT);
    // Making the buffer with length 60
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(90);
    AddressableLEDBufferView leftLED = new AddressableLEDBufferView(ledBuffer, 0, 20);
    AddressableLEDBufferView topLED = new AddressableLEDBufferView(ledBuffer, 21, 40);
    AddressableLEDBufferView rightLED = new AddressableLEDBufferView(ledBuffer, 41, 60);

    AddressableLEDSim ledSim = new AddressableLEDSim(leds);

    String ledState = "default";
}
