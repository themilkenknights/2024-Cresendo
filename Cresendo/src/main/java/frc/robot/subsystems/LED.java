
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intakes.state;

public class LED extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    // ! Creating the AddressableLED object as "m_led"
    // PWM port 2
    // Must be a PWM header, not MXP or DIO
    private AddressableLED led = new AddressableLED(2);
    // Making the buffer with length 60
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
    private static enum LEDStates{
        IN,OUT,NEUTRAL;
    }
    // the of the led to display
    private LEDStates led_state = LEDStates.NEUTRAL;

    /* The Led states
     * 0:Neutral - orange, same state as robot signal light
     * 1:Taking piece in - green, color going down
     * 2:Spitting piece out - red, color going up
     */
    //! ********Constructor********
    public LED() {
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }
    
    /**
     * Make the leds go red
     *
     */
    public void set_go_outtake() {
        led_state=LEDStates.OUT;
    }
    /**
     * Make the leds go green
     */
    public void set_go_intake() {
        led_state=LEDStates.IN;
    }
    /**
     * Make the leds go neutral
     * Have go orange when robot signal light indicator is on. 
     */
    public void set_go_neutral() {
        led_state=LEDStates.NEUTRAL;
    }

    //! ****Code to be run during the robot loop****
    @Override
    public void periodic() {
        
    }

    // TODO: Learn what it means by simulation
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}