
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    // ! Creating the AddressableLED object as "m_led"
    // PWM port 2
    // Must be a PWM header, not MXP or DIO
    private AddressableLED led = new AddressableLED(2);
    // Making the buffer with length 60
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);


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
     * @return a command
     */
    public Command set_go_red() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
            });
    }
    /**
     * Make the leds go green
     *
     * @return a command
     */
    public Command set_go_green() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
            });
    }
    /**
     * Make the leds go neutral
     * Have go orange when robot signal light indicator is on. 
     * @return a command
     */
    public Command set_go_neutral() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
            });
    }

    //! ****Code to be run during the robot loop****
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // TODO: Learn what it means by simulation
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}