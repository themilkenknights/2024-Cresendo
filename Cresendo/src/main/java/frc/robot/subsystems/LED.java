
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    // ! Creating the AddressableLED object as "led"
    // PWM port 2
    // Must be a PWM header, not MXP or DIO
    private AddressableLED led = new AddressableLED(2);
    // Making the buffer with length 60
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);

    private static enum LEDStates {
        IN, OUT, NEUTRAL, ALLINCE;
    }

    // the of the led to display
    private LEDStates led_state = LEDStates.NEUTRAL;

    /*
     * The Led states
     * 0:Neutral - orange, same state as robot signal light
     * 1:Taking piece in - green, color going Up
     * 2:Spitting piece out - red, color going Down
     */
    // ! ********Constructor********
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
        led_state = LEDStates.OUT;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 4 == 0) {
                ledBuffer.setRGB(i, 255, 0, 0);
            } else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    /**
     * Make the leds go green
     */
    public void set_go_intake() {
        led_state = LEDStates.IN;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 4 == 0) {
                ledBuffer.setRGB(i, 2, 222, 94);
            } else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    /**
     * Make the leds go neutral
     * Have go orange when robot signal light indicator is on.
     */
    public void set_go_neutral() {
        led_state = LEDStates.NEUTRAL;
    }

    // ! ****Code to be run during the robot loop****
    @Override
    public void periodic() {

        // * NEUTRAL STATE
        if (led_state == LEDStates.NEUTRAL) {
            //
            if (RobotController.getRSLState()) {
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 255, 106, 0);
                }
            } else {
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 0, 0, 0);
                }
            }
        } else if (led_state == LEDStates.IN) {
            Color last_color = ledBuffer.getLED(ledBuffer.getLength() - 1);
            for (int i = 59; i > 0; i--) {
                ledBuffer.setLED(i, ledBuffer.getLED(i - 1));
            }
            ledBuffer.setLED(0, last_color);
        } else if (led_state == LEDStates.OUT) {
            Color first_color = ledBuffer.getLED(0);
            for (int i = 0; i < 60; i++) {
                ledBuffer.setLED(i, ledBuffer.getLED(i - 1));
            }
            ledBuffer.setLED(ledBuffer.getLength() - 1, first_color);
        } else if (led_state == LEDStates.ALLINCE) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                for (int i = 0; i < 60; i++) {

                    ledBuffer.setLED(i, Color.kRed);
                }

            }else{
                for (int i = 0; i < 60; i++) {

                    ledBuffer.setLED(i, Color.kBlue);
                }
            }
        }
        led.setData(ledBuffer);
    }

    // TODO: Learn what it means by simulation
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}