import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.IntakeElevator;

public class test {

    @Test
    public void test(){

        assertEquals(IntakeElevator.inchestorotationsOld( 3.35), IntakeElevator.inchestorotationsNew( 3.35));
    }
}
