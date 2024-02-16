package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Climb extends PIDSubsystem {
    private static final double spoolsize = 1 * Math.PI;
    private static final double reduction = 15.34;
    private double encoderOffset = 0;
    public static enum Positions {
        TOP, BOTTOM
    }

    private static double inchestorotations(double inches) {
        return spoolsize * inches * reduction;
    }
    //private final TalonFXConfigurator config = new TalonFXConfigurator(climb);
    private TalonFX climber = new TalonFX(Constants.ClimbCANID,"rio");
    
    private TalonFXSimState motorSim =climber.getSimState();
    //private final CANcoder sensor = new CANcoder(Constants.ClimbCANID);
   // private final CANcoderSimState sensorSim = sensor.getSimState();

    private final ElevatorSim m_elevatorSim = new ElevatorSim(0.4, 0.2,
            DCMotor.getFalcon500(1).withReduction(reduction), -3, 3, true, 0);

    private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(1.1, 0.14, 2.83, 0.01);
    private Servo locker = new Servo(Constants.ClimbServoPORT);

    public Climb() {
        super(new PIDController(3, 0, 1));
        climber.setPosition(0);
        locker.set(0.3);
        enable();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        climber.setVoltage(output);// + m_feedforward.calculate(state.velocity));
    }

    @Override
    public double getMeasurement() {
        return climber.getPosition().getValueAsDouble()+encoderOffset;
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput((climber.getMotorVoltage().getValueAsDouble()));

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        //sensorSim.setRawPosition(m_elevatorSim.getPositionMeters());
        motorSim.addRotorPosition(m_elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    public Command goToClimberPosition(Positions state) {

        return new SequentialCommandGroup(runOnce(() -> {
            switch (state) {
                case TOP:
                    setSetpoint(80);//inchestorotations(20));
                    locker.set(0.3);
                    break;
                case BOTTOM:
                    setSetpoint(0);
                    break;

            }
        }),
                run(() -> {
                }).until(this::atSetpoint));
    }

    public Command lockClimb() {
        return runOnce(() -> {
            //disable();
            locker.set(0.6);
        });
    }
    public Command manualDown(DoubleSupplier sup){
        return runOnce(()->{climber.set(-sup.getAsDouble());});
    }

    public Command manualUp(DoubleSupplier sup){
        return runOnce(()->{climber.set(sup.getAsDouble());});
    }
    public void initSendable(SendableBuilder builder) {
        Shuffleboard.getTab("Climb")
                .add(m_controller);
        Shuffleboard.getTab("Climb")
                .add(locker);
        Shuffleboard.getTab("Climb")
                .add("lock", lockClimb());

        Shuffleboard.getTab("Climb")
                .add("unlock", unlockClimb());
        Shuffleboard.getTab("Climb")
                .add("top", goToClimberPosition(Positions.TOP));
        Shuffleboard.getTab("Climb")
                .add("bottom", goToClimberPosition(Positions.BOTTOM));

    }

    public Command unlockClimb() {
        return runOnce(() -> {

            locker.set(0.3);
            //enable();
        });
    }

}
