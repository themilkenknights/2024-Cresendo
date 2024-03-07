package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Climb extends PIDSubsystem {
    private Mechanism2d mech = new Mechanism2d(1, 2);
    private MechanismLigament2d rooLigament2d = mech.getRoot("root", 0.4, 0.1)
            .append(new MechanismLigament2d("climb", 160, 90));
    private static final double spoolsize = 1 * Math.PI;
    private static final double reduction = 15.34;

    public static enum Positions {
        TOP, BOTTOM
    }

    private static double inchestorotations(double inches) {
        return spoolsize * inches * reduction;
    }

    // private final TalonFXConfigurator config = new TalonFXConfigurator(climb);
    private TalonFX climber = new TalonFX(Constants.ClimbCANID, "rio");
    private TalonFXSimState motorSim = climber.getSimState();
    // private final CANcoder sensor = new CANcoder(Constants.ClimbCANID);
    // private final CANcoderSimState sensorSim = sensor.getSimState();

    private final ElevatorSim m_elevatorSim = new ElevatorSim(0.02, 0.8,
            DCMotor.getFalcon500(1).withReduction(reduction), 0, 80, true, 0);

    // private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(1.1,
    // 0.14, 2.83, 0.01);
    private Servo locker = new Servo(Constants.ClimbServoPORT);

    public Climb() {
        super(new PIDController(0.9, 0, 0.1));
        m_controller.setTolerance(5);
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
        return climber.getPosition().getValueAsDouble();
    }
   
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(climber.getMotorVoltage().getValue());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.02);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        // sensorSim.setRawPosition(m_elevatorSim.getPositionMeters());
        motorSim.setRawRotorPosition(m_elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
        rooLigament2d.setLength((m_elevatorSim.getPositionMeters() / 80) + 0.5287690974);

    }

    public Command goToClimberPosition(Positions state) {

        return new SequentialCommandGroup(runOnce(() -> {
            switch (state) {
                case TOP:
                    setSetpoint(80);// inchestorotations(20));
                    locker.set(0.3);
                    break;
                case BOTTOM:
                    setSetpoint(1);
                    break;

            }
        }),
                waitUntil(this::atSetpoint));
    }

    public Command lockClimb() {
        return runOnce(() -> {
            // disable();
            locker.set(0.6);
        });
    }

    /*
     * public Command manualDown(DoubleSupplier sup) {
     * return runOnce(() -> {
     * climber.set(-sup.getAsDouble());
     * });
     * }
     * 
     * public Command manualUp(DoubleSupplier sup) {
     * return runOnce(() -> {
     * climber.set(sup.getAsDouble());
     * });
     * }
     */

    public Command AutoZero() {
        final ParallelCommandGroup zCommand = new ParallelCommandGroup(new InstantCommand(() -> disable()),
                new InstantCommand(() -> {

                    climber.set(0.025);

                }), waitUntil(this::isHighCurrent), runOnce(() -> {
                    climber.set(0);
                    climber.setPosition(-5);

                }), new InstantCommand(() -> disable()));
        zCommand.addRequirements(this);
        return zCommand;
    }

    private boolean isHighCurrent() {
        if ((climber.getTorqueCurrent().asSupplier().get()) > 5) {
            return true;
        } else {
            return false;
        }
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
        Shuffleboard.getTab("Climb")
                .add("Motor", climber);
        Shuffleboard.getTab("Climb")
                .add("Climb", mech);

    }

    public Command unlockClimb() {
        return new ParallelCommandGroup(runOnce(() -> {

            locker.set(0.3);
            // enable();
        }), waitSeconds(0.1));
    }

}
