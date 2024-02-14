package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.Intakes.state;

public class Climb extends ProfiledPIDSubsystem {
    private static final double spoolsize = 1 * Math.PI;
    private static final double reduction = 15.34;

    public static enum Positions {
        TOP, BOTTOM
    }

    private static double inchestorotations(double inches) {
        return spoolsize * inches * reduction;
    }

    private TalonFX climber = new TalonFX(Constants.ClimbCANID);
    private TalonFXSimState motorSim =climber.getSimState();
    //private final CANcoder sensor = new CANcoder(Constants.ClimbCANID);
   // private final CANcoderSimState sensorSim = sensor.getSimState();

    private final ElevatorSim m_elevatorSim = new ElevatorSim(0.4, 0.2,
            DCMotor.getFalcon500(1).withReduction(reduction), -3, 3, true, 0);

    private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(1.1, 0.14, 2.83, 0.01);
    private Servo locker = new Servo(Constants.ClimbServoPORT);

    public Climb() {
        super(new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.55, 0.55)));
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State state) {
        climber.setVoltage(output + m_feedforward.calculate(state.velocity));
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
                    setGoal(inchestorotations(20));
                    break;
                case BOTTOM:
                    setGoal(0);
                    break;

            }
        }),
                run(() -> {
                }).until(this::atSetpoint));
    }

    public Command lockClimb() {
        return runOnce(() -> {
            disable();
            locker.set(0.6);
        });
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

            locker.set(0.4);
            enable();
        });
    }

}
