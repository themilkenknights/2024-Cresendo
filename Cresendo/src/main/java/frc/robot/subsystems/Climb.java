package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Climb extends ProfiledPIDSubsystem {
    public static enum Positions {
        TOP, BOTTOM
    }

    private TalonFX climber = new TalonFX(Constants.ClimbCANID);
    private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(1.1, 0.14, 2.83, 0.01);
    private Servo locker = new Servo(Constants.ClimbServoPORT);

    public Climb() {
        super(new ProfiledPIDController(0.1, 0, 0,new TrapezoidProfile.Constraints(0.55,0.55)));
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

    public Command goToClimberPosition(Positions state) {

        return new SequentialCommandGroup(runOnce(() -> {
            switch (state) {
                case TOP:
                    setGoal(10);// TODO: set Properly
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
            locker.set(0.8);
        });
    }

    public Command unlockClimb() {
        return runOnce(() -> {
            locker.set(0.2);
        });
    }
    


}
