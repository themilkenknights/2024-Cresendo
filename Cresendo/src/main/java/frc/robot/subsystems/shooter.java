// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class shooter extends SubsystemBase {
  CANSparkMax motor1 = new CANSparkMax(100,MotorType.kBrushed);//TODO: canid
  CANSparkMax motor2 = new CANSparkMax(101,MotorType.kBrushed);//TODO: canid
  double targetV = 0;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1);
  /** Creates a new shooter. */
  public shooter() {
    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();

    motor1.setIdleMode(IdleMode.kCoast);
    motor2.setIdleMode(IdleMode.kCoast);


    motor2.follow(motor1);
  }
  public Command shoot(){
    return new SequentialCommandGroup(runOnce(()->{targetV = 1200;}),waitSeconds(1),runOnce(()->{targetV=0;}));
  }
  @Override
  public void periodic() {
    motor1.set(feedforward.calculate(targetV));
    // This method will be called once per scheduler run
  }
}
