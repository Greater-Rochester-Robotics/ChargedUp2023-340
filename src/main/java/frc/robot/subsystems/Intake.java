// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  private DoubleSolenoid solenoid;
  private final double MOTOR_SPEED = 0.5; //TODO: What should this be?

  /** Creates a new Intake. */
  public Intake(int motorId, int solenoidOutId, int solenoidInId) {
    motor = new CANSparkMax(motorId, MotorType.kBrushless);
    motor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);

    motor.setIdleMode(IdleMode.kBrake);

    //TODO: Make sure these are correct
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solenoidOutId, solenoidInId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeIn() {
    solenoid.set(Value.kReverse);
  }

  public void intakeOut() {
    solenoid.set(Value.kForward);
  }

  public void motorIn() {
    motor.set(-MOTOR_SPEED);
  }

  public void motorOut() {
    motor.set(MOTOR_SPEED);
  }

  public void motorStop() {
    motor.set(0.0);
  }

}
