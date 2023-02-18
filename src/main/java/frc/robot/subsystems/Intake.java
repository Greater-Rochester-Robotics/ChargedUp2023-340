// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonSRX motor1;
  // private CANSparkMax motor2;
  private DigitalInput gamePieceSensor;
  private DoubleSolenoid solenoid;
  private final double MOTOR_SPEED = 0.5; //TODO: What should this be?

  /** Creates a new Intake. */
  public Intake() {
    gamePieceSensor = new DigitalInput(Constants.GAME_PEICE_SENSOR_ID);

    motor1 = new TalonSRX(Constants.INTAKE_MOTOR_1);
    // motor2 = new CANSparkMax(Constants.INTAKE_MOTOR_2, MotorType.kBrushless);
    motor1.enableVoltageCompensation(true);
    // motor2.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);

    // motor1.setNeutralMode(NeutralMode.);
    // motor2.setIdleMode(IdleMode.kBrake);

    //TODO: Make sure these are correct
    // motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    // motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    // motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    // motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    // motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    // motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    // motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    // motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    // motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    // motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    // motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    // motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    // gamePieceSensor = new DigitalInput();

    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_OUT, Constants.INTAKE_SOLENOID_IN);
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
    // motor1.set(-MOTOR_SPEED);
    // motor2.set(-MOTOR_SPEED);
  }

  public void motorOut() {
    // motor1.set(MOTOR_SPEED);
    // motor2.set(MOTOR_SPEED);
  }

  public void motorStop() {
    // motor1.set(0.0);
    // motor2.set(0.0);
  }

}
