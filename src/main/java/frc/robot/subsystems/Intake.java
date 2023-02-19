// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonSRX clawWheelsMotor1;
  // private CANSparkMax motor2;
  private DigitalInput gamePieceSensor;
  private DoubleSolenoid solenoid;
  private final double MOTOR_SPEED = 0.5; //TODO: What should this be?

  /** Creates a new Intake. */
  public Intake() {
    gamePieceSensor = new DigitalInput(Constants.GAME_PEICE_SENSOR_ID);

    clawWheelsMotor1 = new TalonSRX(Constants.INTAKE_MOTOR_1);
    // motor2 = new CANSparkMax(Constants.INTAKE_MOTOR_2, MotorType.kBrushless);
    clawWheelsMotor1.enableVoltageCompensation(true);
    // motor2.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);

    clawWheelsMotor1.setNeutralMode(NeutralMode.Brake);
    // motor2.setIdleMode(IdleMode.kBrake);

    //TODO: Make sure these are correct !!!!!
    clawWheelsMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    clawWheelsMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    clawWheelsMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20);
    clawWheelsMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 2000);
    clawWheelsMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 2000);
    clawWheelsMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 20);
    clawWheelsMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 20);

    // motor2.setStatusFramePeriod(StatusFrameEnhanced.kStatus0, 2000);
    // motor2.setStatusFramePeriod(StatusFrameEnhanced.kStatus1, 20);
    // motor2.setStatusFramePeriod(StatusFrameEnhanced.kStatus2, 20);
    // motor2.setStatusFramePeriod(StatusFrameEnhanced.kStatus3, 2000);
    // motor2.setStatusFramePeriod(StatusFrameEnhanced.kStatus4, 2000);
    // motor2.setStatusFramePeriod(StatusFrameEnhanced.kStatus5, 20);
    // motor2.setStatusFramePeriod(StatusFrameEnhanced.kStatus6, 20);

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
    clawWheelsMotor1.set(TalonSRXControlMode.PercentOutput, -MOTOR_SPEED);
    // motor1.set(-MOTOR_SPEED);
    // motor2.set(-MOTOR_SPEED);
  }

  public void motorOut() {
    clawWheelsMotor1.set(TalonSRXControlMode.PercentOutput, MOTOR_SPEED);
    // motor1.set(MOTOR_SPEED);
    // motor2.set(MOTOR_SPEED);
  }

  public void motorStop() {
    clawWheelsMotor1.set(TalonSRXControlMode.PercentOutput, 0.0);
    // motor1.set(0.0);
    // motor2.set(0.0);
  }

  public boolean hasGamePeice(){
    return gamePieceSensor.get();
  }

}
