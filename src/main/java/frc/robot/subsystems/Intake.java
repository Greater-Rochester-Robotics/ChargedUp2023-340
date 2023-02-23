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
  private TalonSRX intakeMotor;
  // private CANSparkMax motor2;
  private DigitalInput gamePieceSensor;
  private DoubleSolenoid intakePistons;
  private final double MOTOR_SPEED = 0.5; //TODO: What should this be?

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR);
    intakeMotor.enableVoltageCompensation(true);//TODO: set the max voltage on this motor
    
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    //TODO: we don't need most of these, set to different values close to 255 for uneeded, 255 is max option
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 2000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 2000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 20);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 20);
    
    gamePieceSensor = new DigitalInput(Constants.INTAKE_GAME_PIECE_SENSOR);

    intakePistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_OUT, Constants.INTAKE_SOLENOID_IN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Move the intake into the robot.
   */
  public void intakeIn() {
    intakePistons.set(Value.kReverse);
  }

  /**
   * moves the intake out of the robot
   */
  public void intakeOut() {
    intakePistons.set(Value.kForward);
  }

  /**
   * run the intake motor, to bring a gamepiece into the robot
   */
  public void motorIn() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -MOTOR_SPEED);
    // motor1.set(-MOTOR_SPEED);
    // motor2.set(-MOTOR_SPEED);
  }

  /**
   * runs the intake motor, to push a gamepiece out of the robot
   */
  public void motorOut() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, MOTOR_SPEED);
    // motor1.set(MOTOR_SPEED);
    // motor2.set(MOTOR_SPEED);
  }

  /**
   * stops the intake motor
   */
  public void motorStop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    // motor1.set(0.0);
    // motor2.set(0.0);
  }

  /**
   * a sensor in the intake to determine if there is a gamepiece in the intake.
   * @return boolean
   */
  public boolean hasGamePeice(){
    return gamePieceSensor.get();
  }

}
