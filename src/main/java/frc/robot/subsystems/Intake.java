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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor;
  // private CANSparkMax motor2;
  private DigitalInput gamePieceSensor;
  private DoubleSolenoid intakePistons;

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR);
    intakeMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    intakeMotor.enableVoltageCompensation(true);
    
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    
    gamePieceSensor = new DigitalInput(Constants.INTAKE_GAME_PIECE_SENSOR);

    intakePistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_OUT, Constants.INTAKE_SOLENOID_IN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake has game peice: ", hasGamePiece());
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
    intakeMotor.set(TalonSRXControlMode.PercentOutput, INTAKE_MOTOR_INTAKE_SPEED);
  }

  /**
   * runs the intake motor, to push a gamepiece out of the robot
   */
  public void motorOut() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, INTAKE_MOTOR_OUTTAKE_SPEED);
  }

  /**
   * stops the intake motor
   */
  public void motorStop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /**
   * a sensor in the intake to determine if there is a gamepiece in the intake.
   * @return boolean
   */
  public boolean hasGamePiece(){
    return !gamePieceSensor.get();
  }

}
