// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private TalonSRX clawMotor;
  private DoubleSolenoid openClose;
  private DigitalInput gamePieceSensor;

  /**
   * The network table instance used by the claw subsystem.
   */
  private NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
  /**
   * The network table used by the claw subsystem.
   */
  private NetworkTable netTable = netInstance.getTable("/dashboard/robotmodel");

  /** Creates a new Claw. */
  public Claw() {
    //Constructs the claws motors and solenoid
    clawMotor = new TalonSRX(Constants.CLAW_MOTOR);
    clawMotor.enableVoltageCompensation(true);

    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 251);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 254);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 237);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 221);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 211);

    openClose = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLAW_SOLENOID_CLOSED, Constants.CLAW_SOLENOID_OPEN); //open is in is reverse, close is out is forward

    gamePieceSensor = new DigitalInput(Constants.CLAW_GAMEPIECE_SENSOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Claw GamePiece", getGamePieceSensor());
    netTable.getEntry("claw").setBoolean(isOpen());
  }

  /**
   * Closes the claw pneumatic
   */
  public void close(){
    openClose.set(Value.kForward);
  }

  /**
   * Opens the claw pneumatic
   */
  public void open(){
    openClose.set(Value.kReverse);
  }

  public boolean isOpen(){
    return openClose.get() == Value.kReverse;
  }

  /**
   * causes the claw motor to intake a cube or cone
   */
  public void intake(){
    clawMotor.set(ControlMode.PercentOutput, ClawConstants.CLAW_MOTOR_INTAKE_SPEED);
  }

  /**
   * cause the intake motor to spit out
   */
  public void outtake(){
    clawMotor.set(TalonSRXControlMode.PercentOutput, ClawConstants.CLAW_MOTOR_OUTTAKE_SPEED);
  }

  /**
   * Stops claw motor
   */
  public void stop(){
    clawMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**
   * causes the claw motor to intake slowly to hold object
   */
  public void hold() {
    clawMotor.set(ControlMode.PercentOutput, ClawConstants.CLAW_MOTOR_HOLD_SPEED);
  }

  /**
   * @return true if game piece detected
   */
  public boolean getGamePieceSensor(){
    return !gamePieceSensor.get();
  }

}