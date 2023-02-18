// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//TODO: write this once we know claw
public class Claw extends SubsystemBase {

  private TalonSRX clawMotor;
  private Solenoid open;
  private Solenoid close;
  private DigitalInput gamePieceSensor;

  /** Creates a new Claw. */
  public Claw() {
    //Constructs the claws motors and solenoids
    clawMotor = new TalonSRX(Constants.CLAW_MOTOR);
    open = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_SOLENOID_IN);
    close = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_SOLENOID_OUT);
    gamePieceSensor = new DigitalInput(Constants.CHANNEL_CLAW_INPUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //TODO: Make sure that the close and open functions drive the solenoids the right direction
  public void closeHard(){
    close.set(true);
    open.set(false);
  }

  public void closeLight(){
    close.set(false);
    open.set(false);
  }

  public void open(){
    close.set(false);
    open.set(true);
  }

  //TODO: set the motor speeds
  public void intake(){
    clawMotor.set(TalonSRXControlMode.PercentOutput, 1);
  }

  public void outtake(){
    clawMotor.set(TalonSRXControlMode.PercentOutput,-1);
  }

  public void stop(){
    clawMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public boolean getGamePieceSensor(){
    return gamePieceSensor.get();
  }
}