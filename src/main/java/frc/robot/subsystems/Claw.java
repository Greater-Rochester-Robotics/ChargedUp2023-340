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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private TalonSRX clawMotor;
  private Solenoid open;
  private Solenoid close;
  private DigitalInput gamePieceSensor;

  /** Creates a new Claw. */
  public Claw() {
    //Constructs the claws motors and solenoids
    //TODO:switch to a NEO550
    clawMotor = new TalonSRX(Constants.CLAW_MOTOR);
    ///TODO:set up NEO550, including a PID for basic position(likely just need P)
    //TODO:Make sure sparkmax set to brake
    //TODO:burn that flash

    //TODO: Switch back to a single object, a double solenoid
    open = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_SOLENOID_IN);
    close = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_SOLENOID_OUT);


    gamePieceSensor = new DigitalInput(Constants.CLAW_GAMEPIECE_SENSOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Claw Encoder",getEncoderPosition());
    SmartDashboard.putBoolean("Claw GamePiece", getGamePieceSensor());

  }

  /**
   * Closes the claw pneumatic
   */
  public void close(){
    //TODO: Make sure that the close and open functions drive the solenoids the right direction
    close.set(true);
    open.set(false);
  }

  /**
   * Opens the claw pneumatic
   */
  public void open(){
    //TODO: Make sure that the close and open functions drive the solenoids the right direction
    close.set(false);
    open.set(true);
  }

  /**
   * causes the claw motor to intake a cube or cone
   */
  //TODO: set the motor speeds
  public void intake(){
    clawMotor.set(TalonSRXControlMode.PercentOutput, 1);//TODO:call this as NEO550 and use a constant from the Claw constants
  }

  /**
   * cause the intake motor to spit out
   */
  public void outtake(){
    clawMotor.set(TalonSRXControlMode.PercentOutput,-1);//TODO:call this as NEO550 and use a constant from the Claw constants
  }

  public void stop(){
    clawMotor.set(TalonSRXControlMode.PercentOutput, 0.0);//TODO:call this as NEO550
  }

  /**
   * @return current value of intake encoder
   */
  public double getEncoderPosition(){
    return 0.0;//TODO: Write me
  }

  /**
   * @param position 
   */
  public void setIntakePosition(double position){
    //TODO: Write me
  }

  /**
   * @return true if game piece detected
   */
  public boolean getGamePieceSensor(){
    return gamePieceSensor.get();
  }
}