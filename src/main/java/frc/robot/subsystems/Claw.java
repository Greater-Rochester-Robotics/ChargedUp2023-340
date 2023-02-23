// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;
  private DoubleSolenoid openClose;
  private DigitalInput gamePieceSensor;

  /** Creates a new Claw. */
  public Claw() {
    //Constructs the claws motors and solenoid
    clawMotor = new CANSparkMax(Constants.CLAW_MOTOR, MotorType.kBrushless);
    clawMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    
    clawMotor.getPIDController().setP(ClawConstants.CLAW_P);  
    clawMotor.getPIDController().setI(ClawConstants.CLAW_I);
    clawMotor.getPIDController().setD(ClawConstants.CLAW_D);
    clawMotor.getPIDController().setFF(ClawConstants.CLAW_F);

    clawMotor.setIdleMode(IdleMode.kBrake);

    clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    clawMotor.burnFlash();

    openClose = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLAW_SOLENOID_OUT, Constants.CLAW_SOLENOID_IN); //open is in is reverse, close is out is forward

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
    openClose.set(Value.kForward);
  }

  /**
   * Opens the claw pneumatic
   */
  public void open(){
    openClose.set(Value.kReverse);
  }

  /**
   * causes the claw motor to intake a cube or cone
   */
  public void intake(){
    clawMotor.set(ClawConstants.CLAW_MOTOR_INTAKE_SPEED);
  }

  /**
   * cause the intake motor to spit out
   */
  public void outtake(){
    clawMotor.set(ClawConstants.CLAW_MOTOR_OUTTAKE_SPEED);
  }

  public void stop(){
    clawMotor.set(0.0);
  }

  /**
   * @return current value of intake encoder in native units
   */
  public double getEncoderPosition() {
    return clawMotor.getEncoder().getPosition();
  }

  /**
   * Sets intake to a position
   * @param position position to set intake to
   */
  public void setIntakePosition(double position){
    clawMotor.getEncoder().setPosition(position);
  }

  /**
   * @return true if game piece detected
   */
  public boolean getGamePieceSensor(){
    return gamePieceSensor.get();
  }
}