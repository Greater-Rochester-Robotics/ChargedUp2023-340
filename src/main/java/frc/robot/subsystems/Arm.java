// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.SheetCollate;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulderRight;
  private CANSparkMax shoulderLeft;
  private CANSparkMax elbowMotor;//TODO: add a secondary elbow motor then set as a follower(maybe rename to include leader)
  private AbsoluteEncoder absoluteEncoderRight;
  private AbsoluteEncoder absoluteEncoderLeft;
  private AbsoluteEncoder absoluteEncoderElbow;
  private AbsoluteEncoder absoluteEncoderWrist;
  private SparkMaxPIDController shoulderRightController;
  private SparkMaxPIDController shoulderLeftController;
  private SparkMaxPIDController elbowController;
  private DoubleSolenoid wrist;
  private Solenoid wristBrakeSolenoid;
  

  /** Creates a new Arm. */
  public Arm() {
    //TODO:for all NEO's use enableVoltageCompensation
    //right shoulder
    shoulderRight = new CANSparkMax(Constants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
    absoluteEncoderRight = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);

    shoulderRightController.setFeedbackDevice(absoluteEncoderRight);
    
    shoulderRight.getPIDController().setP(Constants.SHOUDLER_P);  
    shoulderRight.getPIDController().setI(Constants.SHOUDLER_I);
    shoulderRight.getPIDController().setD(Constants.SHOUDLER_D);
    shoulderRight.getPIDController().setFF(Constants.SHOUDLER_F);

    shoulderRight.setIdleMode(IdleMode.kBrake);

    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    //left shoulder
    shoulderLeft = new CANSparkMax(Constants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
    absoluteEncoderLeft = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);

    shoulderLeftController.setFeedbackDevice(absoluteEncoderLeft);
 
    shoulderLeft.getPIDController().setP(Constants.SHOUDLER_P);
    shoulderLeft.getPIDController().setI(Constants.SHOUDLER_I);
    shoulderLeft.getPIDController().setD(Constants.SHOUDLER_D);
    shoulderLeft.getPIDController().setFF(Constants.SHOUDLER_F);

    shoulderLeft.setIdleMode(IdleMode.kBrake);

    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);


    //elbow
    elbowMotor = new CANSparkMax(Constants.WRIST_MOTOR, MotorType.kBrushless);
    absoluteEncoderElbow = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

    elbowController.setFeedbackDevice(absoluteEncoderElbow);
 
    elbowMotor.getPIDController().setP(Constants.SHOUDLER_P);
    elbowMotor.getPIDController().setI(Constants.SHOUDLER_I);
    elbowMotor.getPIDController().setD(Constants.SHOUDLER_D);
    elbowMotor.getPIDController().setFF(Constants.SHOUDLER_F);

    elbowMotor.setIdleMode(IdleMode.kBrake);

    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    elbowMotor.set(0);

    wrist = new DoubleSolenoid(Constants.WRIST_MOTOR, PneumaticsModuleType.REVPH, 0, 0);
    wristBrakeSolenoid = new Solenoid(Constants.WRIST_MOTOR, PneumaticsModuleType.REVPH, 0);

    //burning flash for all NEOs
    shoulderRight.burnFlash();
    shoulderLeft.burnFlash();
    elbowMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO: write a check to make sure the two shoulders are the same
  }

  // -------------------------- Shoulder Motors Methods

  /**
   * Stop both the shoulder motors
   */
  public void stopShoulder(){
    shoulderRight.set(0);
    shoulderLeft.set(0);
  }

  /**
   * drive only the left shoulder motor via duty cycle
   * @param leftDutyCycle a value between -1.0 and 1.0, 0.0 is stopped
   */
  public void setLeftDutyCycle(Double leftDutyCycle){
    shoulderLeft.set(leftDutyCycle);
  }

  /**
   * drive only the rgith shoulder motor via duty cycle
   * @param rightDutyCycle a value between -1.0 and 1.0, 0.0 is stopped
   */
  public void setRightDutyCycle(Double rightDutyCycle){
    shoulderLeft.set(rightDutyCycle);
  }

  public void setRightShoulderMotor(double output){
    shoulderRight.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
  }
  //TODO: add a single PID set call for both, not independent methods
  public void setLeftShoulderMotor(double output){
    shoulderLeft.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
  }

  // -------------------------- Elbow Methods

  //TODO: write getElbowPosition, pull value from the absolute encoder

  /**
   * Stops the elbow and reengages the brake
   */
  public void stopElbow(){
    elbowMotor.set(0);
    //TODO:remember to disable and reenable brake 
  } 

  public void setElbowDutyCycle(Double elbowDutyCycle){
    elbowMotor.set(elbowDutyCycle);
    //TODO:remember to disable and reenable brake 
  }

  public void setElbowPosition(Double position){
    elbowController.setReference(position, ControlType.kPosition);
    //TODO:remember to disable and reenable brake 
  }
  
  // -------------------------- Wrist Methods

  /**
   * extends the wrist pneumatic piston
   */
  public void extendWrist(){
    wrist.set(Value.kForward);
  }

  /**
   * retracts the wrist pneumatic piston
   */
  public void retractWrist(){
    wrist.set(Value.kReverse);
  }
}
