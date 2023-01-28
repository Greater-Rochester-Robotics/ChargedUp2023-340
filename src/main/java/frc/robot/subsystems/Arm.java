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
  private CANSparkMax elbowMotorLeader;//TODO: add a secondary elbow motor then set as a follower(maybe rename to include leader)
  private CANSparkMax elbowMotorFollower;
  private AbsoluteEncoder absoluteEncoderRight;
  private AbsoluteEncoder absoluteEncoderLeft;
  private AbsoluteEncoder absoluteEncoderElbow;
  private SparkMaxPIDController shoulderRightController;
  private SparkMaxPIDController shoulderLeftController;
  private SparkMaxPIDController elbowController;
  private DoubleSolenoid wrist;
  private Solenoid brakeSolenoid;

  private double rightPos;
  private double leftPos;
  private double shoulderGoalPos;

  private boolean shoulderPIDEnable;
  

  /** Creates a new Arm. */
  public Arm() {
    //TODO:for all NEO's use enableVoltageCompensation
    //right shoulder
    shoulderRight = new CANSparkMax(Constants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
    absoluteEncoderRight = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoderRight.setPositionConversionFactor(Constants.ABS_ENC_TO_RAD_CONV_FACTOR);

    shoulderRightController.setFeedbackDevice(absoluteEncoderRight);

    shoulderRight.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    
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
    absoluteEncoderLeft.setPositionConversionFactor(Constants.ABS_ENC_TO_RAD_CONV_FACTOR);

    shoulderLeftController.setFeedbackDevice(absoluteEncoderLeft);

    shoulderLeft.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
 
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
    elbowMotorLeader = new CANSparkMax(Constants.ELBOW_MOTOR_LEADER, MotorType.kBrushless);
    elbowMotorFollower = new CANSparkMax(Constants.ELBOW_MOTOR_FOLLOWER, MotorType.kBrushless);
    absoluteEncoderElbow = elbowMotorLeader.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoderElbow.setPositionConversionFactor(Constants.ABS_ENC_TO_RAD_CONV_FACTOR);

    elbowController.setFeedbackDevice(absoluteEncoderElbow);

    elbowMotorLeader.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
 
    elbowMotorLeader.getPIDController().setP(Constants.SHOUDLER_P);
    elbowMotorLeader.getPIDController().setI(Constants.SHOUDLER_I);
    elbowMotorLeader.getPIDController().setD(Constants.SHOUDLER_D);
    elbowMotorLeader.getPIDController().setFF(Constants.SHOUDLER_F);

    elbowMotorLeader.setIdleMode(IdleMode.kBrake);

    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    //TODO: add a secondary elbow motor set to follower, don't need PID, absEnc calls, as it follows primary
    elbowMotorFollower.setIdleMode(IdleMode.kBrake);
    elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    elbowMotorFollower.follow(elbowMotorLeader);

    elbowMotorFollower.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);

    elbowMotorLeader.set(0);

    wrist = new DoubleSolenoid(Constants.ELBOW_MOTOR_LEADER, PneumaticsModuleType.REVPH, 0, 0);
    brakeSolenoid = new Solenoid(Constants.ELBOW_MOTOR_LEADER, PneumaticsModuleType.REVPH, 0);//TODO:this is the  brake to the elbow, should be named such

    //burning flash for all NEOs
    shoulderRight.burnFlash();
    shoulderLeft.burnFlash();
    elbowMotorLeader.burnFlash();
    elbowMotorFollower.burnFlash();

    //setting goal to current pos for startup
    shoulderGoalPos = absoluteEncoderRight.getPosition();

    //PID is off on startup
    shoulderPIDEnable = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightPos = absoluteEncoderRight.getPosition();
    leftPos = absoluteEncoderLeft.getPosition();

    //Checks to see if both shoulders are at the same position,
    //stops one closer to goal position
    if(shoulderPIDEnable){
      if(rightPos - leftPos < -0.19625 || rightPos - leftPos > 0.19625){
        double rightDiff = shoulderGoalPos - rightPos;
        double leftDiff = shoulderGoalPos - leftPos;

        if(Math.abs(rightDiff) > Math.abs(leftDiff)){
          shoulderLeft.set(0.0);
          shoulderRight.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
        }
        else{
          shoulderRight.set(0.0);
          shoulderLeft.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
        }
      }
      else{
        shoulderRight.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
        shoulderLeft.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
      }
    }
  }

  // -------------------------- Kinematics Methods
  //TODO: write kinematics here,  need a public position commmand
  
  // -------------------------- Shoulder Motors Methods

  /**
   * Stop both the shoulder motors
   */
  public void stopShoulder(){
    shoulderRight.set(0);
    shoulderLeft.set(0);
    shoulderPIDEnable = false;
  }

  /**
   * drive only the left shoulder motor via duty cycle
   * FOR TESTING PURPOSES ONLY
   * @param leftDutyCycle a value between -1.0 and 1.0, 0.0 is stopped
   */
  public void setLeftDutyCycle(Double leftDutyCycle){
    shoulderLeft.set(leftDutyCycle);
    shoulderPIDEnable = false;
  }

  /**
   * drive only the right shoulder motor via duty cycle
   * FOR TESTING ONLY
   * @param rightDutyCycle a value between -1.0 and 1.0, 0.0 is stopped
   */
  public void setRightDutyCycle(Double rightDutyCycle){
    shoulderLeft.set(rightDutyCycle);
    shoulderPIDEnable = false;
  }

  public void setBothShoulderMotor(double output){
    shoulderRight.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
    shoulderLeft.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
    shoulderGoalPos = output;
    shoulderPIDEnable = true;
  }

  // -------------------------- Elbow Methods

  public double getElbowPosition(){
    return absoluteEncoderElbow.getPosition();
  }
  

  /**
   * Stops the elbow and reengages the brake
   */
  public void stopElbow(){
    elbowMotorLeader.set(0);
    brakeSolenoid.set(true);
  } 

  public void setElbowDutyCycle(Double elbowDutyCycle){
    elbowMotorLeader.set(elbowDutyCycle);
    brakeSolenoid.set(false);
  }

  public void setElbowPosition(Double position){
    elbowController.setReference(position, ControlType.kPosition);
    brakeSolenoid.set(false);
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
