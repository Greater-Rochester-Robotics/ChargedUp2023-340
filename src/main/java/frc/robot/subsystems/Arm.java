// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulderRight;
  private CANSparkMax shoulderLeft;
  private AbsoluteEncoder absoluteEncoderRight;
  private AbsoluteEncoder absoluteEncoderLeft;
  

  /** Creates a new Arm. */
  public Arm() {
    shoulderRight = new CANSparkMax(Constants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
    absoluteEncoderRight = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
    //TODO:  switch to PID based on the ABS sensor for all NEO in this class https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java#L54
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

    shoulderLeft = new CANSparkMax(Constants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
    absoluteEncoderLeft = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);
 
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


    //TODO:Set up elbow motor the same as above, and elbowBrake(singleAction)

    //TODO:Set up Wrist extension(DoubleAction Solenoid)

    //TODO:Burn all the NEO flash

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO: write a check to make sure the two shoulders are the same
  }

  public void stopShoulder(){
    shoulderRight.set(0);
    shoulderLeft.set(0);
  }

  public void leftDutyCycle(Double leftDutyCycle){
    shoulderLeft.set(leftDutyCycle);
  }

  public void rightDutyCycle(Double rightDutyCycle){
    shoulderLeft.set(rightDutyCycle);
  }

  public void setRightShoulderMotor(double output){
    shoulderRight.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);

  }
  //TODo: add a single PID set call
  public void setLeftShoulderMotor(double output){
    shoulderLeft.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);

  }

  //TODO:add elbow motor methods stop, setDutyCycle, and setPosition, remember to disable and reenable brake 

  //TODO:add wrist extension functions 
}
