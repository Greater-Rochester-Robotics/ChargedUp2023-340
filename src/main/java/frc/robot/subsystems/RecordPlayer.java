// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RecordPlayer extends SubsystemBase {
  private CANSparkMax rotationMotor;

  /** Creates a new RecordPlayer. */
  public RecordPlayer() {
    rotationMotor = new CANSparkMax(Constants.RECORD_PLAYER_MOTOR, MotorType.kBrushless);

    rotationMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
 
    rotationMotor.getPIDController().setP(Constants.RECORD_PLAYER_P);
    rotationMotor.getPIDController().setI(Constants.RECORD_PLAYER_I);
    rotationMotor.getPIDController().setD(Constants.RECORD_PLAYER_D);
    rotationMotor.getPIDController().setFF(Constants.RECORD_PLAYER_F);

    rotationMotor.setIdleMode(IdleMode.kBrake);

    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}