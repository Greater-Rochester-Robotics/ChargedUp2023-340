// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RecordPlayer extends SubsystemBase {
  private CANSparkMax rotationMotor;
  private DigitalInput gamePieceSensor;
  private DigitalInput conePositionSensor;

  /** Creates a new RecordPlayer. */
  public RecordPlayer() {
    rotationMotor = new CANSparkMax(Constants.RECORD_PLAYER_MOTOR, MotorType.kBrushless);

    gamePieceSensor = new DigitalInput(Constants.CHANNEL_IS_GAME_PIECE);
    conePositionSensor = new DigitalInput(Constants.CHANNEL_CONE_POSITION);


    rotationMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
 
    rotationMotor.getPIDController().setP(Constants.RecordPlayerConstants.RECORD_PLAYER_P);
    rotationMotor.getPIDController().setI(Constants.RecordPlayerConstants.RECORD_PLAYER_I);
    rotationMotor.getPIDController().setD(Constants.RecordPlayerConstants.RECORD_PLAYER_D);
    rotationMotor.getPIDController().setFF(Constants.RecordPlayerConstants.RECORD_PLAYER_F);

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

  public boolean getGamePieceSensor(){
    return gamePieceSensor.get();
  }

  public boolean getConePositionSensor(){
    return conePositionSensor.get();
  }

  public void setRotationMotor(double speed) {
    rotationMotor.set(speed);
  }

  public void stopRotationMotor() {
    rotationMotor.set(0);
  }

  public double getRotationMotorSpeed() {
    return rotationMotor.getEncoder().getVelocity();
  }

  public double getConeAngle(){
    //TODO use the camra
    double cameraLatency = 0.0;
    double conePosition = 0.0;
     
    return adjustConeAngle(cameraLatency, conePosition);
  }

  public double adjustConeAngle(double cameraLatency, double conePosition) {
    double distanceAhead = rotationMotor.getEncoder().getVelocity() * cameraLatency / 60000; // 60,000 is the number of milla seconds in a minute
    double position = conePosition + distanceAhead;
    return position % Constants.TWO_PI;
  }
}
