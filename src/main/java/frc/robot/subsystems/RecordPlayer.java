// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RecordPlayerConstants;

public class RecordPlayer extends SubsystemBase {
  private CANSparkMax rotationMotor;
  private DigitalInput gamePieceSensor;
  private DigitalInput conePositionSensor0;
  private DigitalInput conePositionSensor1;

  /**
   * The network table instance used by the record player subsystem.
   */
  private NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
  /**
   * The network table used by the record player subsystem.
   */
  private NetworkTable netTable = netInstance.getTable("/dashboard/robotmodel");

  /** Creates a new RecordPlayer. */
  public RecordPlayer() {
    rotationMotor = new CANSparkMax(Constants.RECORD_PLAYER_MOTOR, MotorType.kBrushless);

    gamePieceSensor = new DigitalInput(Constants.RECORD_IS_GAME_PIECE);
    conePositionSensor0 = new DigitalInput(Constants.RECORD_CONE_POSITION_0);
    conePositionSensor1 = new DigitalInput(Constants.RECORD_CONE_POSITION_1);

    rotationMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
 
    rotationMotor.getPIDController().setP(Constants.RecordPlayerConstants.RECORD_PLAYER_P);
    rotationMotor.getPIDController().setI(Constants.RecordPlayerConstants.RECORD_PLAYER_I);
    rotationMotor.getPIDController().setD(Constants.RecordPlayerConstants.RECORD_PLAYER_D);
    rotationMotor.getPIDController().setFF(Constants.RecordPlayerConstants.RECORD_PLAYER_F);
    rotationMotor.getPIDController().setOutputRange(-RecordPlayerConstants.ROTATE_MOTOR_SPEED, RecordPlayerConstants.ROTATE_MOTOR_SPEED);
    rotationMotor.setClosedLoopRampRate(1);

    rotationMotor.getEncoder().setPositionConversionFactor(.01);
    rotationMotor.getEncoder().setVelocityConversionFactor(.01/60);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    rotationMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("record player has cone: ", getConePositionSensor0());
    SmartDashboard.putBoolean("record player has game peice: ", getGamePieceSensor());
    SmartDashboard.putNumber("record player angle",getEncoderAngle());
    Command curCommand = this.getCurrentCommand();
    if(curCommand == null){
      SmartDashboard.putString("RecordPlayer Current Command", "noCommand");
    }else{
      SmartDashboard.putString("RecordPlayer Current Command", curCommand.getName());
    }

    netTable.getEntry("recordplayer").setDouble(rotationMotor.getEncoder().getVelocity());
  }

  public boolean getGamePieceSensor(){
    return gamePieceSensor.get();
  }

  public boolean getConePositionSensor0(){
    return conePositionSensor0.get();
  }
  
  public boolean getConePositionSensor1(){
    return conePositionSensor1.get();
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

  public double getEncoderAngle(){
    return rotationMotor.getEncoder().getPosition();
  }

  public void rotateToAngle(double angle){
    rotationMotor.getPIDController().setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  // public double getConeAngle(){
  //   //TODO use the camera
  //   double cameraLatency = 0.0;
  //   double conePosition = 0.0;
     
  //   return adjustConeAngle(cameraLatency, conePosition);
  // }

  // public double adjustConeAngle(double cameraLatency, double conePosition) {
  //   double distanceAhead = rotationMotor.getEncoder().getVelocity() * cameraLatency / 60000; // 60,000 is the number of milla seconds in a minute
  //   double position = conePosition + distanceAhead;
  //   return position % Constants.TWO_PI;
  // }
}
