// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import static frc.robot.Constants.HarvesterConstants.*;

public class Harvester extends SubsystemBase {
  private TalonSRX harvesterMotor;
  // private CANSparkMax motor2;
  private DigitalInput gamePieceSensor;
  private DoubleSolenoid harvesterPistons;
  Timer rumbleTimer;
  private boolean hadGamePeice = false;
  private boolean isHarvesterOut;

  /**
   * The network table instance used by the harvester subsystem.
   */
  private NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
  /**
   * The network table used by the harvester subsystem.
   */
  private NetworkTable netTable = netInstance.getTable("/dashboard/robotmodel");

  /** Creates a new Intake. */
  public Harvester() {
    rumbleTimer = new Timer();
    rumbleTimer.start();

    harvesterMotor = new TalonSRX(Constants.HARVESTER_MOTOR);
    harvesterMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    harvesterMotor.enableVoltageCompensation(true);
    
    harvesterMotor.setNeutralMode(NeutralMode.Brake);

    harvesterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    harvesterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 251);
    harvesterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 241);
    harvesterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 239);
    harvesterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 233);
    harvesterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 229);
    harvesterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 227);

    harvesterMotor.configOpenloopRamp(.25);
    
    gamePieceSensor = new DigitalInput(Constants.HARVESTER_GAME_PIECE_SENSOR);

    harvesterPistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.HARVESTER_SOLENOID_OUT, Constants.HARVESTER_SOLENOID_IN);

    isHarvesterOut = harvesterPistons.get() == Value.kForward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake has game peice: ", hasGamePiece());

    netTable.getEntry("harvester").setBoolean(isHarvesterOut());
    // if(hadGamePeice && !hasGamePiece())
    //   rumbleTimer.reset();

    // Robot.robotContainer.notifyDriver(!rumbleTimer.hasElapsed(1)&&isHarvesterOut());

    // hadGamePeice = hasGamePiece();
  }

  /**
   * Move the intake into the robot.
   */
  public void harvesterExtensionIn() {
    harvesterPistons.set(Value.kReverse);
    isHarvesterOut = harvesterPistons.get() == Value.kForward;
  }

  /**
   * moves the intake out of the robot
   */
  public void harvesterExtensionOut() {
    harvesterPistons.set(Value.kForward);
    isHarvesterOut = harvesterPistons.get() == Value.kForward;
  }

  public boolean isHarvesterOut(){
    return harvesterPistons.get() == Value.kForward;
  }

  /**
   * run the intake motor, to bring a gamepiece into the robot
   */
  public void motorIn(boolean isCone) {
    harvesterMotor.set(TalonSRXControlMode.PercentOutput, isCone? HARVESTER_MOTOR_INTAKE_SPEED : HARVESTER_MOTOR_CUBE_SPEED);
  }

  /**
   * runs the intake motor, to push a gamepiece out of the robot
   */
  public void motorOut() {
    harvesterMotor.set(TalonSRXControlMode.PercentOutput, HARVESTER_MOTOR_OUTTAKE_SPEED);
  }

  /**
   * stops the intake motor
   */
  public void motorStop() {
    harvesterMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /**
   * a sensor in the intake to determine if there is a gamepiece in the intake.
   * @return boolean
   */
  public boolean hasGamePiece(){
    return !gamePieceSensor.get();
  }

}
