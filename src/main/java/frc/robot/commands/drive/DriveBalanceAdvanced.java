// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;

/**
 * Drive up ramp until balanced
 * relies upon pitch and roll angles and velocities
 * Stops moving when robot is coming to a balance
 * driver has lateral control
 */
public class DriveBalanceAdvanced extends CommandBase {
  private double currentAngle = 0;
  private double prevPitchAngle = 0;
  private double prevRollAngle = 0;
  private double prevTime = 0;
  private PIDController pidX;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveBalanceAdvanced() {
    addRequirements(RobotContainer.swerveDrive);
    pidX = new PIDController(0.25, 0.0, 0.0); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.swerveDrive.setIsOdometry(false);
    currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
    pidX.reset();
    prevPitchAngle = RobotContainer.swerveDrive.getGyroInDegPitch();
    prevRollAngle = RobotContainer.swerveDrive.getGyroInDegRoll();
    prevTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double awaySpeed = 0.0;

    //Gets the pitch yaw and their velocity's
    double pitchAngle = RobotContainer.swerveDrive.getGyroInDegPitch();
    double rollAngle = RobotContainer.swerveDrive.getGyroInDegRoll();

    double time = Timer.getFPGATimestamp();
    double deltaTime = time - prevTime;
    double pitchVel = (pitchAngle - prevPitchAngle)/deltaTime;
    double rollVel = (rollAngle - prevRollAngle)/deltaTime;

    prevTime = time;
    prevPitchAngle = pitchAngle;
    prevRollAngle = rollAngle;

    Rotation2d botRot = RobotContainer.swerveDrive.getGyroRotation2d();

    double awayAngle = (pitchAngle*botRot.getCos()) + (rollAngle*botRot.getSin());
    double awayVel = Math.sqrt((pitchVel*pitchVel) + (rollVel*rollVel));//(pitchVel*botRot.getCos()) + (rollVel*botRot.getSin());

    //if we are facing up and the ramp is moving down (or vice versa) we are coming to balance so stop moving
    if(Math.abs(awayVel) > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE || 
        Math.abs(awayAngle) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE){
      //Math.signum(pitchAng) * Math.signum(pitchVel) < 0) {//
      awaySpeed = 0.0;
    } else {
      awaySpeed = pidX.calculate(pitchAngle, 0.0);
    }

    //puts the value of forward speed between maxSpeed and -maxSpeed
    if(awaySpeed > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
      awaySpeed = SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
    }else if(awaySpeed < -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
      awaySpeed = -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
    }


    // if(Math.signum(rollAng) * Math.signum(rollVel) < 0) {
    //   strafeSpeed = 0.0;
    // } else {
    //   strafeSpeed = pidY.calculate(rollAng, 0.0);
    // }

    double lateralSpeed = Robot.robotContainer.getRobotLateralFull(false);
    if(Robot.robotContainer.getDriverButton(9)){
      lateralSpeed *= 0.5;
    }
    
    
    //moves the robot using driveRobotCentric
    RobotContainer.swerveDrive.driveFieldRelative(
      awaySpeed * -1.0,
      lateralSpeed,
      RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle),
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}