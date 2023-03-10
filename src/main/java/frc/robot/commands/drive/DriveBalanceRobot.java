// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;

/**
 * Drive up ramp until balanced
 * relies upon pitch and roll angles and velocities
 * Stops moving when robot is coming to a balance
 */
public class DriveBalanceRobot extends CommandBase {
  private double currentAngle = 0;
  private double prevPitchAngle = 0;
  private double prevRollAngle = 0;
  private double prevTime = 0;
  private PIDController pidX;
  private PIDController pidY;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveBalanceRobot() {
    addRequirements(RobotContainer.swerveDrive);
    //Instantiats and sets the tolerence for both pid controllers
    pidX = new PIDController(0.25, 0.0, 0.0); 
    // pidX.setTolerance(5);
    pidY = new PIDController(0.25, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.swerveDrive.setIsOdometry(false);
    currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
    pidX.reset();
    pidY.reset();
    prevPitchAngle = RobotContainer.swerveDrive.getGyroInDegPitch();
    prevRollAngle = RobotContainer.swerveDrive.getGyroInDegRoll();
    prevTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = 0.0;
    double strafeSpeed = 0.0;

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

    //if we are facing up and the ramp is moving down (or vice versa) we are coming to balance so stop moving
    if(Math.abs(pitchVel) > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE || 
        Math.abs(pitchAngle) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE){
      //Math.signum(pitchAng) * Math.signum(pitchVel) < 0) {//
      forwardSpeed = 0.0;
    } else {
      forwardSpeed = pidX.calculate(pitchAngle, 0.0);
    }

    //same thing for roll
    if(Math.abs(rollVel) > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE || 
        Math.abs(rollAngle) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE){
      strafeSpeed = 0.0;
    } else {
      strafeSpeed = pidY.calculate(rollAngle, 0.0);
    }

    //puts the value of forward speed between maxSpeed and -maxSpeed
    if(forwardSpeed > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
      forwardSpeed = SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
    }else if(forwardSpeed < -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
      forwardSpeed = -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
    }

    //puts the value of strafe speed between maxSpeed and -maxSpeed
    if(strafeSpeed > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
      strafeSpeed = SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
    }else if(strafeSpeed < -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
      strafeSpeed = -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
    }
    // if(Math.signum(rollAng) * Math.signum(rollVel) < 0) {
    //   strafeSpeed = 0.0;
    // } else {
    //   strafeSpeed = pidY.calculate(rollAng, 0.0);
    // }
    // System.out.println("pitch: " + Math.round(pitchAngle) + "   PitchSpeed: " + pitchVel + "     Output: "+forwardSpeed);
    // TODO: change to field centric and allow driver to move side to side
    //moves the robot using driveRobotCentric
    RobotContainer.swerveDrive.driveRobotCentric(
      forwardSpeed * -1.0,
      strafeSpeed * -1.0,
      RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle),
      false,
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