// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

/**
 * Drive up ramp until balanced
 * relies upon pitch and roll angles and velocities
 * Stops moving when robot is coming to a balance
 * driver only maintains ability to move left and right
 */

public class DriveBalanceRobot extends CommandBase {
  private double currentAngle = 0;
  private double prevPitchAng = 0;
  private double prevTime = 0;
  private PIDController pidX;
  private ProfiledPIDController pidY;
  private double maxSpeed = .2;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveBalanceRobot() {
    addRequirements(RobotContainer.swerveDrive);
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.4, 5.0);
    //Instantiats and sets the tolerence for both pid controllers
    pidX = new PIDController(0.25, 0.0, 0.0); //TODO: Tune all pid and tolerance values
    // pidX.setTolerance(5);
    pidY = new ProfiledPIDController(0.25, 0.0, 0.0, constraints);
    pidY.setTolerance(2.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.swerveDrive.setIsOdometry(false);
    currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
    pidX.reset();
    pidY.reset(RobotContainer.swerveDrive.getGyroInDegRoll());
    prevPitchAng = RobotContainer.swerveDrive.getGyroInDegPitch();
    prevTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets the pitch yaw and their velocity's
    double pitchAng = RobotContainer.swerveDrive.getGyroInDegPitch();
    double time = Timer.getFPGATimestamp();
    double pitchVel = (pitchAng - prevPitchAng)/(time - prevTime);
    prevTime = time;
    double rollAng = RobotContainer.swerveDrive.getGyroInDegRoll();
    double rollVel = RobotContainer.swerveDrive.getRotationalVelocityRoll();
    double forwardSpeed = 0.0;
    double strafeSpeed = 0.0;
    prevTime = time;
    prevPitchAng = pitchAng;
    //if we are facing up and the ramp is moving down (or vice versa) we are coming to balance so stop moving
    if(Math.abs(pitchVel) > 8 || Math.abs(pitchAng) < 5){//Math.signum(pitchAng) * Math.signum(pitchVel) < 0) {//
      forwardSpeed = 0.0;
    } else {
      forwardSpeed = pidX.calculate(pitchAng, 0.0);
    }

    if(forwardSpeed>maxSpeed){
      forwardSpeed = maxSpeed;
    }else if(forwardSpeed < -maxSpeed){
      forwardSpeed = -maxSpeed;
    }
    // if(Math.signum(rollAng) * Math.signum(rollVel) < 0) {
    //   strafeSpeed = 0.0;
    // } else {
    //   strafeSpeed = pidY.calculate(rollAng, 0.0);
    // }
    System.out.println("pitch: " + Math.round(pitchAng) + "   PitchSpeed: " + pitchVel + "     Output: "+forwardSpeed);
    // TODO: allow driver to move side to side
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