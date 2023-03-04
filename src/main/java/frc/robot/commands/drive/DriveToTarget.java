// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TargetConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Target;
import frc.robot.subsystems.Target.GoalLocation;

public class DriveToTarget extends CommandBase {
  /** Creates a new DriveToTarget. */
  public DriveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  double currentAngle;
  double angle;
  GoalLocation currentTarget;
  Alliance currentAlliance;
  int onTargetCount = 0;
  int[] target;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = RobotContainer.swerveDrive.getGyroInDegYaw();
    currentTarget = RobotContainer.target.getTargetPosition();
    currentAlliance = DriverStation.getAlliance();
    target = RobotContainer.target.getTarget();
    // column = 0 or 2, row = 2
    if ((target[1] == 0 || target[1] == 2) && target[2] == 0) {
      angle = Math.round(currentAngle / Math.PI * 2) * Math.PI * 2;
    }else{
      angle = Math.round(currentAngle / Math.PI) * Math.PI;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
    double output = RobotContainer.swerveDrive.getRobotRotationPIDOut(angle);
    // System.out.println("pid output"+output);
    //Turns the robot
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, output, false, true);
    //If the robot is facing the target within a certain tolerence than it increses the onTargetCount if not it sets the onTargetCount to 0
    if(Math.abs(angle - RobotContainer.swerveDrive.getGyroInRadYaw()) < .03){
      onTargetCount++;
    }else{
      onTargetCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetCount >= 10;
  }
}
