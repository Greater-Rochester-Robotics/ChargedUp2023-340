// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.ArmTrajectory;
import edu.wpi.first.wpilibj.Timer;

public class MoveArmToPosition extends CommandBase {
  ArmPosition goalPosition;
  List<ArmTrajectory> trajectory;
  int currentWayPoint = 0;
  Timer timer = new Timer();
  
  /** Creates a new MoveArmToPosition. */
  public MoveArmToPosition(double x, double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
    goalPosition = RobotContainer.arm.inverseKinematics(x, y);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    trajectory = RobotContainer.arm.getTrajectory(RobotContainer.arm.getArmPosition(), goalPosition);
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(trajectory.get(currentWayPoint).time <= timer.get()){
      currentWayPoint++;
      if(currentWayPoint >= trajectory.size())
        return;
    }
    //keep making arm!!!
    RobotContainer.arm.DriveToPosition(trajectory.get(currentWayPoint).armPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
