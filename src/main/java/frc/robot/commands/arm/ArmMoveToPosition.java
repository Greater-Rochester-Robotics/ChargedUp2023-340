// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Arm.ArmTrajectory;
import edu.wpi.first.wpilibj.Timer;

public class ArmMoveToPosition extends CommandBase {
  ArmPosition goalPosition;
  List<ArmTrajectory> trajectory;
  int currentWayPoint = 0;
  Timer timer = new Timer();
  
  /** Creates a new MoveArmToPosition. */
  @Deprecated
  public ArmMoveToPosition(double x, double y) {
    this(ArmPosition.inverseKinematics(x, y));
  }
  
  @Deprecated
  public ArmMoveToPosition(ArmPosition goalPosition){
    this.goalPosition = goalPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentWayPoint = 0;
    trajectory = RobotContainer.arm.getTrajectory(RobotContainer.arm.getArmPosition(), goalPosition);
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(trajectory.get(currentWayPoint).time >= timer.get()){
      currentWayPoint++;
      if(currentWayPoint >= trajectory.size() - 1)
        return;
    }
    // gets the next arm position
    ArmPosition nextPosition = ArmPosition.interpolateArmPosition(
      trajectory.get(currentWayPoint).armPosition, // gets the last way point
      trajectory.get(currentWayPoint + 1).armPosition, // gets the next way point
      // gets the percentage of the way between the last way point and the next way point using the timer 
      (timer.get() - trajectory.get(currentWayPoint).time) / (trajectory.get(currentWayPoint + 1).time - trajectory.get(currentWayPoint).time));

    //keep making arm!!!
    RobotContainer.arm.DriveToPosition(nextPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.stopShoulder();
    RobotContainer.arm.stopElbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentWayPoint >= trajectory.size()-1;
  }
}
