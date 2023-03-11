// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;

public class ArmMoveShoulderElbow extends CommandBase {
  ArmPosition targetPosition;
  boolean shoulderOnTarget;
  int elbowOnTarget;
  /** Creates a new ArmMoveShoulderElbow. */
  public ArmMoveShoulderElbow(ArmPosition targetPosition) {
    this.targetPosition = targetPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbowOnTarget = 0;
    shoulderOnTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shoulderOnTarget = Math.abs(RobotContainer.arm.getShoulderPosition()-targetPosition.getShoulderPosition()) < ArmConstants.SHOULDER_CLOSED_LOOP_ERROR;
    if(shoulderOnTarget){
      RobotContainer.arm.stopShoulder();
    }else{
      RobotContainer.arm.setBothShoulderMotorPosition(targetPosition.getShoulderPosition());
    }

    if( Math.abs(RobotContainer.arm.getElbowPosition()) > ArmConstants.MAX_ELBOW_ANGLE){
      elbowOnTarget = 10;
    }else if(Math.abs(RobotContainer.arm.getElbowPosition()-targetPosition.getElbowPosition()) < ArmConstants.ELBOW_CLOSED_LOOP_ERROR){
      elbowOnTarget++;
    }else{
      elbowOnTarget = 0;
    }
    if(elbowOnTarget == 10){
      RobotContainer.arm.stopElbow();
    }else{
      RobotContainer.arm.setElbowPosition(targetPosition.getElbowPosition());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.stopElbow();
    RobotContainer.arm.stopShoulder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shoulderOnTarget && (elbowOnTarget > 10);
  }
}
