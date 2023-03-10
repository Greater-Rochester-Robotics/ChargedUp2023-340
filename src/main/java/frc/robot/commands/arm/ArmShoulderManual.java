// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmShoulderManual extends CommandBase {
  double rightAxis;
  double leftAxis;
  /** Creates a new ArmManualShoulder. */
  public ArmShoulderManual() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightAxis = Robot.robotContainer.getRightShoulderManual();
    leftAxis = Robot.robotContainer.getLeftShoulderManual();

    RobotContainer.arm.setRightDutyCycle(rightAxis);
    RobotContainer.arm.setLeftDutyCycle(leftAxis);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.stopShoulder();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
