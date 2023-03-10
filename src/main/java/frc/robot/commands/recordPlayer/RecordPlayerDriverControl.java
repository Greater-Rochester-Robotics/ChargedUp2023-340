// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.recordPlayer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RecordPlayerDriverControl extends CommandBase {
  /** Creates a new RecordPlayerDriverControl. */
  public RecordPlayerDriverControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.recordPlayer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotationSpeed = (Robot.robotContainer.getCoDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getCoDriverAxis(Axis.kLeftTrigger)) * 0.6;

    RobotContainer.recordPlayer.setRotationMotor(rotationSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.recordPlayer.stopRotationMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
