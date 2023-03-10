// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HarvesterTillGamePiece extends CommandBase {
  /** Creates a new IntakeTillGamePiece. */
  public HarvesterTillGamePiece() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.harvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.harvester.motorIn(false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.harvester.motorStop();
    RobotContainer.harvester.harvesterExtensionIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.harvester.hasGamePiece();
  }
}
