// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HarvesterIntake extends CommandBase {
  boolean isCone;
  /** Creates a new IntakeIntake. */
  public HarvesterIntake() {
    this(true);
  }
  public HarvesterIntake(boolean isCone) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isCone = isCone;
    addRequirements(RobotContainer.harvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.harvester.harvesterExtensionOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.harvester.motorIn(isCone);
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
