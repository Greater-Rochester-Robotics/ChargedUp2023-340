// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ClawIntake extends InstantCommand {
  /** Creates a new ClawIntake. */
  public ClawIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.claw.intake();
  }
}
