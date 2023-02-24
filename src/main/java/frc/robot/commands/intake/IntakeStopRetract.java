// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeStopRetract extends SequentialCommandGroup {
  /** Creates a new IntakeStopRetract. */
  public IntakeStopRetract() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new IntakeExtensionIn(),
      new IntakeStop()
    );
  }

}
