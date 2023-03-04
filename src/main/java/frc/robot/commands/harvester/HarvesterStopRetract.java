// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HarvesterStopRetract extends SequentialCommandGroup {
  /** Creates a new IntakeStopRetract. */
  public HarvesterStopRetract() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new HarvesterExtensionIn(),
      new WaitCommand(1),
      new HarvesterStop()
    );
  }

}
