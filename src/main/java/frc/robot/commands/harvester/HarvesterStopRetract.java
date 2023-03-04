// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.recordPlayer.RecordPlayerIntake;

public class HarvesterStopRetract extends ParallelRaceGroup {
  /** Creates a new IntakeStopRetract. */
  public HarvesterStopRetract() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new RecordPlayerIntake(),
      Commands.sequence(
        new HarvesterExtensionIn(),
        new WaitCommand(1),
        new HarvesterStop()
      )
    );
  }

}
