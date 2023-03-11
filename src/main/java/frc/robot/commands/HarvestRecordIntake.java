// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.commands.harvester.HarvesterIntake;
import frc.robot.commands.recordPlayer.RecordPlayerIntake;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HarvestRecordIntake extends SequentialCommandGroup {
  /** Creates a new HarvistRecordIntake. */
  public HarvestRecordIntake(boolean isCone) {
    
    addCommands(
      new HarvesterExtensionOut(),
      isCone?new ClawClose():new InstantCommand(),
      Commands.parallel(
        new HarvesterIntake(isCone),
        isCone?new RecordPlayerIntake():new InstantCommand()
      )
    ); 
  }
}
