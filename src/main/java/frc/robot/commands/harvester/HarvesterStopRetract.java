// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.recordPlayer.RecordOrientCone;
import frc.robot.commands.recordPlayer.RecordPlayerIntake;

public class HarvesterStopRetract extends SequentialCommandGroup {
  /** Creates a new IntakeStopRetract. */
  public HarvesterStopRetract(boolean isCone) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      Commands.race(
        new RecordPlayerIntake(),
        Commands.sequence(
          new HarvesterExtensionIn(),
          Commands.race(
            // isCone?new HarvesterOuttake():new InstantCommand(),
            new WaitCommand(.5)
          ),
          new HarvesterStop()
        )
      ),
     new WaitCommand(.5),
     (isCone?new RecordOrientCone():new InstantCommand())
    );
  }

}
