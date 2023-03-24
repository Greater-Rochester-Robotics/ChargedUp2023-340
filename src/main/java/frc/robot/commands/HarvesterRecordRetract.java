// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.harvester.HarvesterExtensionIn;
import frc.robot.commands.harvester.HarvesterStop;
import frc.robot.commands.recordPlayer.RecordPlayerOrientCone;

/**
 * Stops the harvester motors and retracts the harvester into the robot.
 */
public class HarvesterRecordRetract extends SequentialCommandGroup {
    public HarvesterRecordRetract (boolean isCone) {
        this(isCone, 1, 2.5);
    }

    /**
     * Creates a new HarvesterRecordRetract command.
     */
    public HarvesterRecordRetract (boolean isCone, double intakeDelay, double recordPlayerDelay) {
        addCommands(
            // Bring in the harvester.
            new HarvesterExtensionIn(),

            // Add delay to ensure the game piece is in the record player.
            new WaitCommand(intakeDelay),

            // Stop the harvester motors.
            new HarvesterStop(),

            // If we grabbed a cone, add delay to ensure the cone is in position then run the record player.
            new ConditionalCommand(
                Commands.sequence(
                    new WaitCommand(recordPlayerDelay),
                    new RecordPlayerOrientCone()
                ),
                new InstantCommand(),
                () -> isCone
            )
        );
    }
}
