// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.harvester.HarvesterExtensionIn;
import frc.robot.commands.harvester.HarvesterStop;
import frc.robot.commands.recordPlayer.RecordPlayerOrientCone;
import frc.robot.commands.recordPlayer.RecordPlayerSpin;

/**
 * Stops the harvester motors and retracts the harvester into the robot.
 */
public class HarvesterRecordRetract extends SequentialCommandGroup {
    /**
     * Creates a new HarvesterRecordRetract command.
     */
    public HarvesterRecordRetract (boolean isCone) {
        addCommands(
            // If grabbing a cone, spin the record player.s
            new ConditionalCommand(new RecordPlayerSpin(), new InstantCommand(), () -> isCone),

            // Bring in the harvester.
            new HarvesterExtensionIn(),

            // Wait for the game piece to be inside the robot.
            new WaitCommand(isCone ? 1.0 : 2.5),

            // Stop the harvester motors.
            new HarvesterStop(),

            // If we grabbed a cone, add delay to ensure the cone is in position then run the record player.
            new ConditionalCommand(new RecordPlayerOrientCone(), new InstantCommand(), () -> isCone)
        );
    }
}
