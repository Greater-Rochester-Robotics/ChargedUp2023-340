// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.recordPlayer.RecordPlayerOrientCone;
import frc.robot.commands.recordPlayer.RecordPlayerSpin;

/**
 * Stops the harvester motors and retracts the harvester into the robot.
 */
public class HarvesterStopRetract extends SequentialCommandGroup {
    /**
     * Creates a new HarvesterStopRetract command.
     */
    public HarvesterStopRetract (boolean isCone) {
        addCommands(
            // Bring in the harvester extension, delay to ensure the game piece is entirely inside the robot, then stop the harvester motors.
            // If grabbing a cone, start running the record player at the same time.
            Commands.race(
                Commands.sequence(
                    new HarvesterExtensionIn(),
                    new WaitCommand(isCone ? 0.5 : 2.5),
                    new HarvesterStop()
                ),
                new ConditionalCommand(new RecordPlayerSpin(), new InstantCommand(), () -> isCone)
            ),

            // If we grabbed a cone, add delay to ensure the cone is in position then run the record player.
            new ConditionalCommand(
                Commands.sequence(
                    new WaitCommand(0.5),
                    new RecordPlayerOrientCone()
                ),
                new InstantCommand(),
                () -> isCone
            )
        );
    }
}
